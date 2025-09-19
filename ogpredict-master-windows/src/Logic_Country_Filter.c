/*
  OGpredict — extensions to Gpredict for operations planning

  Copyright (C) 2025 Axel Osika <osikaaxel@gmail.com>

  This file is part of OGpredict, a derivative of Gpredict.

  OGpredict is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.  See the GNU General Public License for details.
*/

/* SPDX-License-Identifier: GPL-2.0-or-later */





/*
 * Logic_Country_Filter.c
 * -------
 * Simplified tile loader and filter: ignores the original ID column,
 * and builds a single global list of polygons (4 corners each).
 */

#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 700   /* expose strptime() */
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE         /* expose timegm() */
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include "Logic_Country_Filter.h"

/* ===== Windows / MinGW compatibility shims for time functions ===== */
#ifdef _WIN32
#  include <stdio.h>   /* sscanf */
#  include <string.h>  /* strcmp */
  /* Map POSIX timegm() to Windows _mkgmtime() */
#  ifndef timegm
#    define timegm _mkgmtime
#  endif
  /* gmtime_r() wrapper using gmtime_s() */
  static inline struct tm* gmtime_r(const time_t *t, struct tm *res) {
      return (gmtime_s(res, t) == 0) ? res : NULL;
  }
  /* Minimal strptime() for "%Y-%m-%d %H:%M:%S" only */
  static inline char* strptime_win(const char *s, const char *fmt, struct tm *tm) {
      if (!s || !fmt || !tm) return NULL;
      if (strcmp(fmt, "%Y-%m-%d %H:%M:%S") != 0) return NULL;
      int Y,M,D,h,m,sec;
      if (sscanf(s, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec) != 6) return NULL;
      tm->tm_year = Y - 1900; tm->tm_mon = M - 1; tm->tm_mday = D;
      tm->tm_hour = h; tm->tm_min = m; tm->tm_sec = sec; tm->tm_isdst = -1;
      return (char*)(s + 19); /* "YYYY-MM-DD HH:MM:SS" */
  }
#  define strptime strptime_win
#endif
/* ===== end Windows shims ===== */


/* Global list of all tile polygons loaded from the CSV. */
static GList *all_polygons  = NULL;
static GList *all_countries = NULL;

/* -------- Rectangular fast-path metadata -------- */
/**
 * TileRect
 * Axis-aligned rectangle bounds for a tile; used for O(1) point-in-rectangle tests.
 * lat_min/lat_max, lon_min/lon_max are read from CSV-derived corners.
 */
typedef struct {
    double lat_min, lat_max;
    double lon_min, lon_max;
} TileRect;

/* Map original polygon pointer -> TileRect* (owned here) */
/**
 * rect_meta (polygon → rectangle metadata)
 * Key: GArray* polygon (the 4-corner storage).
 * Val: TileRect* bounds (owned here).
 * Enables constant-time rectangle containment instead of polygon ray casting.
 */
static GHashTable *rect_meta = NULL;  /* key: GArray* ; val: TileRect* */

/* small epsilon for inclusive range tests */
/**
 * EPS (inclusive tolerance)
 * Used to include boundary points and avoid floating-point flicker at edges.
 */
#define EPS 1e-12

/* Normalize longitude to [-180,180) */
static inline double norm_lon(double lon) {
    double x = fmod(lon + 180.0, 360.0);
    if (x < 0) x += 360.0;
    return x - 180.0;
}

/* Point-in-rectangle, handling potential anti-meridian crossing */
/**
 * rect_contains(r, lat, lon)
 * Constant-time point-in-rectangle test with dateline support.
 * - Test latitude interval first.
 * - Normalize longitudes; if [lon_min, lon_max] wraps (a>b), use union [a,180) ∪ [-180,b].
 */
static inline gboolean rect_contains(const TileRect *r, double lat, double lon) {
    if (lat < r->lat_min - EPS || lat > r->lat_max + EPS) return FALSE;
    double a = norm_lon(r->lon_min), b = norm_lon(r->lon_max), L = norm_lon(lon);
    if (a <= b) {
        return (L >= a - EPS && L <= b + EPS);
    } else {
        /* rectangle spans the dateline: interval is [a,180) U (-180,b] */
        return (L >= a - EPS || L <= b + EPS);
    }
}


/* -------------------- Spatial Grid (1° cells) -------------------- */
/**
 * CellKey {r,c}
 * Grid cell coordinates for the equirectangular index (rows by latitude, columns by longitude).
 */
typedef struct { int r, c; } CellKey;
/**
 * grid (spatial index)
 * HashMap from CellKey* to GPtrArray* of polygons (GArray*).
 * Each bucket stores pointers to polygons whose rectangles overlap that cell.
 */
static GHashTable *grid = NULL;  /* key: CellKey*, val: GPtrArray* of GArray* polygons */
static const double CELL_DEG = 1.0;

/**
 * cell_hash(key)
 * Hash function for CellKey using a multiplicative mix (Knuth).
 */
static guint cell_hash(gconstpointer key) {
    const CellKey *k = (const CellKey*)key;
    /* mix r/c; Knuth multiplicative hash */
    return 2654435761u * (guint)(k->r ^ (k->c << 16));
}
/**
 * cell_equal(a,b)
 * Equality predicate for CellKey (row and column must match).
 */
static gboolean cell_equal(gconstpointer a, gconstpointer b) {
    const CellKey *x = (const CellKey*)a, *y = (const CellKey*)b;
    return x->r == y->r && x->c == y->c;
}
/**
 * latlon_to_cell(lat, lon, &r, &c)
 * Map geographic coordinates to a grid cell:
 * - Clamp latitude to [-90,90] for safety.
 * - Normalize longitude to [-180,180).
 * - Convert to row/col via floor; clamp indices to grid bounds.
 */
static inline void latlon_to_cell(double lat, double lon, int *r, int *c) {
    if (lat >  90.0) lat =  90.0;
    if (lat < -90.0) lat = -90.0;
    double L = norm_lon(lon); /* normalise vers [-180,180) */
    *r = (int)floor((lat + 90.0)  / CELL_DEG);
    *c = (int)floor((L   + 180.0) / CELL_DEG);
    int max_r = (int)floor(180.0 / CELL_DEG) - 1;
    int max_c = (int)floor(360.0 / CELL_DEG) - 1;
    if (*r < 0) { *r = 0; }
    if (*r > max_r) { *r = max_r; }
    if (*c < 0) { *c = 0; }
    if (*c > max_c) { *c = max_c; }

}

/**
 * grid_reset()
 * Destroy the existing grid (if any) and create an empty grid with appropriate key/value destructors.
 */
static void grid_reset(void) {
    if (grid) g_hash_table_destroy(grid);
    grid = g_hash_table_new_full((GHashFunc)cell_hash, (GEqualFunc)cell_equal,
                                 g_free, (GDestroyNotify)g_ptr_array_unref);
}

/**
 * grid_insert_bbox(lat_min,lat_max,lon_min,lon_max, poly)
 * Insert a rectangle into all cells it overlaps. If the rectangle spans the dateline (a>b),
 * split into two spans so cell ranges are monotone in longitude. Bucket stores 'poly' pointer.
 */
static void grid_insert_bbox(double lat_min, double lat_max,
                             double lon_min, double lon_max,
                             gpointer poly /* GArray* */)
{
    /* normalise et découpe si le rectangle traverse l'antiméridien */
    double a = norm_lon(lon_min);
    double b = norm_lon(lon_max);

    if (a <= b) {
        int r0,c0,r1,c1;
        latlon_to_cell(lat_min, a, &r0, &c0);
        latlon_to_cell(lat_max, b, &r1, &c1);
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                CellKey *key = g_new(CellKey, 1); key->r = r; key->c = c;
                GPtrArray *bucket = g_hash_table_lookup(grid, key);
                if (!bucket) { bucket = g_ptr_array_new(); g_hash_table_insert(grid, key, bucket); }
                else { g_free(key); }
                g_ptr_array_add(bucket, poly);
            }
        }
    } else {
        /* span 1 : [a, 180) */
        int r0,c0,r1,c1;
        latlon_to_cell(lat_min, a, &r0, &c0);
        latlon_to_cell(lat_max, 180.0 - 1e-9, &r1, &c1);
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                CellKey *key = g_new(CellKey, 1); key->r = r; key->c = c;
                GPtrArray *bucket = g_hash_table_lookup(grid, key);
                if (!bucket) { bucket = g_ptr_array_new(); g_hash_table_insert(grid, key, bucket); }
                else { g_free(key); }
                g_ptr_array_add(bucket, poly);
            }
        }
        /* span 2 : [-180, b] */
        latlon_to_cell(lat_min, -180.0, &r0, &c0);
        latlon_to_cell(lat_max, b, &r1, &c1);
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                CellKey *key = g_new(CellKey, 1); key->r = r; key->c = c;
                GPtrArray *bucket = g_hash_table_lookup(grid, key);
                if (!bucket) { bucket = g_ptr_array_new(); g_hash_table_insert(grid, key, bucket); }
                else { g_free(key); }
                g_ptr_array_add(bucket, poly);
            }
        }
    }
}

/**
 * Free all stored polygons and clear the list.
 */
static void
_free_all_polygons(void)
{
    for (GList *l = all_polygons; l; l = l->next) {
        g_array_free((GArray*)l->data, TRUE);
    }
    g_list_free(all_polygons);

    all_polygons = NULL;
    if (rect_meta) { g_hash_table_destroy(rect_meta); rect_meta = NULL; }
    if (grid) { g_hash_table_destroy(grid); grid = NULL; }

    /* free each strdup’d country string, then the list */
    for (GList *l = all_countries; l; l = l->next)
        g_free(l->data);
    g_list_free(all_countries);
    all_countries = NULL;

}

/**
 * Load the CSV file, ignoring the first field, reading columns:
 *   [3]=longitude center, [4]=latitude center,
 *   [5]=width, [6]=height
 * and appending one 4-corner polygon per row to all_polygons.
 */
static void
_load_csv(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f) {
        g_error("tool_init: cannot open CSV '%s': %s", filename, strerror(errno));
        return;
    }
    char line[1024];
    /* skip header, bail if we didn’t get one */
    if (fgets(line, sizeof(line), f) == NULL) {
        fclose(f);
        return;
    }

    while (fgets(line, sizeof(line), f)) {
        gchar **fld = g_strsplit(line, ",", -1);
        
        if (!fld[3] || !fld[4] || !fld[5] || !fld[6]) {
            g_strfreev(fld);
            continue;
        }
        double lon_c = g_ascii_strtod(fld[3], NULL);
        double lat_c = g_ascii_strtod(fld[4], NULL);
        double w     = g_ascii_strtod(fld[5], NULL);
        double h     = g_ascii_strtod(fld[6], NULL);
        /* build a new GArray of 4 GeoPoint corners */
        GArray *arr = g_array_new(FALSE, FALSE, sizeof(GeoPoint));
        GeoPoint corners[4] = {
            { lat_c - h/2, lon_c - w/2 },  /* SW */
            { lat_c - h/2, lon_c + w/2 },  /* SE */
            { lat_c + h/2, lon_c + w/2 },  /* NE */
            { lat_c + h/2, lon_c - w/2 }   /* NW */
        };
        for (int i = 0; i < 4; ++i)
            g_array_append_val(arr, corners[i]);
        /* O(1) insert; we'll reverse once after the loop */
        all_polygons  = g_list_prepend(all_polygons, arr);

        /* Build & store rectangle metadata (fast path) */
        if (!rect_meta)
            rect_meta = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, g_free);
        TileRect *R = g_new0(TileRect, 1);
        /* bbox in degrees; handle potential wrap later in contains() */
        R->lat_min = fmin(fmin(corners[0].lat, corners[1].lat), fmin(corners[2].lat, corners[3].lat));
        R->lat_max = fmax(fmax(corners[0].lat, corners[1].lat), fmax(corners[2].lat, corners[3].lat));
        R->lon_min = fmin(fmin(corners[0].lon, corners[1].lon), fmin(corners[2].lon, corners[3].lon));
        R->lon_max = fmax(fmax(corners[0].lon, corners[1].lon), fmax(corners[2].lon, corners[3].lon));
        g_hash_table_insert(rect_meta, arr, R);

        /* Index the rectangle bbox into the spatial grid (stores arr ptr) */
        grid_insert_bbox(R->lat_min, R->lat_max, R->lon_min, R->lon_max, arr);
       
        /* strip leading/trailing whitespace (newlines) from the country field */
       if (fld[7])
           fld[7] = g_strstrip(fld[7]);
        /* …and the country name (FIELD 7 of your CSV) */
        all_countries = g_list_prepend(
            all_countries,
            fld[7] && *fld[7]
                ? g_strdup(fld[7])
                : g_strdup("")); 
    }

    /* restore original order after prepend inserts */
    all_polygons  = g_list_reverse(all_polygons);
    all_countries = g_list_reverse(all_countries);
    fclose(f);

}

/**
 * Ray-casting algorithm: test if (lat,lon) lies inside the polygon pts[0..n-1].
 */
gboolean
_point_in_poly(const GeoPoint *pts, guint n, double lat, double lon)
{
    gboolean inside = FALSE;
    for (guint i = 0, j = n - 1; i < n; j = i++) {
        double xi = pts[i].lon, yi = pts[i].lat;
        double xj = pts[j].lon, yj = pts[j].lat;
        if ((yi > lat) != (yj > lat)) {
            double x_int = xi + (lat - yi) * (xj - xi) / (yj - yi);
            if (lon < x_int)
                inside = !inside;
        }
    }
    return inside;
}

/*=============================================================================
 * Public API
 *=============================================================================*/

/**
 * tool_init(csv_path)
 * Reset global state, rebuild spatial grid, and load CSV-derived polygons/rectangles/countries.
 */
void tool_init(const char *csv_path)
{
    if (all_polygons || all_countries)
        _free_all_polygons();
    grid_reset();
    _load_csv(csv_path);
}

/**
 * tool_cleanup()
 * Free all data structures created by tool_init/_load_csv.
 */
void tool_cleanup(void)
{
    _free_all_polygons();
}

/**
 * Get the list of all tile polygons (GArray* of GeoPoint) loaded from CSV.
 */
GList*
tool_get_all_polygons(void)
{
    return all_polygons;
}

/**
 * tool_get_all_countries() → GList*
 * Accessor: returns the list of strdup'd country names aligned with polygons.
 */
GList*
tool_get_all_countries(void)
{
    return all_countries;
}

/**
 * Filter a list of ToolEphemPoint* to only those inside any polygon in 'polygons'.
 */


GList*
ephemeris_filter_by_polygons(GList *pass, GList *polygons)
{
    /* Build a temporary set for quick membership tests if a subset is provided */
    GHashTable *allow = NULL;
    if (polygons) {
        allow = g_hash_table_new(g_direct_hash, g_direct_equal);
        for (GList *p = polygons; p; p = p->next)
            g_hash_table_add(allow, p->data); /* keys are GArray* */
    }

    GList *out = NULL;
    for (GList *l = pass; l; l = l->next) {
        ToolEphemPoint *pt = (ToolEphemPoint*)l->data;
        int cr, cc; latlon_to_cell(pt->lat, pt->lon, &cr, &cc);
        gboolean hit = FALSE;

        /* Check current cell + 8 neighbors */
        for (int dr = -1; dr <= 1 && !hit; ++dr) {
            for (int dc = -1; dc <= 1 && !hit; ++dc) {
                CellKey key = { cr + dr, cc + dc };
                GPtrArray *bucket = grid ? g_hash_table_lookup(grid, &key) : NULL;
                if (!bucket) continue;
                for (guint i = 0; i < bucket->len && !hit; ++i) {
                    GArray *arr = g_ptr_array_index(bucket, i);
                    if (allow && !g_hash_table_contains(allow, arr)) continue;
                    const TileRect *R = rect_meta ? g_hash_table_lookup(rect_meta, arr) : NULL;
                    if (R && rect_contains(R, pt->lat, pt->lon)) hit = TRUE;
                }
            }
        }
        if (hit) out = g_list_prepend(out, pt);
    }

    if (allow) g_hash_table_destroy(allow);
    out = g_list_reverse(out);
    return out;
}

/**
 * Build a GtkListStore from filtered ephemeris points,
 * inserting blank rows when time gaps exceed 30s.
 */
GtkListStore*
build_ephemeris_store(GList *filtered_pass)
{
    enum { COL_TIME, COL_LAT, COL_LON, N_COLS };
    GtkListStore *store = gtk_list_store_new(
        N_COLS,
        G_TYPE_STRING,
        G_TYPE_DOUBLE,
        G_TYPE_DOUBLE);
    GtkTreeIter iter;
    guint last_t = 0;
    for (GList *l = filtered_pass; l; l = l->next) {
        ToolEphemPoint *pt = (ToolEphemPoint*)l->data;
        if (last_t && pt->timestamp - last_t > 30) {
            gtk_list_store_append(store, &iter);
            gtk_list_store_set(store, &iter,
                               COL_TIME, "",
                               COL_LAT,  0.0,
                               COL_LON,  0.0,
                               -1);
        }
        char buf[64];
        struct tm tm;
        time_t t = pt->timestamp;
        gmtime_r(&t, &tm);
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter,
                           COL_TIME, buf,
                           COL_LAT,  pt->lat,
                           COL_LON,  pt->lon,
                           -1);
        last_t = pt->timestamp;
    }
    return store;
}

/**
 * Convert a GtkTreeModel back into a list of ToolEphemPoint*.
 */
GList*
tool_list_from_model(GtkTreeModel *model)
{
    GList *out = NULL;
    GtkTreeIter iter;

    enum { COL_TIME, COL_LAT, COL_LON, N_COLS };

    /* Start at the first row; if none, return NULL */
    if (!gtk_tree_model_get_iter_first(model, &iter))
        return NULL;

    do {
        gchar *timestr;
        double lat, lon;

        /* Pull the original timestamp string and coordinates */
        gtk_tree_model_get(model, &iter,
                           COL_TIME, &timestr,
                           COL_LAT,  &lat,
                           COL_LON,  &lon,
                           -1);

        /* Parse the timestamp string into a time_t */
        struct tm tm = {0};
        strptime(timestr, "%Y-%m-%d %H:%M:%S", &tm);
        time_t t = timegm(&tm);

        /* Allocate and populate the ephemeris point */
        ToolEphemPoint *pt = g_new0(ToolEphemPoint, 1);
        pt->timestamp = (guint)t;
        pt->lat       = lat;
        pt->lon       = lon;
        pt->time_str  = timestr;  /* take ownership of the string */

        out = g_list_prepend(out, pt);

        /* Do NOT g_free(timestr)—we store it in pt->time_str */
    } while (gtk_tree_model_iter_next(model, &iter));

        /* restore chronological order */
    out = g_list_reverse(out);
    return out;

}

