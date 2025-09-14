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



/**
 * Logic_POI_Filter.c — POI tile loading and high-speed ephemeris filtering
 * Purpose:
 * - Load POI tiles as axis-aligned rectangles (preferred: Lat_min/Lat_max/Lon_min/Lon_max; fallback: center+Tile_km).
 * - Build a 1°×1° spatial grid index (CellKey→bucket of polygons) with dateline-aware insertion.
 * - Filter ephemeris points using constant-time rectangle tests over small local buckets.
 * 
 * Critical globals:
 * - all_polygons : GList* of GArray* (4-corner rectangles kept for compatibility/UI).
 * - poi_rects    : GHashTable (key=GArray*, val=TileRect*) rectangle metadata for fast inclusion tests.
 * - grid         : GHashTable spatial index of buckets for quick candidate retrieval.
 * - CELL_DEG     : grid cell size in degrees (1.0 by default).
 * - EPS          : small tolerance for inclusive bounds to prevent borderline misses.
 * 
 * Performance model:
 * - Build: O(M·α) (α = average cells per rectangle).
 * - Query: O(N·B) with B = local bucket size (typically ≪ M).
 * Both CSV ingest and output lists use prepend+reverse for O(M) / O(|S|) behavior.
 */



#include "Logic_POI_Filter.h"
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "Logic_Country_Filter.h"    /* for _point_in_poly prototype */

/* Internal globals */
static GList *all_polygons = NULL;


/**
 * TileRect
 * Axis-aligned rectangle bounds in degrees for a POI tile.
 */
typedef struct { double lat_min, lat_max, lon_min, lon_max; } TileRect;

/**
 * poi_rects (polygon → rectangle metadata)
 * Key: GArray* polygon (4 corners).
 * Val: TileRect* bounds (owned here) for constant-time rect_contains.
 */
static GHashTable *poi_rects = NULL;   /* key: GArray* ; val: TileRect* */

#define EPS 1e-12

/**
 * norm_lon(lon)
 * Normalize longitude into [-180,180). Required by grid mapping and wrap-safe interval tests.
 */
static inline double norm_lon(double lon){ double x=fmod(lon+180.0,360.0); if(x<0)x+=360.0; return x-180.0; }

/**
 * rect_contains(R, lat, lon)
 * Constant-time rectangle membership:
 * - Latitude interval test, then longitude interval test.
 * - If [lon_min, lon_max] wraps (a>b), test union [a,180) ∪ [-180,b].
 */
static inline gboolean rect_contains(const TileRect*R,double lat,double lon){
    if (lat < R->lat_min - EPS || lat > R->lat_max + EPS) return FALSE;
    double a=norm_lon(R->lon_min), b=norm_lon(R->lon_max), L=norm_lon(lon);
    if (a <= b) return (L >= a - EPS && L <= b + EPS);
    return (L >= a - EPS || L <= b + EPS);
}

/* Spatial grid */
/**
 * CellKey {r,c}
 * Grid coordinates (row by latitude, column by longitude).
 */
typedef struct { int r, c; } CellKey;
/**
 * grid (spatial index)
 * HashMap from CellKey* to GPtrArray* (bucket of polygons).
 */
static GHashTable *grid = NULL;   /* key: CellKey*, val: GPtrArray* of GArray* polygons */
static const double CELL_DEG = 1.0;
/**
 * cell_hash(key)
 * Hash for CellKey using a multiplicative mix.
 */
static guint cell_hash(gconstpointer key) {
    const CellKey *k = (const CellKey*)key;
    return 2654435761u * (guint)(k->r ^ (k->c << 16));
}
/**
 * cell_equal(a,b)
 * Row/column equality for CellKey.
 */
static gboolean cell_equal(gconstpointer a, gconstpointer b) {
    const CellKey *x = (const CellKey*)a, *y = (const CellKey*)b;
    return x->r == y->r && x->c == y->c;
}
/**
 * latlon_to_cell(lat, lon, &r, &c)
 * Clamp latitude, normalize longitude, compute row/col via floor; clamp indices to bounds.
 */
static inline void latlon_to_cell(double lat, double lon, int *r, int *c) {

    /* clamp + normalisation pour robustesse autour de ±180° */
    if (lat >  90.0) lat =  90.0;
    if (lat < -90.0) lat = -90.0;
    double L = norm_lon(lon);
    *r = (int)floor((lat + 90.0)  / CELL_DEG);
    *c = (int)floor((L   + 180.0) / CELL_DEG);
    int max_r = (int)floor(180.0 / CELL_DEG) - 1;
    int max_c = (int)floor(360.0 / CELL_DEG) - 1;
    /* Clamp indices with explicit blocks to silence -Wmisleading-indentation */
    if (*r < 0) {
        *r = 0;
    } else if (*r > max_r) {
        *r = max_r;
    }
    if (*c < 0) {
        *c = 0;
    } else if (*c > max_c) {
        *c = max_c;
    }

}
/**
 * grid_reset()
 * Destroy and recreate the spatial grid with proper destructors.
 */
static void grid_reset(void) {
    if (grid) g_hash_table_destroy(grid);
    grid = g_hash_table_new_full((GHashFunc)cell_hash, (GEqualFunc)cell_equal,
                                 g_free, (GDestroyNotify)g_ptr_array_unref);
}

/**
 * grid_insert_bbox(lat_min,lat_max,lon_min,lon_max, poly)
 * Index rectangle into all overlapped cells. If dateline-wrapped, split into two spans so insertion is monotone in lon.
 */
static void grid_insert_bbox(double lat_min, double lat_max,
                             double lon_min, double lon_max,
                             gpointer poly /* GArray* */)
{
    /* normalise et découpe si nécessaire (antiméridien) */
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
        int r0,c0,r1,c1;
        /* [a, 180) */
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
        /* [-180, b] */
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

/*----------------------------------------------------------------------*/
/* lp_init: read CSV, build each tile polygon                           */
/*----------------------------------------------------------------------*/
/**
 * lp_init(csv_path, err_out) → gboolean
 * Load POI tiles from CSV:
 * - Preferred: Lat_min/Lat_max/Lon_min/Lon_max columns → exact rectangles.
 * - Fallback: Center_Lat/Center_Lon + Tile_km (≤10 km squares) → convert to degrees at that latitude.
 * - For each tile: build 4-corner polygon (compat), store TileRect in poi_rects, index into grid.
 * - Prepend rows (O(1)); reverse once after reading to preserve CSV order.
 */
gboolean lp_init(const char *csv_path, GError **err_out)
{
    grid_reset();

    FILE *f = fopen(csv_path, "r");
    if (!f) {
        g_set_error(err_out, G_FILE_ERROR, g_file_error_from_errno(errno),
                    "Cannot open CSV '%s': %s", csv_path, strerror(errno));
        return FALSE;
    }

    char line[2048];
    /* read header */
    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        return FALSE;
    }

    /* map header columns */
    gchar **hdr = g_strsplit(line, ",", -1);
    int idx_lat_min = -1, idx_lat_max = -1, idx_lon_min = -1, idx_lon_max = -1;
    int idx_center_lat = -1, idx_center_lon = -1, idx_tile_km = -1; /* optional fallback */
    for (int i = 0; hdr[i]; ++i) {
        gchar *h = g_strstrip(hdr[i]);
        if (!g_ascii_strcasecmp(h, "Lat_min"))      idx_lat_min   = i;
        else if (!g_ascii_strcasecmp(h, "Lat_max")) idx_lat_max   = i;
        else if (!g_ascii_strcasecmp(h, "Lon_min")) idx_lon_min   = i;
        else if (!g_ascii_strcasecmp(h, "Lon_max")) idx_lon_max   = i;
        else if (!g_ascii_strcasecmp(h, "Center_Lat")) idx_center_lat = i;
        else if (!g_ascii_strcasecmp(h, "Center_Lon")) idx_center_lon = i;
        else if (!g_ascii_strcasecmp(h, "Tile_km"))    idx_tile_km    = i;
    }
    g_strfreev(hdr);


    while (fgets(line, sizeof(line), f)) {
        gchar **fld = g_strsplit(line, ",", -1);
        
        /* Preferred path: read explicit bounds from CSV (Lat_min/Lat_max/Lon_min/Lon_max) */
        if (idx_lat_min >= 0 && idx_lat_max >= 0 && idx_lon_min >= 0 && idx_lon_max >= 0) {
            double lat_min = g_ascii_strtod(fld[idx_lat_min], NULL);
            double lat_max = g_ascii_strtod(fld[idx_lat_max], NULL);
            double lon_min = g_ascii_strtod(fld[idx_lon_min], NULL);
            double lon_max = g_ascii_strtod(fld[idx_lon_max], NULL);

            /* 4-corner polygon (SW, SE, NE, NW) for compatibility */
            GArray *poly = g_array_new(FALSE, FALSE, sizeof(LP_GeoPoint));
            LP_GeoPoint corners[4] = {
                { lat_min, lon_min },
                { lat_min, lon_max },
                { lat_max, lon_max },
                { lat_max, lon_min }
            };
            for (int i = 0; i < 4; ++i) g_array_append_val(poly, corners[i]);
            all_polygons = g_list_prepend(all_polygons, poly);

            /* rectangle meta (fast path) */
            if (!poi_rects) poi_rects = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, g_free);
            TileRect *R = g_new0(TileRect, 1);
            R->lat_min = lat_min; R->lat_max = lat_max; R->lon_min = lon_min; R->lon_max = lon_max;
            g_hash_table_insert(poi_rects, poly, R);

            /* index into spatial grid */
            grid_insert_bbox(lat_min, lat_max, lon_min, lon_max, poly);
        }
        
        /* Fallback path: legacy columns Center_Lat/Center_Lon + Tile_km (square <=10 km) */
        else if (idx_center_lat >= 0 && idx_center_lon >= 0 && idx_tile_km >= 0) {
            double lat_c = g_ascii_strtod(fld[idx_center_lat], NULL);
            double lon_c = g_ascii_strtod(fld[idx_center_lon], NULL);
            double half_km = g_ascii_strtod(fld[idx_tile_km], NULL) * 0.5;
            /* 1° lat ≈ 110.574 km; 1° lon ≈ 111.320 km × cos(lat) */
            double lat_deg = half_km / 110.574;
            double lon_deg = half_km / (111.320 * cos(lat_c * G_PI/180.0));
            double lat_min = lat_c - lat_deg, lat_max = lat_c + lat_deg;
            double lon_min = lon_c - lon_deg, lon_max = lon_c + lon_deg;

            GArray *poly = g_array_new(FALSE, FALSE, sizeof(LP_GeoPoint));
            LP_GeoPoint corners[4] = {
                { lat_min, lon_min },
                { lat_min, lon_max },
                { lat_max, lon_max },
                { lat_max, lon_min }
            };
            for (int i = 0; i < 4; ++i) g_array_append_val(poly, corners[i]);
            all_polygons = g_list_prepend(all_polygons, poly);

            if (!poi_rects) poi_rects = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, g_free);
            TileRect *R = g_new0(TileRect, 1);
            R->lat_min = lat_min; R->lat_max = lat_max; R->lon_min = lon_min; R->lon_max = lon_max;
            g_hash_table_insert(poi_rects, poly, R);


            grid_insert_bbox(lat_min, lat_max, lon_min, lon_max, poly);

        }
        /* Unknown CSV layout: skip row */
        g_strfreev(fld);
    }

    /* restore original order after prepend insertions */
    all_polygons = g_list_reverse(all_polygons);
    fclose(f);
    return TRUE;

}

/*----------------------------------------------------------------------*/
/* lp_cleanup: free polygons                                            */
/*----------------------------------------------------------------------*/
/**
 * lp_cleanup()
 * Free polygons, poi_rects, and the spatial grid.
 */
void lp_cleanup(void)
{
    for (GList *n = all_polygons; n; n = n->next) {
        g_array_free((GArray*)n->data, TRUE);
    }

    g_list_free(all_polygons);
    all_polygons = NULL;
    if (poi_rects) { g_hash_table_destroy(poi_rects); poi_rects = NULL; }
    if (grid)      { g_hash_table_destroy(grid);      grid = NULL; }

}

/*----------------------------------------------------------------------*/
/* lp_get_all_polygons: accessor                                        */
/*----------------------------------------------------------------------*/
/**
 * lp_get_all_polygons() → GList*
 * Accessor: returns the list of polygon GArray* (read-only ownership).
 */
GList* lp_get_all_polygons(void)
{
    return all_polygons;
}

/* Compute the centroid of a rectangular tile (average of corners 0 & 2) */
LP_GeoPoint
lp_polygon_center(GArray *poly)
{
    /* Prefer rectangle metadata if available (robust) */
    if (poi_rects) {
        const TileRect *R = g_hash_table_lookup(poi_rects, poly);
        if (R) {
            double lat_c = 0.5 * (R->lat_min + R->lat_max);
            double lon_c = 0.5 * (R->lon_min + R->lon_max);
            return (LP_GeoPoint){ lat_c, lon_c };
        }
    }
    /* Fallback: compute from stored corners */
    LP_GeoPoint *pts = (LP_GeoPoint*)poly->data;
    double lat_c = (pts[0].lat + pts[2].lat) * 0.5;
    double lon_c = (pts[0].lon + pts[2].lon) * 0.5;
    return (LP_GeoPoint){ lat_c, lon_c };

}


/*----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*/
/* lp_filter_points_by_tiles: spatial grid + rectangle fast path */
/*----------------------------------------------------------------------*/

/**
 * lp_filter_points_by_tiles(all_ephemirs) → GList*
 * Filter on the hot path: for each ephemeris point map to cell, probe 3×3 buckets,
 * use TileRect from poi_rects for constant-time rect_contains; early exit;
 * prepend match and reverse once at end.
 */
GList* lp_filter_points_by_tiles(GList *all_ephemirs)
{
    GList *out = NULL;
    for (GList *e = all_ephemirs; e; e = e->next) {
        LP_EphemPoint *pt = e->data;
        int cr, cc; latlon_to_cell(pt->lat, pt->lon, &cr, &cc);
        gboolean hit = FALSE;
        for (int dr = -1; dr <= 1 && !hit; ++dr) {
            for (int dc = -1; dc <= 1 && !hit; ++dc) {
                CellKey key = { cr + dr, cc + dc };
                GPtrArray *bucket = grid ? g_hash_table_lookup(grid, &key) : NULL;
                if (!bucket) continue;
                for (guint i=0;i<bucket->len && !hit;i++) {
                    GArray *arr = g_ptr_array_index(bucket, i);
                    const TileRect *R = poi_rects ? g_hash_table_lookup(poi_rects, arr) : NULL;
                    if (R && rect_contains(R, pt->lat, pt->lon)) {
                        hit = TRUE; /* found in rectangle */
                    }
                }
            }
        }
        if (hit) out = g_list_prepend(out, pt);
    }

    out = g_list_reverse(out);
    return out;

}

/*----------------------------------------------------------------------*/
/* lp_point_in_poly: public wrapper for internal point‐in‐poly test     */
/*----------------------------------------------------------------------*/

/**
 * lp_point_in_poly(pts, n_pts, lat, lon) → gboolean
 * Wrapper for the generic polygon test; useful when rectangles are not applicable.
 */
gboolean lp_point_in_poly(const LP_GeoPoint *pts, guint n_pts,
                          double lat, double lon)
{
    return _point_in_poly((const GeoPoint*)pts, n_pts, lat, lon);
}


/*----------------------------------------------------------------------*/
/* lp_compute_distance_km: haversine formula                              */
/*----------------------------------------------------------------------*/

/**
 * lp_compute_distance_km(a, b) → km
 * Great-circle distance via haversine; helper for diagnostics/UI.
 */
double lp_compute_distance_km(const LP_GeoPoint *a,
                              const LP_GeoPoint *b)
{
    const double R = 6371.0; /* Earth radius in km */
    double dlat = (b->lat - a->lat) * G_PI/180.0;
    double dlon = (b->lon - a->lon) * G_PI/180.0;
    double lat1 = a->lat * G_PI/180.0;
    double lat2 = b->lat * G_PI/180.0;
    double h = sin(dlat/2)*sin(dlat/2) +
               sin(dlon/2)*sin(dlon/2) * cos(lat1)*cos(lat2);
    return 2 * R * atan2(sqrt(h), sqrt(1-h));
}

/*----------------------------------------------------------------------*/
/* lp_compute_bearing_deg: forward azimuth                                */
/*----------------------------------------------------------------------*/
/**
 * lp_compute_bearing_deg(from, to) → degrees
 * Forward azimuth from 'from' to 'to' in degrees [0,360).
 */
double lp_compute_bearing_deg(const LP_GeoPoint *from,
                              const LP_GeoPoint *to)
{
    double φ1 = from->lat * G_PI/180.0;
    double φ2 = to->lat   * G_PI/180.0;
    double Δλ = (to->lon - from->lon) * G_PI/180.0;
    double y = sin(Δλ) * cos(φ2);
    double x = cos(φ1)*sin(φ2) -
               sin(φ1)*cos(φ2)*cos(Δλ);
    double θ = atan2(y, x) * 180.0/G_PI;
    return fmod(θ + 360.0, 360.0);
}







