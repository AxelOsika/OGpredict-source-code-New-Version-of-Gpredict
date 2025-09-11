/*
 * tool.c
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
#include <string.h>
#include <errno.h>
#include <time.h>
#include "Logic_Country_Filter.h"



/* Global list of all tile polygons loaded from the CSV. */
static GList *all_polygons  = NULL;
static GList *all_countries = NULL;

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
        /* append the polygon… */
        all_polygons  = g_list_append(all_polygons, arr);
       /* strip leading/trailing whitespace (newlines) from the country field */
       if (fld[7])
           fld[7] = g_strstrip(fld[7]);
        /* …and the country name (FIELD 7 of your CSV) */
        all_countries = g_list_append(
            all_countries,
            fld[7] && *fld[7]
                ? g_strdup(fld[7])
                : g_strdup("")); 
    }
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

void tool_init(const char *csv_path)
{
    if (all_polygons || all_countries)
        _free_all_polygons();
    _load_csv(csv_path);
}

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
    GList *out = NULL;
    for (GList *l = pass; l; l = l->next) {
        ToolEphemPoint *pt = (ToolEphemPoint*)l->data;
        for (GList *pl = polygons; pl; pl = pl->next) {
            GArray *arr = (GArray*)pl->data;
            /* ray‐cast horizontal at y=pt->lat */
            gboolean inside = FALSE;
            GeoPoint *pts = (GeoPoint*)arr->data;
            guint n = arr->len;
            for (guint i = 0, j = n-1; i < n; j = i++) {
                double xi = pts[i].lon, yi = pts[i].lat;
                double xj = pts[j].lon, yj = pts[j].lat;
                if ((yi > pt->lat) != (yj > pt->lat)) {
                    double x_int = xi + (pt->lat - yi) * (xj - xi) / (yj - yi);
                    if (pt->lon < x_int)
                        inside = !inside;
                }
            }
            if (inside) {
                out = g_list_append(out, pt);
                break;
            }
        }
    }
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

        out = g_list_append(out, pt);

        /* Do NOT g_free(timestr)—we store it in pt->time_str */
    } while (gtk_tree_model_iter_next(model, &iter));

    return out;
}

