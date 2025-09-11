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

/*----------------------------------------------------------------------*/
/* lp_init: read CSV, build each tile polygon                           */
/*----------------------------------------------------------------------*/
gboolean lp_init(const char *csv_path, GError **err_out)
{
    FILE *f = fopen(csv_path, "r");
    if (!f) {
        g_set_error(err_out, G_FILE_ERROR, g_file_error_from_errno(errno),
                    "Cannot open CSV '%s': %s", csv_path, strerror(errno));
        return FALSE;
    }

    char line[1024];
    /* skip header */
    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        return FALSE;
    }

    while (fgets(line, sizeof(line), f)) {
        gchar **fld = g_strsplit(line, ",", -1);
        
        /* expect: [3]=lat, [4]=lon, [5]=width, [6]=height */
        if (fld[3] && fld[4] && fld[5] && fld[6]) {
            /* center of tile */
            double lat_c = g_ascii_strtod(fld[3], NULL);
            double lon_c = g_ascii_strtod(fld[4], NULL);
            /* half-width/height in kilometers */
            double half_w_km = g_ascii_strtod(fld[5], NULL) * 0.5;
            double half_h_km = g_ascii_strtod(fld[6], NULL) * 0.5;
            /* convert km → degrees:
            1° lat ≈ 110.574 km; 1° lon ≈ 111.320 km × cos(lat) */
            double lat_deg_per_km = 1.0/110.574;
            double lon_deg_per_km = 1.0/(111.320 * cos(lat_c * G_PI/180.0));
            double h = half_h_km * lat_deg_per_km;
            double w = half_w_km * lon_deg_per_km;

            /* build 4-corner polygon in *degrees* */
            GArray *poly = g_array_new(FALSE, FALSE, sizeof(LP_GeoPoint));
            LP_GeoPoint corners[4] = {
                { lat_c - h, lon_c - w },
                { lat_c - h, lon_c + w },
                { lat_c + h, lon_c + w },
                { lat_c + h, lon_c - w }
            };
            for (int i = 0; i < 4; ++i)
                g_array_append_val(poly, corners[i]);

            all_polygons = g_list_append(all_polygons, poly);
        }
        g_strfreev(fld);
    }

    fclose(f);
    return TRUE;
}

/*----------------------------------------------------------------------*/
/* lp_cleanup: free polygons                                            */
/*----------------------------------------------------------------------*/
void lp_cleanup(void)
{
    for (GList *n = all_polygons; n; n = n->next) {
        g_array_free((GArray*)n->data, TRUE);
    }
    g_list_free(all_polygons);
    all_polygons = NULL;
}

/*----------------------------------------------------------------------*/
/* lp_get_all_polygons: accessor                                        */
/*----------------------------------------------------------------------*/
GList* lp_get_all_polygons(void)
{
    return all_polygons;
}

/* Compute the centroid of a rectangular tile (average of corners 0 & 2) */
LP_GeoPoint
lp_polygon_center(GArray *poly)
{
    LP_GeoPoint *pts = (LP_GeoPoint*)poly->data;
    /* corners[0] = bottom-left, corners[2] = top-right */
    double lat_c = (pts[0].lat + pts[2].lat) * 0.5;
    double lon_c = (pts[0].lon + pts[2].lon) * 0.5;
    return (LP_GeoPoint){ lat_c, lon_c };
}


/*----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*/
/* lp_filter_points_by_tiles: ray‐cast test per point/polygon           */
/*----------------------------------------------------------------------*/
GList* lp_filter_points_by_tiles(GList *all_ephemirs)
{
    GList *out = NULL;
    for (GList *e = all_ephemirs; e; e = e->next) {
        LP_EphemPoint *pt = e->data;
        for (GList *p = all_polygons; p; p = p->next) {
            GArray *arr = (GArray*)p->data;
            LP_GeoPoint *pts = (LP_GeoPoint*)arr->data;
            if (lp_point_in_poly(pts, arr->len, pt->lat, pt->lon)) {
               out = g_list_append(out, pt);
                break;
            }
        }
    }
    return out;
}

/*----------------------------------------------------------------------*/
/* lp_point_in_poly: public wrapper for internal point‐in‐poly test     */
/*----------------------------------------------------------------------*/

gboolean lp_point_in_poly(const LP_GeoPoint *pts, guint n_pts,
                          double lat, double lon)
{
    return _point_in_poly((const GeoPoint*)pts, n_pts, lat, lon);
}


/*----------------------------------------------------------------------*/
/* lp_compute_distance_km: haversine formula                              */
/*----------------------------------------------------------------------*/

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






