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



#include "points_interests.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Static cache of names (each element is a g_strdup’d char*) */
static GPtrArray *poi_names = NULL;
static GPtrArray *poi_types = NULL;      /* parallel array for the “Type” column */

void points_interest_init(const char *csv_file)
{
    if (poi_names) return;
    poi_names = g_ptr_array_new_with_free_func(g_free);
    poi_types = g_ptr_array_new_with_free_func(g_free);   /* allocate types too */

    FILE *fp = fopen(csv_file, "r");
    if (!fp) {
        g_warning("Could not open POI CSV '%s'", csv_file);
        return;
    }

    char line[1024];
    /* skip header */
    if (!fgets(line, sizeof(line), fp)) {
        fclose(fp);
        return;
    }

    while (fgets(line, sizeof(line), fp)) {
        /* strip newline/cr */
        char *p = strchr(line, '\r');
        if (p) *p = '\0';
        p = strchr(line, '\n');
        if (p) *p = '\0';

        /* tokenize on comma: first field = name, second field = type */
        char *name = strtok(line, ",");
        char *type = strtok(NULL, ",");
        if (name && *name) {
            g_ptr_array_add(poi_names, g_strdup(name));
            /* if there was no type column, fall back to empty string */
            g_ptr_array_add(poi_types, g_strdup(type ? type : ""));
        }
    }
    fclose(fp);
}

GPtrArray* points_interest_get_names(void)
{
    if (!poi_names) {
        points_interest_init(POI_CSV_FILE);
    }
    return poi_names;
}

GPtrArray *
points_interest_get_types(void)
{
    return poi_types;
}

void points_interest_shutdown(void)
{
    if (poi_names) {
        g_ptr_array_free(poi_names, TRUE);
        poi_names = NULL;
    }
    if (poi_types) {
        g_ptr_array_free(poi_types, TRUE);
        poi_types = NULL;
    }
}



/* ─────────────────────────────── Helpers ─────────────────────────────── */
static inline double clamp(double v, double lo, double hi){
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline double norm_lon(double lon){
    /* normalize into [-180, 180] */
    while (lon < -180.0) lon += 360.0;
    while (lon >  180.0) lon -= 360.0;
    return lon;
}

/* Core: degrees per km varies with latitude for longitude; latitude is ~constant */
#define LAT_KM_PER_DEG 111.32

gboolean
points_interest_compute_bounds(double center_lat,
                               double center_lon,
                               double tile_km,
                               double *lat_min,
                               double *lat_max,
                               double *lon_min,
                               double *lon_max)
{
    if (!isfinite(center_lat) || !isfinite(center_lon) ||
        !isfinite(tile_km)    || tile_km <= 0.0)
        return FALSE;

    /* avoid singularity at the poles for longitudinal scale */
    const double lat_rad = center_lat * (G_PI/180.0);
    double coslat = cos(lat_rad);
    if (fabs(coslat) < 1e-6) coslat = (coslat < 0 ? -1e-6 : 1e-6);

    const double half = tile_km * 0.5;
    const double dlat = half / LAT_KM_PER_DEG;
    const double dlon = half / (LAT_KM_PER_DEG * coslat);

    double la_min = clamp(center_lat - dlat, -90.0, 90.0);
    double la_max = clamp(center_lat + dlat, -90.0, 90.0);
    double lo_min = norm_lon(center_lon - dlon);
    double lo_max = norm_lon(center_lon + dlon);

    /* If min > max after normalization (date line), swap to keep CSV simple */
    if (lo_min > lo_max){
        double t = lo_min; lo_min = lo_max; lo_max = t;
    }

    if (lat_min) *lat_min = la_min;
    if (lat_max) *lat_max = la_max;
    if (lon_min) *lon_min = lo_min;
    if (lon_max) *lon_max = lo_max;
    return TRUE;
}

static gboolean ensure_header_if_new(FILE *fp){
    /* If file is empty, write header */
    long pos = ftell(fp);
    if (pos < 0) return TRUE; /* can't tell, best effort */
    fseek(fp, 0, SEEK_END);
    long end = ftell(fp);
    fseek(fp, pos, SEEK_SET);
    if (end == 0){
        fputs("Name,Type,Tile_km,Center_Lat,Center_Lon,Lat_min,Lat_max,Lon_min,Lon_max\n", fp);
    }
    return TRUE;
}

gboolean
points_interest_add_to_csv(const char *csv_file,
                           const char *name,
                           const char *type,
                           double      tile_km,
                           double      center_lat,
                           double      center_lon,
                           GError    **error)
{
    if (!name || !*name){
        g_set_error(error, g_quark_from_static_string("points_interest"),
                    1, "Name is required");
        return FALSE;
    }
    if (!type) type = "";

    double la_min, la_max, lo_min, lo_max;
    if (!points_interest_compute_bounds(center_lat, center_lon, tile_km,
                                        &la_min, &la_max, &lo_min, &lo_max)){
        g_set_error(error, g_quark_from_static_string("points_interest"),
                    2, "Invalid inputs for bounds computation");
        return FALSE;
    }

    const char *path = csv_file ? csv_file : POI_CSV_FILE;
    FILE *fp = fopen(path, "a+");
    if (!fp){
        g_set_error(error, g_quark_from_static_string("points_interest"),
                    3, "Could not open '%s' for append", path);
        return FALSE;
    }
    /* write header if file is empty */
    ensure_header_if_new(fp);

    /* Always use '.' decimal; fprintf does. */
    int rc = fprintf(fp, "%s,%s,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",
                     name, type,
                     tile_km, center_lat, center_lon,
                     la_min, la_max, lo_min, lo_max);
    fflush(fp);
    fclose(fp);
    if (rc < 0){
        g_set_error(error, g_quark_from_static_string("points_interest"),
                    4, "Write failed for '%s'", path);
        return FALSE;
    }

    /* Make sure caches exist, then append so completion sees the new entry */
    if (!poi_names) points_interest_init(path);
    if (poi_names) g_ptr_array_add(poi_names, g_strdup(name));
    if (poi_types) g_ptr_array_add(poi_types, g_strdup(type));
    return TRUE;
}