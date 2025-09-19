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



#ifndef LOGIC_POINT_H
#define LOGIC_POINT_H

#include <glib.h>
#include <time.h>

/* A simple lat/lon pair */
typedef struct {
    double  lat;
    double  lon;
} LP_GeoPoint;

/* One ephemeris sample */
typedef struct {
    time_t        time;      /* seconds since UNIX epoch */
    double        lat, lon;  /* degrees */
    char         *time_str;  /* human‐readable timestamp */
} LP_EphemPoint;

/* Initialize: load tile CSV into polygons list */
gboolean lp_init(const char *csv_path, GError **err_out);

/* Free all loaded tile data */
void     lp_cleanup(void);

/* Get loaded polygons (GArray* of LP_GeoPoint) */
GList*   lp_get_all_polygons(void);

/* Filter ephemeris list to only points inside any tile */
GList*   lp_filter_points_by_tiles(GList *all_ephemirs);

/* Compute great‐circle distance (km) and bearing (deg) */
double lp_compute_distance_km(const LP_GeoPoint *a,
                              const LP_GeoPoint *b);
double lp_compute_bearing_deg(const LP_GeoPoint *from,
                              const LP_GeoPoint *to);

/* Expose point-in-polygon publicly */
gboolean lp_point_in_poly(const LP_GeoPoint *pts, guint n_pts,
                          double lat, double lon);

/* Given a 4-corner tile polygon, return its geographic center */
LP_GeoPoint lp_polygon_center(GArray *poly);

#endif /* LOGIC_POINT_H */
