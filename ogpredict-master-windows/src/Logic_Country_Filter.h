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




// Logic_Country_Filter.h
#ifndef TOOL_H
#define TOOL_H

#include <glib.h>
#include <gtk/gtk.h>

/**
 * A lightweight ephemeris point used internally by the territory filter.
 * This is distinct from the EphemPoint struct in ephem_point.h.
 */
typedef struct {
    guint timestamp;  // seconds since UNIX epoch (UTC)
    double lat;       // degrees
    double lon;       // degrees
    gchar  *time_str;   /* exact string from Tab 1, e.g. "2025-06-18 14:23:10" */
} ToolEphemPoint;

 /* One corner of a tile (lat,lon in degrees).*/

typedef struct {
    double lat;
    double lon;
} GeoPoint;


/**
 * Ray‐casting algorithm: test if (lat,lon) lies inside the polygon pts[0..n-1].
 * X‐axis = longitude, Y‐axis = latitude.
 */
gboolean _point_in_poly(const GeoPoint *pts,
                        guint           n,
                        double          lat,
                        double          lon);

/**
 * Load tile polygons from a CSV file.
 * CSV must have columns:
 *   territory_id, ..., Longitude centre, Latitude centre, Largeur (°), Hauteur (°), ...
 * @param csv_path  Path to CSV.
 */
void tool_init(const char *csv_path);

/** Free all loaded territory data. */
void tool_cleanup(void);

/**
 * Return a GList of gchar* country names, in **exact** one‐to‐one order
 * with the polygons list. Caller must not free these strings.
 */
GList* tool_get_all_countries(void);

/**
 * Return a GList of GArray* of GeoPoint corners loaded from the CSV.
 * Caller must not free the GArray* or its data.
 */
GList* tool_get_all_polygons(void);

/**
 * Given a full pass (list of ToolEphemPoint*), return only those inside any polygon.
 */
GList* ephemeris_filter_by_polygons(GList *pass,
                                   GList *polygons);

/**
 * Build a GtkListStore from filtered pass:
 *   - COL_TIME (string),
 *   - COL_LAT  (double),
 *   - COL_LON  (double),
 * inserting blank rows whenever >30 s gaps occur.
 */
GtkListStore* build_ephemeris_store(GList *filtered_pass);

/**
 * Convert an existing GtkTreeModel (with columns Time, Lat, Lon) into
 * a GList of ToolEphemPoint*. Caller must free each point and the list.
 */
GList* tool_list_from_model(GtkTreeModel *model);

#endif // TOOL_H
