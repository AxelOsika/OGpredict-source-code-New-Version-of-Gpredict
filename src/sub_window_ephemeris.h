/*
  OGpredict — extensions to Gpredict for operations planning

  Copyright (C) 2025 Axel Osika <axel.osika@example.com>

  This file is part of OGpredict, a derivative of Gpredict.

  OGpredict is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.  See the GNU General Public License for details.
*/

/* SPDX-License-Identifier: GPL-2.0-or-later */




/* src/sub_window_ephemeris.h */
#pragma once
#include <gtk/gtk.h>

typedef enum {
     SUBWIN_FORMAT_CSV,
     SUBWIN_FORMAT_TXT
} SubwinFormat;

typedef struct {
     gchar         *filepath;  /* absolute path the user picked */
     SubwinFormat   format;    /* CSV or TXT */
} SubwinSaveSpec;

/* Column mapping for the POI GtkTreeModel */
typedef struct {
     gint col_time;
     gint col_lat;
     gint col_lon;
     gint col_range;
     gint col_dir;
     gint col_name;
     gint col_type;
} POIColumns;

/* Opens a native Save dialog. Returns TRUE if user confirmed, FALSE otherwise. */
gboolean sub_window_ephemeris_run(GtkWindow *parent, SubwinSaveSpec *out_spec);

 /* Free strings held in the spec */
void sub_window_ephemeris_spec_free(SubwinSaveSpec *spec);


/* Write the POI table to disk according to spec (CSV/TXT). Returns TRUE on success. */
gboolean sub_window_ephemeris_export_poi(GtkTreeView *tv,
                                          const SubwinSaveSpec *spec,
                                          const POIColumns *cols,
                                          GError **err);
