#ifndef POINTS_INTERESTS_H
#define POINTS_INTERESTS_H

#include <glib.h>
#include <gtk/gtk.h>

/** Path to the CSV shipped with your app; adjust as needed */
#define POI_CSV_FILE "src/Points_of_Interests.csv"

/**
 * Read the CSV (once) and cache all POI names.
 *
 * @param csv_file  full path to Points_of_Interests.csv
 */
void points_interest_init(const char *csv_file);

/**
 * Return a GPtrArray* of (g_strdup’d) POI names.
 * If init() hasn’t been called yet, this will call
 * points_interest_init(POI_CSV_FILE).
 */
GPtrArray* points_interest_get_names(void);
GPtrArray *points_interest_get_types(void);
/**
 * Free all cached POI names and reset internal cache.
 * Call at shutdown if you care about leak-free.
 */
void points_interest_shutdown(void);

/**
 * Compute lat/lon bounds for a square tile of size tile_km (km) centered
 * at (center_lat, center_lon). Returns FALSE on invalid input.
 */
gboolean points_interest_compute_bounds(double center_lat,
                                        double center_lon,
                                        double tile_km,
                                        double *lat_min,
                                        double *lat_max,
                                        double *lon_min,
                                        double *lon_max);

/**
 * Append a new POI to the CSV (creates file/header if missing) and update
 * the in-memory name/type caches so completions see it immediately.
 * If csv_file is NULL, POI_CSV_FILE is used.
 */
gboolean points_interest_add_to_csv(const char *csv_file,
                                    const char *name,
                                    const char *type,
                                    double      tile_km,
                                    double      center_lat,
                                    double      center_lon,
                                    GError    **error);

#endif /* POINTS_INTEREST_H */
