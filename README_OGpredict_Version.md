# OGpredict

**OGpredict** (Optimize General Prediction) is a production‑oriented extension of **Gpredict** for second‑accurate pass planning and target selection. It adds a new **Ephemeris Data** window (with three tabs: *Ephemeris*, *Territory*, *Points of Interest*) to compute 1 Hz ephemerides, filter by countries/territories, and select the best trigger time for named targets — with deterministic, spreadsheet‑ready exports.

OGpredict keeps Gpredict’s strengths (SGP4 on fresh TLEs, AOS/LOS, multi‑platform GTK/GLib stack) and augments them with non‑blocking bulk computations, lightweight geodata pipelines, and Windows packaging (MSYS2/MinGW‑w64 build, `.exe`, bootstrap `.bat`, NSIS installer).

---

## Key features (what’s new vs. Gpredict)

- **Ephemeris Data window** with three complementary tabs:  
  - **Ephemeris** – Generate **1 Hz** ground‑track ephemerides over user‑defined horizons (e.g., multi‑day at 1‑second step) with a responsive UI (worker + chunked insertion).  
  - **Territory** – Filter the ephemeris stream by **ISO‑3166** country/territory (including overseas territories) using a lightweight CSV dataset (`territories.csv`).  
  - **Points of Interest (POI)** – For each named target, pick the **time of minimum ground‑range** (and bearing) within a configurable “tile” size; export results as UTF‑8+BOM CSV for direct spreadsheet use.
- **Deterministic, high‑granularity outputs** (1 Hz; UTC timestamps with metadata headers: TLE source, time step, horizon).
- **Responsive GTK UI** under heavy loads via **asynchronous workers** and **chunked model updates**; proper cancellation and lifecycle handling.
- **Multi‑platform delivery**: reproducible Linux/WSL builds and **Windows native packaging**  
  (**MSYS2/MinGW‑w64** → `OGpredict.exe` + **bootstrap `.bat`** + **NSIS installer**).
- **Lightweight geo‑data pipeline**: ESA WorldCover → normalized CSVs (`territories.csv`, `poi.csv`) suitable for fast selection (tile‑based approach, ISO‑3166 normalization, DROM‑COM handling).

---

## Why OGpredict?

Operational need: **plan image triggers to the second** from a single ground station under SSO constraints, then export immediately actionable time‑tagged commands. The new window transforms SGP4/TLE ephemerides into **target‑oriented, traceable** time slots, at scale, without freezing the UI. Validated against vanilla Gpredict (AOS/LOS, az/el, ground tracks) across Linux/WSL/Windows.

---

## UI tour

### 1) Ephemeris
- Choose **Hours** (horizon) and **Step** (seconds, down to **1 s**).  
- The view streams **Time / Lat / Lon** rows incrementally, with a **Total** counter and a **progress** indicator.  
- Implementation keeps the GUI responsive on long horizons.

### 2) Territory
- Type to search a country/territory; table lists **Time / Lat / Lon / Country** matches.  
- Under the hood, matches are powered by the `territories.csv` tiling & majority‑area mapping to ISO‑3166 (incl. overseas).

### 3) Points of Interest
- Load a POI library; for each target, the controller selects the **minimum‑range instant** within its **tile size** and computes the **bearing**.  
- **Save** exports a normalized **CSV (UTF‑8 with BOM)**: `UTC, lat, lon, range_km, azimuth_deg, Name, Type` with a metadata header.

---

## Data pipeline (ESA WorldCover → CSVs)

- Start from **ESA WorldCover** and administrative polygons.  
- Build a global **tile grid** in WGS‑84; assign each tile to a country by **majority surface** (edge cases: coastal/frontier/islands handled; **DROM‑COM** kept separate from metropolitan codes).  
- Normalize names/codes and export to compact **`territories.csv`**.  
- Build **`poi.csv`** with center/type and a **tile size (km)** to represent extended targets (cities, glaciers, critical sites).

> These CSVs keep the app fast, portable, and easy to ship alongside the binaries.

---

## Architecture & performance notes

- **Worker thread** for propagation (SGP4/TLE) + **chunked insertion** into GTK models (no blocking on large horizons).  
- **Deterministic formatting** and reproducible exports (UTC, UTF‑8 + BOM).  
- Robust cancellations and widget lifecycle; **non‑regression** vs. Gpredict maintained.

---

## Installation

### Windows (recommended for operators)
1. Run **`OGpredict-Setup.exe`** (NSIS).  
   The installer places the executable, adjacent DLLs, CSV resources (icons, `data/`), and creates shortcuts/uninstaller.  
2. You may also use the portable **`OGpredict.exe`** with the adjacent DLLs and a helper **`.bat`** that sets `PATH` then launches the app.

### Linux / WSL (from source)
1. Ensure build dependencies (Autotools toolchain, GTK/GLib, curl, goocanvas, etc.).  
2. Generate scripts, configure, build, run under your X server (WSLg on Windows also works).  
3. Typical flow mirrors Gpredict’s Autotools build: `autoreconf -fi && ./configure && make -j$(nproc) && ./src/ogpredict`.

---

## Quick start

1. **Update TLE** in Gpredict as usual (fresh TLEs are essential for 1–3‑day accuracy with SGP4).  
2. Open **Ephemeris Data** → set **Hours** and **Step (1 s)** → **Run**.  
3. Switch to **Territory** or **POI** to focus on your targets; watch counters/progress.  
4. **Save** → CSV (UTF‑8+BOM) with a metadata header (TLE source, step, horizon).

---

## Validation & readiness

- Cross‑checked AOS/LOS, az/el, ground tracks against vanilla Gpredict on **Linux/WSL/Windows**; exports are **deterministic** and identical at the second when TLE, step, and horizons match.  
- Project maturity: from **TRL 1–2 → TRL 6–7** (representative demo), then **TRL 8** via Windows packaging and operator uptake.

---

## Roadmap (suggested)

- **Multi‑station** planning  
- **Geospatial index** (R‑tree/quad‑tree) backing Territory/POI for long horizons  
- **CLI / API** (headless) for nightly batch generation  
- **CI multi‑OS** with signed artifacts  
- **ADCS & illumination gates**  
- **POI library management** (bundles, tags)

---

## License (GNU GPL)

OGpredict is a **derivative work of Gpredict** and is distributed **under the same GNU General Public License terms** as Gpredict. As required by the GPL, the **full corresponding source code** for OGpredict, including your modifications, must be made available; the license grants users the freedoms to run, study, share, and modify the software, and requires that **any redistribution of binaries be accompanied by the corresponding source** (or a written offer), with **derivatives remaining under the GPL**. Please keep the repository’s `COPYING` file and **preserve copyright notices** and license headers where present.

> Practical notes:
> - If you distribute Windows installers (`.exe`/NSIS), include the **source** (or a valid **offer**) in the release.  
> - Document any third‑party data/files you ship (e.g., **ESA WorldCover‑derived CSVs**) with appropriate attribution and licenses in `data/README`.

---

## Credits

- **Gpredict** by its original authors and contributors (GTK/GLib, SGP4/SDP4).  
- **ESA WorldCover** for the land‑cover basemap used in the data‑reduction pipeline.

---

## Citing OGpredict

If you use OGpredict in research, please cite the technical report sections describing the architecture, data pipeline, and packaging (Ephemeris Data window; Windows MSYS2/NSIS chain; CSV exports).

---

### Maintainer

Axel Osika — initial author of the OGpredict extensions (Ephemeris Data, Territory, POI, packaging). For issues/PRs, please use this repository’s issue tracker.
