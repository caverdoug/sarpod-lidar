# SARPOD Lidar

Understory visibility and line-of-sight maps from leaf-off airborne lidar.

> **Experimental research tool.  Not validated against field data.  Do not use it for operational decisions.**  Every output is best read as a relative ranking of ground (this patch is thicker than that one), not as a measured distance.

## What it does

Airborne lidar records every return a laser pulse makes on the way down: canopy, brush, stems, rocks and the ground.  The USGS 3DEP collections are flown leaf-off, so the brush layer that blocks a person's view at ground level is in the data.  This tool counts it.

For every 1 m cell it builds two layers:

1. **Understory.**  The share of returns between 0.5 m and 3 m above the ground out of all returns below 3 m.  Canopy above 3 m is ignored.  Averaged over the surrounding 10-15 m.
2. **Hidden ground (line of sight).**  From 24 directions around each cell, an eye at 1.6 m looks at an object lying on the ground (0.3 m) out to about 19 m, against the bare earth.  Boulders and ledges that the ground classification removed are put back first.  Cells where the terrain alone blocks a large share of those looks are flagged.  Think of someone lying behind a boulder, in a gully or below a ledge.

It is the same idea as render distance in a block game: the world is a grid, and you check what a ray from the eye hits before it reaches the target.

## Outputs

For a circle around a point, or a drawn box:

| File | What it is |
| --- | --- |
| `*_sweepwidth_webmercator.tif` | Understory map, 4 green classes, RGBA in EPSG:3857.  Drops straight into CalTopo / SARTopo as a map sheet, or any GIS. |
| `*_hidden_ground_webmercator.tif` | Where terrain hides the ground: orange 10-25%, red 25-50%, dark red 50%+ of looks blocked. |
| `*_obstruction_0to10.tif` | Obstruction index per cell, 0 = wide open, 10 = a wall.  uint8, 255 = no data. |
| `*_sweepwidth_DATA_for_QGIS.tif` | The continuous value per cell (float, metres) for analysis. |
| `*_sweepwidth_summary.txt` | Which lidar was used and the share of ground in each class. |

The "metres" come from two placeholder constants (W_open 15 m, K 6) and a bell-shaped detection curve.  They are not calibrated.  The file names keep "sweepwidth" for now; that is where this started.

## Quick start

### Windows (no Python install)

Download the zip from the [Releases](https://github.com/caverdoug/sarpod-lidar/releases) page, unzip it anywhere, and double-click **SARPOD Lidar.bat**.

### Any platform

```
pip install -r requirements.txt
python sarpod_lidar_gui.pyw                                   # map window
python lidar_sweep_map.py --at "39.616, -79.823" --radius 1000  # command line
python lidar_sweep_map.py export.json --marker IPP --radius 2000  # marker from a CalTopo/SARTopo export
```

In the map window, double-click the map to set the center (or type `lat, lon`), set a radius and click **Make map**.  **Draw box** then **Process all ground** works through a large area in chunks and can run unattended overnight; **Map processed ground** makes one map sheet of everything already processed in the box.

Useful flags: `--source auto|aws|usgs`, `--no-terrain`, `--workers 16`, `--set-cache`, `--list-only`.  Run `python lidar_sweep_map.py -h` for all of them.

## Lidar sources

- **Amazon** (default when it covers the area): the public `usgs-lidar-public` Entwine Point Tiles copy of 3DEP.  Only the pieces inside your area are fetched, and it is not throttled.
- **USGS**: whole LAZ tiles from The National Map.  Slow (about 0.1 MB/s per connection) but has collections Amazon lacks.  Downloads are size-checked and resumed.

Rough speed on one desktop: 1.3-1.5 km² per minute once the lidar is local.

## Where files go

| What | Default location (Windows) | Change it with |
| --- | --- | --- |
| Lidar downloads | `C:\Users\<you>\SARPOD\lidar_cache` | **Lidar cache...** button, **Set lidar cache folder.bat**, `--set-cache`, or the `SARPOD_LIDAR_CACHE` environment variable |
| Processed ground (reusable 500 m squares) | `<lidar cache>\_processed` | moves with the lidar cache |
| Finished maps (GeoTIFFs + summary) | your Downloads folder, or next to the SARTopo export if you loaded one | **Save maps in...** button, `--out` |

On Mac and Linux the default cache is `~/SARPOD/lidar_cache`.  The chosen cache folder is remembered in `~/SARPOD/lidar_cache_location.txt`; the environment variable, if set, wins over it.

### What's in the cache

```
lidar_cache\
  <USGS project>\                      whole USGS LAZ tiles (often 50-170 MB each)
  AWS_<collection>\ept-data\           small Amazon pieces (only the ones your areas needed)
  AWS_<collection>\ept-hierarchy\      Amazon index files
  _aws_index\                          list of collections on Amazon (refreshed every 30 days)
  _processed\<source>_<collection>\v1\ processed 500 m squares (E_N.npz) + crs.wkt
```

### How big it gets

- **Lidar is the big part.**  A 2 km radius circle is typically a few GB of USGS tiles (tiles run past the circle's edge), less from Amazon since only the pieces inside the area come down.  A large box can run to tens of GB.  Put the cache on a drive with room to spare.
- **Processed squares are small**: about 3-4.5 MB per km².  A 250 km² box is around 1 GB.

### Cleaning up

- **Lidar folders can be deleted** once an area is processed.  Runs over ground that is already processed don't need the lidar at all.  If something later needs it (a new area next door, or a new version of the tool that changes how ground is processed), it is downloaded again.
- **Keep `_processed`.**  That is what makes repeat runs fast; deleting it means reprocessing from lidar.
- Partly downloaded files (`.part`) are resumed on the next run; you don't need to delete them.

### Keeping it current

Every run checks which lidar is best for the area.  Amazon collections are re-checked weekly; if one has been updated, its cached pieces are cleared and the affected squares rebuilt.  USGS tiles are checked by file size.  A newer collection or a new processing version means squares are rebuilt rather than reused.

### Sharing a cache between computers

Point every computer at the same folder (for example in Google Drive for desktop) with **Set lidar cache folder.bat**.  Processing done on one computer is then reused by the others.  With Google Drive, mark the folder "Available offline": reading thousands of small Amazon pieces straight from the cloud is several times slower.

## Limits

- Leaf-off only.  Summer leaves are not in the data.
- Point density varies by collection; sparse older collections undercount brush.
- Ground classification is whatever the USGS QC'd.  Misclassified ground affects everything downstream.
- Stems and brush are counted, not identified.  A laurel thicket and a blowdown can score alike.
- No calibration yet.  The plan is to compare the 0-10 index against experienced searchers rating fixed points in the field on the same 0-10 scale.

## Ideas wanted

Issues and pull requests are welcome, especially field data that can be used to check the index, better handling of steep slopes and cliffs, and ways to speed up processing for statewide runs.

## Method and background

See [docs/METHOD.md](docs/METHOD.md).

## Credits

- Lidar: U.S. Geological Survey 3D Elevation Program (3DEP).  Amazon copy: `usgs-lidar-public` (Entwine Point Tiles); index from Hobu, Inc.
- Basemaps in the map window: OpenStreetMap contributors, OpenTopoMap (CC-BY-SA).
- Detection-curve and sweep width background: B.O. Koopman, *Search and Screening*; ground search sweep width experiments by the U.S. Coast Guard and others.
- Vegetation-from-lidar visibility background: Lee et al. 2013, U.S. Army ERDC, https://doi.org/10.21079/11681/35175, and earlier U.S. Army Tropic Test Center work on line of sight in vegetation.

## License

Code: MIT (see [LICENSE](LICENSE)).  Documentation in `docs/`: CC BY 4.0.
