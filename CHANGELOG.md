# Changelog

## 0.1.0 (2026-11-24) - first public release

- Understory share from leaf-off USGS 3DEP lidar (returns 0.5-3 m above ground vs all returns below 3 m).
- Boulder and ledge recovery where the ground classification removed them.
- Terrain line of sight from 24 directions; "hidden ground" map.
- 0-10 obstruction index output (0 = wide open, 10 = a wall).
- Lidar from cached USGS tiles, Amazon's EPT copy of 3DEP, or USGS tiles; resumable downloads, 429 back-off.
- Processed 500 m squares cached and reused; re-checked when the source lidar changes.
- Tkinter map window: pick a center, draw a box, process a whole box overnight, map everything processed.
