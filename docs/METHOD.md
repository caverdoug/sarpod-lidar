# Method

*This document is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).  The code is MIT.*

Status: experimental.  The constants below are placeholders chosen to give sensible relative maps.  None of them have been fitted to field data.

## 1. Lidar

Leaf-off USGS 3DEP point clouds.  Points classed 7 and 18 (noise) are dropped.  All work is done on a 1 m grid in the collection's UTM zone (WGS84).

## 2. Bare earth

The DEM is the mean height of class-2 (ground) returns in each 1 m cell, with gaps filled from the nearest ground cell and a light smoothing (sigma 0.7 m).  The ground classification is the USGS-QC'd one; it is not re-run.

## 3. Understory share

For each cell, over a 3 x 3 m window:

    understory = returns 0.5-3 m above ground / all returns below 3 m

The 0.5 m floor keeps grass and leaf litter out; the 3 m ceiling keeps canopy out.  Returns below 3 m are the ones that made it through the canopy, so the ratio is "of the pulses that reached person height, how many were stopped by something at person height."  The result is smoothed with a Gaussian (sigma 5 m) so each cell reflects the ground a searcher would be looking across.

## 4. Boulders and ledges

Ground classification removes large rocks.  Within a 3 x 3 window, a cell is marked solid when no return falls below 1.5 m above the DEM, there are at least 4 returns, and the lowest return is 1.5-12 m up.  Solid patches are grown into neighbours whose own returns are all 1.5 m or higher, and patches under 3 m² are dropped.  The lowest return height is added to the DEM, and understory is set to zero on solid cells (it is rock, not brush).

## 5. Line of sight

From each cell, 24 rays out to 2.5·s (about 19 m with the defaults), where s is defined in step 6.  The observer's eye is 1.6 m above the ground at distance d; the object is 0.3 m above the ground at the cell.  Stepping outward from the object, keep the running maximum slope M of the terrain (DEM + rock) seen from the object.  A look from distance d is visible (V = 1) if

    (z_eye + 1.6 - z_object - 0.3) / d  >=  M

## 6. Combining them

The value per cell:

    W = W_open · Σ w(d) · V(d) · exp(-K · D · d / 10)  /  Σ w(d)

- D is the smoothed understory share at the cell (step 3).
- w(d) = 0.8 · exp(-d² / 2s²) · d, a bell-shaped detection curve weighted by distance, with s = W_open / (2.4755 · 0.8).
- W_open = 15 m and K = 6 are placeholders.

On flat open ground W = W_open.  The ratio W_terrain / W_flat, with V = 1 everywhere, gives the hidden-ground classes: 10-25%, 25-50% and 50%+ of looks blocked.

## 7. Obstruction index (0-10)

    index = round(10 · (1 - W / W_open)), clipped 0-10

0 = nothing between you and the ground at 15-20 m, 10 = a wall.  Because W scales with W_open, the index depends on K and the terrain but not on the W_open value.  This is the number intended for field checking: experienced people go to fixed points and rate them on the same 0-10 scale, and the two are compared.

## 8. Map classes

The understory map sheet uses four classes on W: under 4 m, 4-8 m, 8-12 m, 12 m and up.  Four was chosen for readability on a map; the full range is in the DATA and 0-10 files.

## 9. Processing and caching

Areas are split into 500 m squares on a fixed grid with a 40 m halo, so every square is computed the same way whether it was part of a circle or a box run.  Each square stores the DEM, rock heights, understory and coverage (about 4 MB per km²) along with a fingerprint of the lidar it came from.  A change in the source lidar or in the processing version rebuilds the square.
