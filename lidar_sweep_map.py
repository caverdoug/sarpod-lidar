#!/usr/bin/env python3
"""
lidar_sweep_map.py - understory visibility and line-of-sight maps from leaf-off airborne lidar (SARPOD Lidar).

EXPERIMENTAL RESEARCH TOOL.  Not validated against field data.  Do not use it for operational decisions.

  python lidar_sweep_map.py --at "39.61654, -79.82290" --radius 2000     (a center by coordinates)
  python lidar_sweep_map.py export.json --marker IPP --radius 2000        (a marker from a SARTopo / CalTopo export)
  python sarpod_lidar_gui.pyw                                             (the map window; also does whole-box runs)

What it does:
  1. finds the lidar for a circle (or a box): USGS tiles if they're all in the cache already; otherwise Amazon's copy
     of the USGS 3DEP lidar (only the pieces inside the area, not throttled); otherwise USGS tiles (fetched in
     parallel).  --source aws / usgs forces one.  Downloads are cached and resumed.
  2. bare earth from the ground-classified returns (1 m cells).
  3. understory share at each 1 m cell = returns 0.5-3 m above ground / all returns below 3 m (3 x 3 m window),
     averaged over the surrounding ~10-15 m (Gaussian, sigma 5 m).  Canopy above 3 m is ignored.
  4. boulders and rock ledges that the ground classification removed are put back: where no return reaches within
     1.5 m of the ground over a few m2, the lowest return is the top of something solid.
  5. line of sight: from 24 directions around each cell, out to ~19 m, an observer's eye (1.6 m) to an object on the
     ground (0.3 m), checked against the bare earth (+ rock) and thinned by understory along the ray.
  6. writes, clipped to the circle (or box):
       <name>_sweepwidth_webmercator.tif      understory map, 4 green classes, RGBA EPSG:3857 (CalTopo map sheet)
       <name>_hidden_ground_webmercator.tif   where terrain alone hides the ground (orange 10-25%, red 25-50%, dark red 50%+)
       <name>_obstruction_0to10.tif           obstruction index per cell, 0 = wide open, 10 = a wall (uint8, 255 = no data)
       <name>_sweepwidth_DATA_for_QGIS.tif    the continuous value per cell, in metres (float, for QGIS / analysis)
       <name>_sweepwidth_summary.txt          which lidar was used and the share of ground in each class
     The "metres" are an estimated effective sweep width that uses two PLACEHOLDER constants (W_open 15 m, K 6) and
     a bell-shaped lateral range curve; they are NOT calibrated.  Read every output as a relative ranking.

Leaf-off lidar is what you want (it sees evergreen thickets and woody stems through bare canopy); summer leaves are
not in it.  Processed ground is cached in 500 m squares so later runs nearby are fast.

Needs: numpy scipy pyproj laspy[lazrs] rasterio      (pip install numpy scipy pyproj "laspy[lazrs]" rasterio)
License: MIT.  See README.md and docs/METHOD.md.
"""
import argparse, json, math, os, re, sys, time, threading, urllib.request, urllib.parse, datetime as dt
from concurrent.futures import ThreadPoolExecutor, as_completed

TNM_API = 'https://tnmaccess.nationalmap.gov/api/v1/products'
DATASET = 'Lidar Point Cloud (LPC)'
CLASSES = [(0, 4, (0x00, 0x44, 0x1B), 170, 'Thicket (W < 4 m)'), (4, 8, (0x23, 0x8B, 0x45), 140, 'Dense (4-8 m)'),
           (8, 12, (0x74, 0xC4, 0x76), 110, 'Moderate (8-12 m)'), (12, 999, (0xE5, 0xF5, 0xE0), 70, 'Open (12 m+)')]


HOOK = None                      # a window can set this to get progress: HOOK(event, **info)
CANCEL = threading.Event()       # set to stop a run


class Cancelled(RuntimeError):
    pass


def _emit(ev, **kw):
    if HOOK:
        try: HOOK(ev, **kw)
        except Exception: pass


def _check():
    if CANCEL.is_set(): raise Cancelled('Stopped.')


SETTING = os.path.join(os.path.expanduser('~'), 'SARPOD', 'lidar_cache_location.txt')


def default_cache():
    """Where lidar tiles are kept: the SARPOD_LIDAR_CACHE environment variable, else the folder named in
    ~/SARPOD/lidar_cache_location.txt (set with --set-cache, e.g. a Google Drive folder shared by several
    computers), else ~/SARPOD/lidar_cache."""
    v = os.environ.get('SARPOD_LIDAR_CACHE', '').strip()
    if not v and os.path.exists(SETTING):
        v = open(SETTING, encoding='utf-8').read().strip()
    return v or os.path.join(os.path.expanduser('~'), 'SARPOD', 'lidar_cache')


def set_cache_folder():
    """Pick the lidar cache folder (e.g. G:\\My Drive\\SARPOD\\lidar_cache) and remember it on this computer."""
    path = ''
    try:
        import tkinter, tkinter.filedialog
        root = tkinter.Tk(); root.withdraw(); root.attributes('-topmost', True)
        print('A folder picker is opening (it may be behind other windows).')
        path = tkinter.filedialog.askdirectory(parent=root, title='Folder for the lidar tiles (e.g. in Google Drive)')
    except Exception:
        pass
    if not path: path = input('Folder for the lidar tiles: ').strip().strip('"')
    if not path: print('Nothing changed.'); return
    os.makedirs(path, exist_ok=True); os.makedirs(os.path.dirname(SETTING), exist_ok=True)
    open(SETTING, 'w', encoding='utf-8').write(os.path.normpath(path))
    print(f'Lidar tiles on this computer will now be kept in:  {os.path.normpath(path)}')
    old = os.path.join(os.path.expanduser('~'), 'SARPOD', 'lidar_cache')
    if os.path.isdir(old) and os.path.normpath(old) != os.path.normpath(path) and os.listdir(old):
        print(f'Tiles from earlier runs are still in {old}.  Move the folders inside it into the new folder to keep them.')


# ---------------------------------------------------------------- SARTopo markers
def read_markers(path):
    d = json.load(open(path, encoding='utf-8'))
    if isinstance(d, dict) and 'state' in d.get('result', {}): d = d['result']['state']
    feats = d.get('features', d) if isinstance(d, dict) else d
    out = []
    for f in feats:
        g, p = f.get('geometry') or {}, f.get('properties') or {}
        if g.get('type') == 'Point' and p.get('class', 'Marker') in ('Marker', 'Clue', None):
            lon, lat = g['coordinates'][:2]
            out.append(((p.get('title') or '').strip() or '(untitled)', lon, lat))
    return out


def parse_latlon(text):
    """'39.61654, -79.82290', '39.61654 -79.82290', '39 36 59.5 N 79 49 22.4 W', 39°36'59.5"N 79°49'22.4"W, or
    degrees + decimal minutes.  Returns (lat, lon) or raises ValueError."""
    t = text.strip().upper().replace(',', ' ')
    t = re.sub(r"[°'\"`′″:]", ' ', t)
    hemi = re.findall(r'[NSEW]', t)
    nums = [float(v) for v in re.findall(r'[-+]?\d+(?:\.\d+)?', t)]
    if len(nums) not in (2, 4, 6): raise ValueError('Give latitude and longitude, e.g. 39.61654, -79.82290')
    k = len(nums) // 2
    def dms(v): return abs(v[0]) + (v[1] / 60 if len(v) > 1 else 0) + (v[2] / 3600 if len(v) > 2 else 0)
    la, lo = dms(nums[:k]), dms(nums[k:])
    if nums[0] < 0 or (len(hemi) > 0 and hemi[0] == 'S'): la = -la
    if nums[k] < 0 or (len(hemi) > 1 and hemi[1] == 'W') or (len(hemi) == 1 and hemi[0] == 'W'): lo = -lo
    if len(hemi) == 0 and lo > 0 and -125 < -lo < -66 and 24 < la < 50: lo = -lo   # US: west longitude typed without sign
    if not (-90 <= la <= 90 and -180 <= lo <= 180): raise ValueError('Coordinates out of range.')
    return la, lo


def pick_marker(markers, want):
    if want:
        hits = [m for m in markers if want.lower() == m[0].lower()] or [m for m in markers if want.lower() in m[0].lower()]
        if len(hits) == 1: return hits[0]
        if not hits: sys.exit(f'No marker matching "{want}".  Markers: ' + ', '.join(m[0] for m in markers))
        print(f'"{want}" matches several markers; pick one:')
        markers = hits
    for n, m in enumerate(markers, 1): print(f'  {n:3}. {m[0]}   ({m[2]:.5f}, {m[1]:.5f})')
    while True:
        v = input('Marker number: ').strip()
        if v.isdigit() and 1 <= int(v) <= len(markers): return markers[int(v) - 1]


# ---------------------------------------------------------------- USGS tile list + download
def bbox_around(lon, lat, radius_m, pad_m=250):
    r = radius_m + pad_m
    dlat = r / 110540.0; dlon = r / (111320.0 * math.cos(math.radians(lat)))
    return lon - dlon, lat - dlat, lon + dlon, lat + dlat


def area_bbox(lon, lat, radius_m):
    """Lon/lat box of lidar a run needs: whole 500 m squares around the circle plus a halo, so every square
    the run touches can be finished and saved."""
    from pyproj import CRS, Transformer
    utm = CRS.from_epsg((32600 if lat >= 0 else 32700) + int((lon + 180) // 6) + 1)
    cx, cy = Transformer.from_crs(4326, utm, always_xy=True).transform(lon, lat)
    h = radius_m + 30
    e0 = math.floor((cx - h) / SQ) * SQ - HALO - 60; e1 = math.ceil((cx + h) / SQ) * SQ + HALO + 60
    n0 = math.floor((cy - h) / SQ) * SQ - HALO - 60; n1 = math.ceil((cy + h) / SQ) * SQ + HALO + 60
    lo, la = Transformer.from_crs(utm, 4326, always_xy=True).transform([e0, e1, e0, e1], [n0, n0, n1, n1])
    return min(lo), min(la), max(lo), max(la)


def tnm_products(bbox, log=print):
    items, offset = [], 0
    while True:
        q = urllib.parse.urlencode({'bbox': ','.join(f'{v:.6f}' for v in bbox), 'datasets': DATASET, 'max': 200,
                                    'offset': offset, 'outputFormat': 'JSON'})
        for attempt in range(4):
            try:
                with urllib.request.urlopen(TNM_API + '?' + q, timeout=90) as r:
                    d = json.load(r); break
            except Exception as e:
                if attempt == 3: raise RuntimeError(f'USGS lidar search failed: {e}')
                time.sleep(3 * (attempt + 1))
        got = d.get('items', [])
        items += got
        if len(got) < 200 or len(items) >= d.get('total', 0): break
        offset += 200
    return items


def item_url(it):
    for k in ('downloadLazURL', 'downloadURL'):
        u = it.get(k) or ''
        if u.lower().endswith(('.laz', '.las')): return u
    return ''


def project_of(url):
    m = re.search(r'/Projects/([^/]+)/', url)
    return m.group(1) if m else 'unknown'


def choose_project(items, bbox, prefer=None, log=print, cover_bbox=None):
    """Group tiles by lidar project; pick the one covering the most of the area, newest first on ties."""
    cb = cover_bbox or bbox
    groups = {}
    for it in items:
        u = item_url(it)
        if not u: continue
        b = it.get('boundingBox') or {}
        if b and (b['maxX'] < bbox[0] or b['minX'] > bbox[2] or b['maxY'] < bbox[1] or b['minY'] > bbox[3]): continue
        groups.setdefault(project_of(u), []).append(it)
    if not groups: raise RuntimeError('USGS has no lidar point cloud tiles for this area.')
    area = (cb[2] - cb[0]) * (cb[3] - cb[1])

    def cover(tiles):
        a = 0.0
        for it in tiles:
            b = it.get('boundingBox') or {}
            if not b: continue
            a += max(0, min(b['maxX'], cb[2]) - max(b['minX'], cb[0])) * max(0, min(b['maxY'], cb[3]) - max(b['minY'], cb[1]))
        return min(1.0, a / area)

    rows = []
    for p, tiles in groups.items():
        date = max((t.get('publicationDate') or '') for t in tiles)
        rows.append((p, tiles, cover(tiles), date, sum(int(t.get('sizeInBytes') or 0) for t in tiles)))
    rows.sort(key=lambda r: (round(r[2], 2), r[3]), reverse=True)
    log('Lidar available for this area:')
    for p, tiles, c, date, size in rows:
        log(f'  {p:55} {len(tiles):4} tiles  {size / 1e6:7.0f} MB  covers {100 * c:3.0f}%  published {date[:10]}')
    if prefer:
        hit = [r for r in rows if prefer.lower() in r[0].lower()]
        if not hit: raise RuntimeError(f'No lidar project matching "{prefer}".')
        rows = hit
    p, tiles, c, date, size = rows[0]
    log(f'Using {p} ({len(tiles)} tiles, {size / 1e6:.0f} MB, covers {100 * c:.0f}% of the area).')
    if c < 0.95: log('  WARNING: this project does not cover the whole circle; uncovered ground will be left blank.')
    return p, tiles


def tiles_cached(tiles, cache_dir):
    for it in tiles:
        fn = os.path.join(cache_dir, os.path.basename(urllib.parse.urlparse(item_url(it)).path)); size = int(it.get('sizeInBytes') or 0)
        if not os.path.exists(fn) or (size and os.path.getsize(fn) < 0.99 * size): return False
    return True


def download_tiles(tiles, cache_dir, workers=16, log=print):
    os.makedirs(cache_dir, exist_ok=True)
    todo, paths = [], []
    for it in tiles:
        u = item_url(it); fn = os.path.join(cache_dir, os.path.basename(urllib.parse.urlparse(u).path))
        paths.append(fn)
        size = int(it.get('sizeInBytes') or 0)
        if os.path.exists(fn) and size and os.path.getsize(fn) < 0.99 * size:     # cut-off download from an older run
            log(f'  {os.path.basename(fn)} is incomplete ({os.path.getsize(fn) / 1e6:.0f} of {size / 1e6:.0f} MB); downloading it again.')
            os.replace(fn, fn + '.part')                                         # resume from where it stopped
        if not os.path.exists(fn): todo.append((u, fn, size))
    if not todo:
        log(f'All {len(paths)} files already in {cache_dir}'); return paths
    total = sum(t[2] for t in todo)
    log(f'Downloading {len(todo)} files' + (f' ({total / 1e6:.0f} MB)' if total else '') +
        f' to {cache_dir}, {workers} at a time.  {len(paths) - len(todo)} already cached.')
    done = [0]; nfiles = [0]; lock = threading.Lock(); t0 = time.time()
    # USGS answers "429 Too Many Requests" if it thinks we have too many downloads open.  Then everyone backs off,
    # the number allowed at once drops by a quarter (never below 4), and the refused tile waits its turn again.
    slots = {'max': workers, 'busy': 0, 'calm_until': 0.0}; cond = threading.Condition()

    def acquire():
        with cond:
            while slots['busy'] >= slots['max'] or time.time() < slots['calm_until']:
                _check()
                cond.wait(timeout=min(1.0, max(0.2, slots['calm_until'] - time.time())))
            slots['busy'] += 1

    def release():
        with cond:
            slots['busy'] -= 1; cond.notify_all()

    def too_many(e):
        ra = e.headers.get('Retry-After') if getattr(e, 'headers', None) else None
        wait = int(ra) if ra and ra.isdigit() else 60
        with cond:
            if time.time() >= slots['calm_until']:                     # one cut per back-off, not one per refused tile
                new = max(4, int(slots['max'] * 0.75))
                if new < slots['max']: log(f'  USGS says too many downloads at once; slowing to {new} at a time.')
                slots['max'] = new
            slots['calm_until'] = max(slots['calm_until'], time.time() + wait); cond.notify_all()

    def get(u, fn, size):
        part = fn + '.part'; err = None; tries = 0
        while tries < 8:
            acquire()
            _emit('piece', fn=fn, state='busy')
            try:
                have = os.path.getsize(part) if os.path.exists(part) else 0
                req = urllib.request.Request(u, headers={'Range': f'bytes={have}-'} if have else {})
                with urllib.request.urlopen(req, timeout=120) as r:
                    if have and r.status != 206: have = 0                 # server ignored the range: start over
                    clen = r.headers.get('Content-Length')
                    expect = have + int(clen) if clen and clen.isdigit() else (size or 0)
                    with open(part, 'ab' if have else 'wb') as fh:
                        while True:
                            b = r.read(1 << 20)
                            if not b: break
                            fh.write(b)
                            with lock: done[0] += len(b)
                            _check()
                got = os.path.getsize(part)
                if expect and got < expect:                               # connection dropped quietly: resume
                    raise IOError(f'cut off at {got / 1e6:.0f} of {expect / 1e6:.0f} MB')
                os.replace(part, fn)
                with lock: nfiles[0] += 1
                _emit('piece', fn=fn, state='done')
                return fn
            except urllib.error.HTTPError as e:
                if e.code == 416 and os.path.exists(part) and (not size or os.path.getsize(part) >= 0.99 * size):
                    os.replace(part, fn); return fn
                if e.code == 416 and os.path.exists(part): os.remove(part)  # bad partial: start over
                err = e
                if e.code in (429, 503):
                    _emit('piece', fn=fn, state='wait'); too_many(e); continue   # not counted as a failure
            except Cancelled:
                raise
            except Exception as e:
                err = e
            finally:
                release()
            tries += 1
            _emit('piece', fn=fn, state='wait')
            for _ in range(5 * tries): _check(); time.sleep(1)
        _emit('piece', fn=fn, state='fail')
        raise RuntimeError(f'{os.path.basename(fn)} ({err})')

    stop = threading.Event()

    def progress():
        n = 0
        while not stop.wait(1):
            n += 1
            _emit('bytes', done=done[0], total=total, files=nfiles[0], nfiles=len(todo), at_once=slots['max'],
                  rate=done[0] / max(time.time() - t0, 1))
            if n % 15: continue
            el = time.time() - t0; mb = done[0] / 1e6
            of = f'of {total / 1e6:.0f} MB' if total else f'MB, {nfiles[0]} of {len(todo)} files'
            log(f'  {mb:6.0f} {of}  ({mb / max(el, 1):.1f} MB/s, {slots["max"]} at a time)')
    th = threading.Thread(target=progress, daemon=True); th.start()
    failed = []
    try:
        with ThreadPoolExecutor(workers) as ex:
            for f in as_completed([ex.submit(get, *t) for t in todo]):
                try: f.result()
                except Cancelled: CANCEL.set()
                except Exception as e: failed.append(str(e))
    finally:
        stop.set()
        _emit('bytes', done=done[0], total=total, files=nfiles[0], nfiles=len(todo), at_once=slots['max'], rate=0)
    _check()
    if failed:
        raise DownloadIncomplete(f'{len(failed)} of {len(todo)} tiles did not download:\n  ' + '\n  '.join(failed) +
                                 '\nEverything that did download is kept.  Run it again to get the rest.')
    log(f'Downloads finished in {(time.time() - t0) / 60:.1f} min.')
    return paths


# ---------------------------------------------------------------- Amazon copy of the USGS lidar (Entwine Point Tiles)
# USGS 3DEP lidar is also on AWS (s3://usgs-lidar-public) in EPT form: an octree of small LAZ files in Web Mercator.
# Only the pieces inside the circle are downloaded, and Amazon doesn't throttle like rockyweb.usgs.gov does.
EPT_INDEX = 'https://raw.githubusercontent.com/hobuinc/usgs-lidar/master/boundaries/resources.geojson'


def _get_json(url, cache_file=None, max_age=None, timeout=90):
    if cache_file and os.path.exists(cache_file) and (max_age is None or time.time() - os.path.getmtime(cache_file) < max_age):
        with open(cache_file, encoding='utf-8') as fh: return json.load(fh)
    err = None
    for attempt in range(5):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r: d = json.load(r)
            if cache_file:
                os.makedirs(os.path.dirname(cache_file), exist_ok=True)
                with open(cache_file + '.tmp', 'w', encoding='utf-8') as fh: json.dump(d, fh)
                os.replace(cache_file + '.tmp', cache_file)
            return d
        except urllib.error.HTTPError as e:
            if e.code == 404: raise
            err = e
        except Exception as e:
            err = e
        time.sleep(3 * (attempt + 1))
    if cache_file and os.path.exists(cache_file):             # offline: an old copy is better than nothing
        with open(cache_file, encoding='utf-8') as fh: return json.load(fh)
    raise RuntimeError(f'Could not read {url} ({err})')


def _outer_rings(geom):
    t, c = geom.get('type'), geom.get('coordinates') or []
    if t == 'Polygon': return [c[0]] if c else []
    if t == 'MultiPolygon': return [p[0] for p in c if p]
    return []


def _covered(rings, px, py):
    """Share of the sample points inside any of the rings (even-odd ray casting)."""
    import numpy as np
    inside = np.zeros(px.shape, bool)
    for ring in rings:
        r = np.asarray(ring, float); xa, ya = r[:-1, 0], r[:-1, 1]; xb, yb = r[1:, 0], r[1:, 1]
        if px.max() < r[:, 0].min() or px.min() > r[:, 0].max() or py.max() < r[:, 1].min() or py.min() > r[:, 1].max():
            continue
        hit = np.zeros(px.shape, bool)
        for i in range(len(xa)):
            cond = (ya[i] > py) != (yb[i] > py)
            xc = xa[i] + (py - ya[i]) * (xb[i] - xa[i]) / np.where(yb[i] != ya[i], yb[i] - ya[i], 1e-12)
            hit ^= cond & (px < xc)
        inside |= hit
    return inside


def choose_aws(bbox, cache, prefer=None, log=print, cover_bbox=None):
    """Pick the Amazon lidar collection covering the area (batches like _B1.._B4 of one collection are combined).
    Returns (name, [ept.json urls], coverage) or None."""
    import numpy as np
    idx = _get_json(EPT_INDEX, os.path.join(cache, '_aws_index', 'resources.geojson'), max_age=30 * 86400, timeout=180)
    cb = cover_bbox or bbox
    gx, gy = np.meshgrid(np.linspace(cb[0], cb[2], 30), np.linspace(cb[1], cb[3], 30))
    groups = {}
    for f in idx.get('features', []):
        p = f.get('properties') or {}; name, url = p.get('name', ''), p.get('url', '')
        rings = _outer_rings(f.get('geometry') or {})
        if not name or not url or not rings: continue
        ins = _covered(rings, gx, gy)
        if not ins.any(): continue
        key = re.sub(r'_B\d+(QL\d+)?(?=_|$)', '', name)
        g = groups.setdefault(key, {'urls': [], 'mask': np.zeros(gx.shape, bool)})
        g['urls'].append(url); g['mask'] |= ins
    if not groups: return None
    rows = []
    for key, g in groups.items():
        yrs = re.findall(r'(19|20)\d\d', key); yr = max(int(m.group(0)) for m in re.finditer(r'(19|20)\d\d', key)) if yrs else 0
        rows.append((key, g['urls'], float(g['mask'].mean()), yr))
    rows.sort(key=lambda r: (round(r[2], 2), r[3]), reverse=True)
    log('Lidar on Amazon (AWS) for this area:')
    for key, urls, c, yr in rows: log(f'  {key:45} {len(urls)} part(s)  covers {100 * c:3.0f}%')
    if prefer:
        rows = [r for r in rows if prefer.lower() in r[0].lower()] or rows
    return rows[0][0], rows[0][1], rows[0][2]


def aws_nodes(ept_url, box, cache_dir, log=print):
    """EPT nodes (LAZ pieces) that overlap box = (xmin, ymin, xmax, ymax) in Web Mercator."""
    base = ept_url.rsplit('/', 1)[0]
    prev = None
    pf = os.path.join(cache_dir, 'ept.json')
    if os.path.exists(pf):
        try:
            with open(pf, encoding='utf-8') as fh: prev = json.load(fh)
        except Exception: prev = None
    info = _get_json(ept_url, pf, max_age=7 * 86400)
    if prev and (prev.get('points'), prev.get('bounds')) != (info.get('points'), info.get('bounds')):
        import shutil
        log(f'  {os.path.basename(cache_dir)} was updated on Amazon; its cached pieces will be downloaded again.')
        for sub in ('ept-hierarchy', 'ept-data'): shutil.rmtree(os.path.join(cache_dir, sub), ignore_errors=True)
    if info.get('dataType') != 'laszip': raise RuntimeError(f'{base}: unsupported EPT data type {info.get("dataType")}')
    bx0, by0, bz0, bx1, by1, bz1 = info['bounds']; size = bx1 - bx0
    keys, todo, seen, nread = [], ['0-0-0-0'], set(), 0
    _emit('stage', name='Finding the lidar pieces on Amazon (index files read)', i=0, n=0)
    with ThreadPoolExecutor(16) as ex:                       # index files fetched 16 at a time, level by level
        while todo:
            batch = [hk for hk in dict.fromkeys(todo) if hk not in seen]; todo = []
            seen.update(batch)
            futs = {ex.submit(_get_json, f'{base}/ept-hierarchy/{hk}.json',
                              os.path.join(cache_dir, 'ept-hierarchy', hk + '.json')): hk for hk in batch}
            for f in as_completed(futs):
                hk = futs[f]; h = f.result(); nread += 1
                _emit('stage', name='Finding the lidar pieces on Amazon (index files read)', i=nread, n=0)
                _check()
                for key, cnt in h.items():
                    D, X, Y, Z = map(int, key.split('-')); s = size / 2 ** D
                    nx0, ny0 = bx0 + X * s, by0 + Y * s
                    if nx0 > box[2] or nx0 + s < box[0] or ny0 > box[3] or ny0 + s < box[1]: continue
                    if cnt == -1:
                        if key != hk: todo.append(key)
                    elif cnt > 0:
                        keys.append(key)
    log(f'  read {nread} index files, {len(keys)} pieces overlap the area')
    out = []
    for k in keys:
        D, X, Y, Z = map(int, k.split('-')); s_ = size / 2 ** D
        out.append({'downloadLazURL': f'{base}/ept-data/{k}.laz', 'sizeInBytes': 0, '_depth': D,
                    '_b3857': (bx0 + X * s_, by0 + Y * s_, bx0 + (X + 1) * s_, by0 + (Y + 1) * s_)})
    return out


def plan_aws(bbox, cache, prefer=None, log=print, cover_bbox=None, min_cover=0.95):
    """Which Amazon pieces cover bbox (lon/lat).  Returns a plan dict or None."""
    from pyproj import Transformer
    pick = choose_aws(bbox, cache, prefer, log, cover_bbox)
    if not pick: log('  none.'); return None
    name, urls, cov = pick
    if cov < min_cover:
        log(f'  {name} covers only {100 * cov:.0f}% of the area.'); return None
    tm = Transformer.from_crs(4326, 3857, always_xy=True); back = Transformer.from_crs(3857, 4326, always_xy=True)
    xs, ys = tm.transform([bbox[0], bbox[2], bbox[0], bbox[2]], [bbox[1], bbox[1], bbox[3], bbox[3]])
    box = (min(xs), min(ys), max(xs), max(ys))
    items = []
    for u in urls:
        cdir = os.path.join(cache, 'AWS_' + u.rstrip('/').split('/')[-2])
        for it in aws_nodes(u, box, cdir, log):
            b = it.pop('_b3857'); lo, la = back.transform([b[0], b[2]], [b[1], b[3]])
            it['_dir'] = os.path.join(cdir, 'ept-data'); it['_bounds'] = (lo[0], la[0], lo[1], la[1])
            items.append(it)
    fps = []
    for u in urls:
        inf = _get_json(u, os.path.join(cache, 'AWS_' + u.rstrip('/').split('/')[-2], 'ept.json'))
        fps.append(f"{u.rstrip('/').split('/')[-2]}:{inf.get('points')}:{inf.get('bounds')}")
    srs = _get_json(urls[0], os.path.join(cache, 'AWS_' + urls[0].rstrip('/').split('/')[-2], 'ept.json')).get('srs') or {}
    log(f'Using {name} from Amazon: {len(items)} pieces cover the area.')
    return {'name': name, 'source': 'Amazon', 'items': items, 'epsg': int(srs.get('horizontal') or 3857), 'fp': '|'.join(sorted(fps))}


def plan_lidar(bbox, cache, source='auto', prefer=None, log=print, cover_bbox=None, min_cover=0.95):
    """Decide where the lidar comes from.  auto: USGS tiles if all already cached, else Amazon, else USGS tiles."""
    tiles = proj = None
    if source in ('auto', 'usgs'):
        try:
            proj, tiles = choose_project(tnm_products(bbox, log), bbox, prefer, log, cover_bbox)
        except Exception as e:
            if source == 'usgs': raise
            log(f'USGS tile search: {e}')
        if tiles:
            d = os.path.join(cache, proj)
            for it in tiles:
                b = it.get('boundingBox') or {}
                it['_dir'] = d
                it['_bounds'] = (b['minX'], b['minY'], b['maxX'], b['maxY']) if b else None
            if source == 'usgs' or tiles_cached(tiles, d):
                if source == 'auto': log('All of these USGS tiles are already in the cache; using them.')
                return {'name': proj, 'source': 'USGS', 'items': tiles, 'epsg': None}
    if source in ('auto', 'aws'):
        try:
            p = plan_aws(bbox, cache, prefer, log, cover_bbox, min_cover)
            if p: return p
        except Exception as e:
            if source == 'aws': raise
            log(f'Amazon lidar not usable here ({e}).')
    if tiles:
        log('Using the USGS tiles.')
        return {'name': proj, 'source': 'USGS', 'items': tiles, 'epsg': None}
    raise RuntimeError('No lidar found for this area on USGS or Amazon.')


def piece_path(it):
    return os.path.join(it['_dir'], os.path.basename(urllib.parse.urlparse(item_url(it)).path))


def show_plan(plan):
    _emit('pieces', source=plan['source'], name=plan['name'],
          pieces=[(piece_path(it), it.get('_bounds'), it.get('_depth', 0), os.path.exists(piece_path(it))) for it in plan['items']])


def fetch_lidar(plan, workers=16, log=print):
    show_plan(plan)
    w = max(workers, 24) if plan['source'] == 'Amazon' else workers
    paths = []
    for d in sorted({it['_dir'] for it in plan['items']}):
        paths += download_tiles([it for it in plan['items'] if it['_dir'] == d], d, w, log)
    return paths

# ---------------------------------------------------------------- sweep width from the point cloud
def w_of_d_table(w_open, k, p0=0.8):
    import numpy as np
    s = w_open / (2.4755 * p0); d = np.arange(1, int(2.5 * s) + 1, dtype=float)
    w = p0 * np.exp(-d * d / (2 * s * s)) * d
    D = np.linspace(0, 1, 201)
    return D, w_open * (w[None, :] * np.exp(-k * D[:, None] * d[None, :] / 10)).sum(1) / w.sum()


class DamagedTile(RuntimeError):
    pass


class DownloadIncomplete(RuntimeError):
    pass


def build_sweep_width(tiles, lon, lat, radius, w_open, k, cell_m, log=print, terrain=True, src_epsg=None, sqcache=None):
    """Returns W, geo, crs, points read, W without terrain, solid m2, squares from cache, squares in all."""
    try:
        return _build_sweep_width(tiles, lon, lat, radius, w_open, k, cell_m, log, terrain, src_epsg, sqcache)
    except DamagedTile:
        raise


def _read_chunks(t, n=5_000_000):
    """Chunks of one tile; a damaged file is deleted so the next run downloads it again."""
    import laspy
    try:
        with laspy.open(t) as rd:
            h = rd.header
            yield h, None
            for ch in rd.chunk_iterator(n): yield h, ch
    except DamagedTile:
        raise
    except Exception as e:
        try: os.remove(t)
        except OSError: pass
        raise DamagedTile(f'Lidar tile {os.path.basename(t)} is damaged ({e}).  It has been deleted; run again and '
                          f'it will be downloaded fresh (the other tiles stay cached).')


def _addcount(dst, li, w=None):
    """dst[li] += w (or 1), touching only the stretch of dst the points fall in (fast for small pieces)."""
    import numpy as np
    if li.size == 0: return
    lo = int(li.min()); hi = int(li.max()) + 1
    dst[lo:hi] += np.bincount(li - lo, w, hi - lo)


def _points(tiles, overlaps, workers=12):
    """Yield (file number, chunk) for every file that overlaps, then (file number, None) when a file is finished.
    Big USGS tiles are read in chunks one at a time; small Amazon pieces are read many at once, which hides the
    delay of opening each file (a big deal when the cache is on Google Drive)."""
    from collections import deque
    sample = tiles[:: max(1, len(tiles) // 50)]
    small = len(tiles) > 30 and all(os.path.getsize(t) < 40e6 for t in sample)
    if not small:
        for i, t in enumerate(tiles, 1):
            for h, ch in _read_chunks(t):
                if ch is None:
                    if not overlaps(h): break
                    continue
                yield i, ch
            yield i, None
        return

    def load(t):
        import laspy
        try:
            with laspy.open(t) as rd:
                if not overlaps(rd.header): return None
                return rd.read_points(rd.header.point_count)
        except Exception as e:
            try: os.remove(t)
            except OSError: pass
            raise DamagedTile(f'Lidar piece {os.path.basename(t)} is damaged ({e}).  It has been deleted; run again and '
                              f'it will be downloaded fresh.')
    with ThreadPoolExecutor(workers) as ex:
        todo = iter(enumerate(tiles, 1)); q = deque()
        def more():
            nxt = next(todo, None)
            if nxt: q.append((nxt[0], ex.submit(load, nxt[1])))
        for _ in range(workers * 4): more()
        while q:
            i, f = q.popleft(); res = f.result(); more()
            if res is not None: yield i, res
            yield i, None


PROC_VERSION = 1          # bump whenever the way ground / brush / rock are measured changes: old squares get rebuilt
SQ = 500                  # processed squares, metres
HALO = 40                 # metres of extra lidar read around squares being built (smoothing, gap fill, boulder windows)


def _read_wkt(f):
    """The saved coordinate system.  Older files were written in Windows' own encoding (cp1252), so try both."""
    from pyproj import CRS
    with open(f, 'rb') as fh: raw = fh.read()
    for enc in ('utf-8', 'cp1252', 'latin-1'):
        try: return CRS.from_wkt(raw.decode(enc))
        except (UnicodeDecodeError, Exception): continue
    raise RuntimeError(f'Could not read {f}')


class SquareCache:
    """Processed 500 m squares (bare earth, rock heights, brush density, lidar coverage) for one lidar collection,
    so a run near an earlier one reads lidar only for new ground.  Lives in <lidar cache>/_processed/."""

    def __init__(self, root, plan):
        self.dir = os.path.join(root, '_processed', re.sub(r'[^A-Za-z0-9_.-]+', '_', f"{plan['source']}_{plan['name']}"), f'v{PROC_VERSION}')
        self.fp = plan.get('fp') or plan['name']
        self.pieces = {os.path.basename(piece_path(it)): (int(it.get('sizeInBytes') or 0), it.get('_bounds'))
                       for it in plan['items'] if plan['source'] == 'USGS'}

    def crs(self):
        f = os.path.join(self.dir, 'crs.wkt')
        if os.path.exists(f):
            from pyproj import CRS
            return _read_wkt(f)
        return None

    def set_crs(self, crs):
        os.makedirs(self.dir, exist_ok=True)
        f = os.path.join(self.dir, 'crs.wkt')
        if not os.path.exists(f):
            with open(f, 'w', encoding='utf-8') as fh: fh.write(crs.to_wkt())

    def path(self, e, n): return os.path.join(self.dir, f'{int(e)}_{int(n)}.npz')

    def _ok(self, meta, sq_lonlat):
        if meta.get('fp') != self.fp or meta.get('version') != PROC_VERSION: return False
        used = meta.get('pieces') or {}
        for name, (size, b) in self.pieces.items():                 # USGS: a changed or new tile under the square
            if not b or b[2] < sq_lonlat[0] or b[0] > sq_lonlat[2] or b[3] < sq_lonlat[1] or b[1] > sq_lonlat[3]: continue
            if name not in used or (size and used[name] and used[name] != size): return False
        return True

    def load(self, e, n, sq_lonlat, meta_only=False):
        import numpy as np
        f = self.path(e, n)
        if not os.path.exists(f): return None
        try:
            with np.load(f) as z:
                meta = json.loads(str(z['meta']))
                if not self._ok(meta, sq_lonlat): return None
                if meta_only: return True
                return {k: z[k] for k in ('dem', 'rock', 'dens', 'cov')}
        except Exception:
            return None

    def save(self, e, n, dem, rock, dens, cov, used):
        import numpy as np
        os.makedirs(self.dir, exist_ok=True)
        meta = json.dumps({'fp': self.fp, 'version': PROC_VERSION, 'pieces': used, 'made': dt.datetime.now().isoformat(timespec='seconds')})
        tmp = self.path(e, n) + '.tmp.npz'
        np.savez_compressed(tmp, dem=dem.astype(np.float32), rock=rock.astype(np.float16), dens=dens.astype(np.float16),
                            cov=cov.astype(bool), meta=np.array(meta))
        os.replace(tmp, self.path(e, n))


def _work_crs(src, lon, lat):
    from pyproj import CRS, Transformer
    if src.is_geographic or src.to_epsg() == 3857:          # Amazon EPT is Web Mercator: not true metres -> work in UTM
        crs_h = CRS.from_epsg((32600 if lat >= 0 else 32700) + int((lon + 180) // 6) + 1)
        return crs_h, Transformer.from_crs(src, crs_h, always_xy=True)
    return src, None


def _grid(crs_h, lon, lat, radius, cell_m):
    from pyproj import Transformer
    unit = crs_h.axis_info[0].unit_conversion_factor
    cx, cy = Transformer.from_crs(4326, crs_h, always_xy=True).transform(lon, lat)
    half = (radius + 30.0) / unit; cell = cell_m / unit
    x0 = math.floor((cx - half) / cell) * cell; y1 = math.ceil((cy + half) / cell) * cell      # snapped: squares line up
    nx, ny = int(math.ceil((cx + half - x0) / cell)), int(math.ceil((y1 - (cy - half)) / cell))
    return unit, cx, cy, x0, y1, cell, nx, ny


def _squares(x0, y1, nx, ny, cell):
    e0 = math.floor(x0 / SQ) * SQ; n0 = math.floor((y1 - ny * cell) / SQ) * SQ
    return [(e, n) for e in range(int(e0), int(x0 + nx * cell), SQ) for n in range(int(n0), int(y1), SQ)]


def _sq_lonlat(crs_h, e, n):
    from pyproj import Transformer
    lo, la = Transformer.from_crs(crs_h, 4326, always_xy=True).transform([e, e + SQ, e, e + SQ], [n, n, n + SQ, n + SQ])
    return (min(lo), min(la), max(lo), max(la))


def list_processed(cache_root):
    """Every processed square in the cache (current processing version only), as
    [(collection, [(lon, lat) x 4 corners]), ...] for drawing on a map."""
    from pyproj import CRS, Transformer
    out = []
    base = os.path.join(cache_root, '_processed')
    if not os.path.isdir(base): return out
    for coll in sorted(os.listdir(base)):
        d = os.path.join(base, coll, f'v{PROC_VERSION}'); f = os.path.join(d, 'crs.wkt')
        if not os.path.exists(f): continue
        try:
            crs = _read_wkt(f)
            tr = Transformer.from_crs(crs, 4326, always_xy=True)
        except Exception:
            continue
        for name in os.listdir(d):
            m = re.match(r'^(-?\d+)_(-?\d+)\.npz$', name)
            if not m: continue
            e, n = int(m.group(1)), int(m.group(2))
            lo, la = tr.transform([e, e + SQ, e + SQ, e], [n, n, n + SQ, n + SQ])
            out.append((coll, list(zip(lo, la))))
    return out


def map_processed(cache_root, box, out_dir, w_open=15.0, k=6.0, terrain=True, log=print, block=2000, name=None):
    """One sweep width + hidden ground map for every processed square inside box = (lon0, lat0, lon1, lat1),
    straight from the processed-ground cache (no lidar).  Worked in 2 km blocks with a halo so memory stays low."""
    import numpy as np
    from scipy import ndimage
    from pyproj import CRS, Transformer
    CANCEL.clear()
    base_dir = os.path.join(cache_root, '_processed')
    best = None
    for coll in sorted(os.listdir(base_dir)) if os.path.isdir(base_dir) else []:
        d = os.path.join(base_dir, coll, f'v{PROC_VERSION}'); f = os.path.join(d, 'crs.wkt')
        if not os.path.exists(f): continue
        crs = _read_wkt(f)
        tr = Transformer.from_crs(4326, crs, always_xy=True)
        xs, ys = tr.transform([box[0], box[2], box[0], box[2]], [box[1], box[1], box[3], box[3]])
        bx0, bx1, by0, by1 = min(xs), max(xs), min(ys), max(ys)
        sq = []
        for nm in os.listdir(d):
            m = re.match(r'^(-?\d+)_(-?\d+)\.npz$', nm)
            if not m: continue
            e, n = int(m.group(1)), int(m.group(2))
            if e + SQ > bx0 and e < bx1 and n + SQ > by0 and n < by1: sq.append((e, n))
        if sq and (best is None or len(sq) > len(best[3])): best = (coll, d, crs, sq, (bx0, bx1, by0, by1))
    if not best: raise RuntimeError('No processed ground inside that box.  Run some areas first (they show in purple).')
    coll, d, crs, sq, (bx0, bx1, by0, by1) = best
    have = set(sq)
    x0 = max(min(e for e, n in sq), math.floor(bx0)); x1 = min(max(e for e, n in sq) + SQ, math.ceil(bx1))
    y0 = max(min(n for e, n in sq), math.floor(by0)); y1 = min(max(n for e, n in sq) + SQ, math.ceil(by1))
    nx, ny = int(x1 - x0), int(y1 - y0)
    if nx * ny > 250e6: raise RuntimeError(f'That box holds {nx * ny / 1e6:.0f} km2 of processed ground; draw a smaller one (up to 250 km2).')
    log(f'{coll}: {len(sq)} processed squares in the box; map {nx / 1000:.1f} x {ny / 1000:.1f} km at 1 m.')
    W = np.full((ny, nx), np.nan, np.float32); Wf = np.full((ny, nx), np.nan, np.float32)
    loaded = {}
    def get(e, n):
        if (e, n) not in have: return None
        if (e, n) not in loaded:
            if len(loaded) > 60: loaded.pop(next(iter(loaded)))
            try:
                with np.load(os.path.join(d, f'{e}_{n}.npz')) as z:
                    loaded[(e, n)] = {kk: z[kk] for kk in ('dem', 'rock', 'dens', 'cov')}
            except Exception:
                loaded[(e, n)] = None
        return loaded[(e, n)]
    blocks = [(bx, by) for by in range(int(y1), int(y0), -block) for bx in range(int(x0), int(x1), block)]
    solid = 0
    for bi, (bx, by) in enumerate(blocks, 1):
        _check(); _emit('stage', name='Mapping processed ground (2 km blocks)', i=bi - 1, n=len(blocks))
        ex0, ey1 = bx - HALO, by + HALO
        ex1, ey0 = min(bx + block, x1) + HALO, max(by - block, y0) - HALO
        enx, eny = int(ex1 - ex0), int(ey1 - ey0)
        dem = np.full((eny, enx), np.nan, np.float32); rock = np.zeros((eny, enx), np.float32)
        dens = np.zeros((eny, enx), np.float32); cov = np.zeros((eny, enx), bool)
        for e in range(int(math.floor(ex0 / SQ) * SQ), int(ex1), SQ):
            for n in range(int(math.floor(ey0 / SQ) * SQ), int(ey1), SQ):
                g = get(e, n)
                if g is None: continue
                c0, c1 = max(0, e - ex0), min(enx, e + SQ - ex0); r0, r1 = max(0, ey1 - (n + SQ)), min(eny, ey1 - n)
                if c1 <= c0 or r1 <= r0: continue
                c0, c1, r0, r1 = int(c0), int(c1), int(r0), int(r1)
                sc0 = int(ex0 + c0 - e); sr0 = int((n + SQ) - (ey1 - r0))
                win = (slice(sr0, sr0 + r1 - r0), slice(sc0, sc0 + c1 - c0))
                dem[r0:r1, c0:c1] = g['dem'][win]; rock[r0:r1, c0:c1] = g['rock'][win]
                dens[r0:r1, c0:c1] = g['dens'][win]; cov[r0:r1, c0:c1] = g['cov'][win]
        if np.isnan(dem).all(): continue
        if np.isnan(dem).any():                                          # gaps between squares: nearest ground, no coverage
            idx = ndimage.distance_transform_edt(np.isnan(dem), return_distances=False, return_indices=True)
            dem = dem[tuple(idx)]
        D = ndimage.gaussian_filter(dens, 5.0).astype(np.float32)
        if terrain:
            Wb, Wfb = terrain_sweep_width(dem + rock, D, 1.0, w_open, k)
        else:
            Dt, Wt = w_of_d_table(w_open, k); Wb = np.interp(D, Dt, Wt).astype(np.float32); Wfb = Wb
        # interior of the block (without the halo) -> output
        ic0, ir0 = int(bx - ex0), int(ey1 - by)
        iw, ih = int(min(bx + block, x1) - bx), int(by - max(by - block, y0))
        oc0, or0 = int(bx - x0), int(y1 - by)
        sl = (slice(ir0, ir0 + ih), slice(ic0, ic0 + iw)); so = (slice(or0, or0 + ih), slice(oc0, oc0 + iw))
        m = cov[sl]
        W[so] = np.where(m, Wb[sl], np.nan); Wf[so] = np.where(m, Wfb[sl], np.nan)
        solid += int(((rock[sl] > 0) & m).sum())
    _emit('stage', name='Writing maps', i=0, n=2)
    os.makedirs(out_dir, exist_ok=True)
    name = name or f'SARPOD_processed_{dt.datetime.now():%Y%m%d_%H%M}'
    base = os.path.join(out_dir, name)
    shares, med, prev_w = write_outputs(W, (x0, y1, 1.0), crs, base)
    obst = write_obstruction(W, (x0, y1, 1.0), crs, base, w_open)
    _emit('stage', name='Writing maps', i=1, n=2)
    hid, prev_h = write_hidden(W, Wf, (x0, y1, 1.0), crs, base) if terrain else ({}, None)
    _emit('stage', name='Writing maps', i=2, n=2)
    v = ~np.isnan(W)
    lines = [f'Sweep width map of all processed ground in the box ({nx / 1000:.1f} x {ny / 1000:.1f} km, 1 m cells)',
             f'Made {dt.datetime.now():%Y-%m-%d %H:%M} from {len(sq)} processed 500 m squares of {coll} ({float(v.sum()) / 1e6:.1f} km2 with lidar).',
             f'Open-woods sweep width {w_open:g} m and understory factor K {k:g} are PLACEHOLDERS (not yet calibrated).',
             f'Median local sweep width {med:.1f} m (placeholder units); median obstruction index {obst:.0f} of 10.'] + [f'  {kk:22} {val:5.1f}% of ground' for kk, val in shares.items()] + (
             [f'Terrain: included; cuts sweep width by 10%+ on {100 * float(np.nanmean((W / Wf)[v] < 0.90)):.1f}% and 25%+ on '
              f'{100 * float(np.nanmean((W / Wf)[v] < 0.75)):.1f}% of the ground.  Solid objects: {solid:,.0f} m2.']
             + [f'  {kk:22} {val:5.1f}% of ground' for kk, val in hid.items()] if terrain else ['Terrain: not included.']) + [
             'Upload *_sweepwidth_webmercator.tif and *_hidden_ground_webmercator.tif to CalTopo as map sheets; they place themselves.']
    with open(base + '_sweepwidth_summary.txt', 'w', encoding='utf-8') as fh: fh.write('\n'.join(lines) + '\n')
    sizes = {kk: os.path.getsize(base + suf) / 1e6 for kk, suf in (('sweep', '_sweepwidth_webmercator.tif'), ('hidden', '_hidden_ground_webmercator.tif'))
             if os.path.exists(base + suf)}
    lines.append('Map sheet sizes: ' + ', '.join(f'{kk} {mb:.1f} MB' for kk, mb in sizes.items()) + ' (CalTopo limit 40 MB).')
    return {'lines': lines, 'base': base, 'out_dir': out_dir, 'minutes': 0, 'preview_sweep': prev_w, 'preview_hidden': prev_h}


def process_box(cache_root, box, source='auto', prefer=None, workers=16, log=print, chunk=2000, log_file=None):
    """Download and process every 500 m square inside box = (lon0, lat0, lon1, lat1), 2 km at a time, saving the
    squares as it goes.  Chunks already processed are skipped, so a stopped run carries on where it left off."""
    import numpy as np
    from pyproj import CRS, Transformer
    CANCEL.clear()
    if log_file:                                             # keep a copy of everything for the morning
        _log0 = log
        def log(m, _f=log_file):
            _log0(m)
            try:
                with open(_f, 'a', encoding='utf-8') as fh: fh.write(f'{dt.datetime.now():%H:%M:%S}  {m}\n')
            except OSError: pass
    lon_c, lat_c = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    utm = CRS.from_epsg((32600 if lat_c >= 0 else 32700) + int((lon_c + 180) // 6) + 1)
    fw = Transformer.from_crs(4326, utm, always_xy=True); bk = Transformer.from_crs(utm, 4326, always_xy=True)
    xs, ys = fw.transform([box[0], box[2], box[0], box[2]], [box[1], box[1], box[3], box[3]])
    X0 = math.floor(min(xs) / SQ) * SQ; X1 = math.ceil(max(xs) / SQ) * SQ
    Y0 = math.floor(min(ys) / SQ) * SQ; Y1 = math.ceil(max(ys) / SQ) * SQ
    chunks = [(cx, cy) for cy in range(int(Y1), int(Y0), -chunk) for cx in range(int(X0), int(X1), chunk)]
    area = (X1 - X0) * (Y1 - Y0) / 1e6
    log(f'Box: {(X1 - X0) / 1000:.1f} x {(Y1 - Y0) / 1000:.1f} km ({area:.0f} km2, {int(area * 4)} squares), {len(chunks)} chunks of {chunk / 1000:g} km.')
    done = skipped = failed = 0; t0 = time.time(); colls = set(); retry = []
    def one(ci, cx, cy):
        nonlocal done, skipped, failed
        cx1, cy0 = min(cx + chunk, X1), max(cy - chunk, Y0)
        _emit('chunk', i=ci, n=len(chunks), done=done, skipped=skipped, failed=failed, elapsed=time.time() - t0)
        lo, la = bk.transform([cx - HALO - 60, cx1 + HALO + 60, cx - HALO - 60, cx1 + HALO + 60],
                              [cy0 - HALO - 60, cy0 - HALO - 60, cy + HALO + 60, cy + HALO + 60])
        fetch_box = (min(lo), min(la), max(lo), max(la))
        lo2, la2 = bk.transform([cx, cx1, cx, cx1], [cy0, cy0, cy, cy])
        cover_box = (min(lo2), min(la2), max(lo2), max(la2))
        log(f'--- chunk {ci} of {len(chunks)}')
        try:
            plan = plan_lidar(fetch_box, cache_root, source, prefer, log=lambda m: None, cover_bbox=cover_box, min_cover=0.02)
        except Exception as e:
            log(f'  no lidar here ({e}); skipped.'); failed += 1; return
        sc = SquareCache(cache_root, plan); crs_h = sc.crs()
        def chunk_squares(crs):
            tr = Transformer.from_crs(utm, crs, always_xy=True)
            qx, qy = tr.transform([cx, cx1, cx, cx1], [cy0, cy0, cy, cy])
            a0, a1 = math.floor(min(qx) / SQ) * SQ, math.ceil(max(qx) / SQ) * SQ
            b0, b1 = math.floor(min(qy) / SQ) * SQ, math.ceil(max(qy) / SQ) * SQ
            # squares whose centre falls in this chunk (so neighbouring chunks don't both claim one)
            out = []
            for e in range(int(a0), int(a1), SQ):
                for n in range(int(b0), int(b1), SQ):
                    ux, uy = Transformer.from_crs(crs, utm, always_xy=True).transform(e + SQ / 2, n + SQ / 2)
                    if cx <= ux < cx1 and cy0 <= uy < cy: out.append((e, n))
            return out
        if crs_h is not None:
            sqs = chunk_squares(crs_h)
            missing = [q for q in sqs if not sc.load(q[0], q[1], _sq_lonlat(crs_h, *q), meta_only=True)]
            if not missing:
                if sqs: log(f'  already processed ({len(sqs)} squares); skipped.'); skipped += 1; colls.add(plan['name'])
                return
        log(f"  {plan['source']}: {plan['name']}, {len(plan['items'])} pieces")
        paths = fetch_lidar(plan, workers, log)
        crs0 = CRS.from_epsg(plan['epsg']) if plan['epsg'] else next(_read_chunks(paths[0]))[0].parse_crs()
        src = crs0.sub_crs_list[0] if crs0.is_compound else crs0
        crs_w, to_work = _work_crs(src, lon_c, lat_c)
        if crs_h is not None and not crs_h.equals(crs_w): crs_w = crs_h
        sc.set_crs(crs_w)
        if crs_w.axis_info[0].unit_conversion_factor != 1:
            log('  this lidar is in feet; processed squares need metres, skipped.'); failed += 1; return
        sqs = chunk_squares(crs_w)
        missing = [q for q in sqs if not sc.load(q[0], q[1], _sq_lonlat(crs_w, *q), meta_only=True)]
        if not missing: skipped += 1; return
        bx0 = min(e for e, n in missing) - HALO; bx1 = max(e for e, n in missing) + SQ + HALO
        by0 = min(n for e, n in missing) - HALO; by1 = max(n for e, n in missing) + SQ + HALO
        bnx, bny = int(bx1 - bx0), int(by1 - by0)
        dem, rock, dens, cov, total = _layers(paths, src, crs_w, to_work, 1.0, bx0, by1, bnx, bny, 1.0, 1.0, log)
        used = {name: size for name, (size, b) in sc.pieces.items()}
        for e, n in missing:
            c0, r0 = int(e - bx0), int(by1 - (n + SQ)); win = (slice(r0, r0 + SQ), slice(c0, c0 + SQ))
            sc.save(e, n, dem[win], rock[win], dens[win], cov[win], used)
        done += 1; colls.add(plan['name'])
        el = time.time() - t0; left = (len(chunks) - ci) * el / ci
        log(f'  saved {len(missing)} squares ({total:,} points).  {el / 60:.0f} min so far, about {left / 60:.0f} min to go.')
        del dem, rock, dens, cov
    for ci, (cx, cy) in enumerate(chunks, 1):
        _check()
        try:
            one(ci, cx, cy)
        except Cancelled:
            raise
        except Exception as e:
            log(f'  chunk {ci} failed ({e}); will try again at the end.'); retry.append((ci, cx, cy))
    if retry:
        log(f'Trying the {len(retry)} failed chunk(s) once more.')
        for ci, cx, cy in retry:
            _check()
            try:
                one(ci, cx, cy)
            except Cancelled:
                raise
            except Exception as e:
                log(f'  chunk {ci} failed again ({e}).'); failed += 1
    _emit('chunk', i=len(chunks), n=len(chunks), done=done, skipped=skipped, failed=failed, elapsed=time.time() - t0)
    log(f'Box finished: {done} chunks processed, {skipped} already done, {failed} without usable lidar, {(time.time() - t0) / 60:.0f} min.')
    if len(colls) > 1: log(f'  Note: more than one lidar collection was used ({", ".join(sorted(colls))}); the box map uses the one with the most squares.')
    return done, skipped, failed


def squares_ready(plan, cache_root, lon, lat, radius, cell_m=None):
    """True if every processed square this circle needs is already cached (then no lidar has to be downloaded)."""
    cell_m = cell_m or (1.0 if radius <= 2500 else 2.0)
    if cell_m != 1.0: return False
    sc = SquareCache(cache_root, plan); crs_h = sc.crs()
    if crs_h is None: return False
    unit, cx, cy, x0, y1, cell, nx, ny = _grid(crs_h, lon, lat, radius, cell_m)
    if unit != 1: return False
    return all(sc.load(e, n, _sq_lonlat(crs_h, e, n), meta_only=True) for e, n in _squares(x0, y1, nx, ny, cell))


def _layers(tiles, src, crs_h, to_work, unit, x0, y1, nx, ny, cell, cell_m, log):
    """Read the lidar for one box: bare earth, rock heights, brush density, coverage."""
    import numpy as np
    from scipy import ndimage
    if to_work:                                              # the same box in the source CRS, for skipping files
        from pyproj import Transformer
        back = Transformer.from_crs(crs_h, src, always_xy=True)
        xa, xb, ya, yb = x0, x0 + nx * cell, y1 - ny * cell, y1; xm, ym = (xa + xb) / 2, (ya + yb) / 2
        bx, by = back.transform([xa, xb, xa, xb, xm, xm, xa, xb], [ya, ya, yb, yb, ya, yb, ym, ym])
        sx0, sx1, sy0, sy1 = min(bx), max(bx), min(by), max(by)
    else:
        sx0, sx1, sy0, sy1 = x0, x0 + nx * cell, y1 - ny * cell, y1

    def xy(ch, m=None):
        X, Y = np.asarray(ch.x), np.asarray(ch.y)
        if m is not None: X, Y = X[m], Y[m]
        return to_work.transform(X, Y) if to_work else (X, Y)

    def cells(x, y):
        c = ((x - x0) / cell).astype(np.int64); r = ((y1 - y) / cell).astype(np.int64)
        ok = (c >= 0) & (c < nx) & (r >= 0) & (r < ny)
        return r, c, ok

    def overlaps(h):
        return not (h.maxs[0] < sx0 or h.mins[0] > sx1 or h.maxs[1] < sy0 or h.mins[1] > sy1)

    zs = np.zeros(ny * nx); n = np.zeros(ny * nx); total = 0
    step = max(1, len(tiles) // 40); tlast = 0
    for i, ch in _points(tiles, overlaps):
        if ch is None:
            if len(tiles) < 60 or i % (step * 4) == 0 or i == len(tiles): log(f'  ground {i}/{len(tiles)}')
            if time.time() - tlast > 0.3 or i == len(tiles):
                _emit('stage', name='Reading ground', i=i, n=len(tiles)); tlast = time.time()
            continue
        cls = np.asarray(ch.classification); g = cls == 2
        Xs, Ys = np.asarray(ch.x), np.asarray(ch.y)
        inb = (Xs >= sx0) & (Xs <= sx1) & (Ys >= sy0) & (Ys <= sy1); total += int(inb.sum()); g &= inb
        r, c, ok = cells(*xy(ch, g))
        li = r[ok] * nx + c[ok]
        _addcount(zs, li, np.asarray(ch.z)[g][ok]); _addcount(n, li)
        _check()
    zs = zs.reshape(ny, nx); n = n.reshape(ny, nx)
    if not (n > 0).any(): raise RuntimeError('No ground points inside the circle.')
    dem = np.where(n > 0, zs / np.maximum(n, 1), np.nan); del zs
    covered = ndimage.binary_dilation(n > 0, iterations=max(1, int(5 / cell_m)))
    idx = ndimage.distance_transform_edt(np.isnan(dem), return_distances=False, return_indices=True)
    dem = ndimage.gaussian_filter(dem[tuple(idx)], 0.7); del idx, n
    und = np.zeros(ny * nx); below = np.zeros(ny * nx)
    low = np.zeros(ny * nx); upto = np.zeros(ny * nx); minh = np.full(ny * nx, np.inf)
    tlast = 0
    for i, ch in _points(tiles, overlaps):
        if ch is None:
            if len(tiles) < 60 or i % (step * 4) == 0 or i == len(tiles): log(f'  understory {i}/{len(tiles)}')
            if time.time() - tlast > 0.3 or i == len(tiles):
                _emit('stage', name='Reading brush and rock', i=i, n=len(tiles)); tlast = time.time()
            continue
        cls = np.asarray(ch.classification); z = np.asarray(ch.z)
        if to_work:                                      # drop points outside the box before reprojecting
            Xs, Ys = np.asarray(ch.x), np.asarray(ch.y)
            inb = (Xs >= sx0) & (Xs <= sx1) & (Ys >= sy0) & (Ys <= sy1)
            r, c, ok = cells(*to_work.transform(Xs[inb], Ys[inb])); z, cls = z[inb], cls[inb]
        else:
            r, c, ok = cells(np.asarray(ch.x), np.asarray(ch.y))
        r, c, z, cls = r[ok], c[ok], z[ok], cls[ok]
        hag = (z - dem[r, c]) * unit
        keep = (cls != 7) & (cls != 18); li = r * nx + c
        m = keep & (hag > 0.5) & (hag < 3.0); _addcount(und, li[m])
        mb = keep & (hag < 3.0); _addcount(below, li[mb])
        ml = keep & (hag < BOULDER_MIN_H); _addcount(low, li[ml])           # boulder detection (always on, so the
        mu = keep & (hag < BOULDER_MAX_H); _addcount(upto, li[mu])          # brush count is the same with or
        mh = mu & ~ml; np.minimum.at(minh, li[mh], hag[mh])                 # without terrain, and squares can be shared)
        _check()
    und = und.reshape(ny, nx); below = below.reshape(ny, nx)
    rock, solid = find_boulders(low.reshape(ny, nx), upto.reshape(ny, nx), minh.reshape(ny, nx), cell_m)
    und[solid] = 0                                         # terrain handles them; don't count twice as brush
    del low, upto, minh
    w3 = max(1, int(round(3 / cell_m)))
    dens = np.clip(ndimage.uniform_filter(und, w3) / np.maximum(ndimage.uniform_filter(below, w3), 1e-6), 0, 1)
    return (dem * unit).astype(np.float32), rock, dens.astype(np.float32), covered, total


def _build_sweep_width(tiles, lon, lat, radius, w_open, k, cell_m, log=print, terrain=True, src_epsg=None, sqcache=None):
    import numpy as np
    from scipy import ndimage
    from pyproj import CRS
    src = None; crs_h = sqcache.crs() if sqcache else None
    if tiles:
        crs = CRS.from_epsg(src_epsg) if src_epsg else next(_read_chunks(tiles[0]))[0].parse_crs()
        if crs is None: raise RuntimeError('Lidar tile has no coordinate system.')
        src = crs.sub_crs_list[0] if crs.is_compound else crs
        crs_w, to_work = _work_crs(src, lon, lat)
        if crs_h is None or not crs_h.equals(crs_w): crs_h = crs_w
    else:
        to_work = None
        if crs_h is None: raise RuntimeError('No lidar files and no processed squares for this area.')
    unit, cx, cy, x0, y1, cell, nx, ny = _grid(crs_h, lon, lat, radius, cell_m)
    log(f'Grid {nx} x {ny} cells of {cell_m:g} m in {crs_h.name}')
    use_sq = sqcache is not None and cell_m == 1.0 and unit == 1
    dem = np.full((ny, nx), np.nan, np.float32); rock = np.zeros((ny, nx), np.float32)
    dens = np.zeros((ny, nx), np.float32); covered = np.zeros((ny, nx), bool); have = np.zeros((ny, nx), bool)
    total = 0; from_cache = 0
    sqs = _squares(x0, y1, nx, ny, cell) if use_sq else []

    def window(e, n, xa, ya, gx0, gy1, gnx, gny):
        """Overlap of square (e, n) with a grid -> (grid rows, grid cols, square rows, square cols) or None."""
        c0 = max(0, int(round((e - gx0) / cell))); c1 = min(gnx, int(round((e + SQ - gx0) / cell)))
        r0 = max(0, int(round((gy1 - (n + SQ)) / cell))); r1 = min(gny, int(round((gy1 - n) / cell)))
        if c1 <= c0 or r1 <= r0: return None
        sc0 = int(round((gx0 + c0 * cell - e) / cell)); sr0 = int(round(((n + SQ) - (gy1 - r0 * cell)) / cell))
        return slice(r0, r1), slice(c0, c1), slice(sr0, sr0 + r1 - r0), slice(sc0, sc0 + c1 - c0)

    missing = []
    for j, (e, n) in enumerate(sqs):
        got = sqcache.load(e, n, _sq_lonlat(crs_h, e, n))
        if got is None: missing.append((e, n)); continue
        w = window(e, n, None, None, x0, y1, nx, ny)
        if not w: continue
        gr, gc, sr, sc = w
        dem[gr, gc] = got['dem'][sr, sc]; rock[gr, gc] = got['rock'][sr, sc]; dens[gr, gc] = got['dens'][sr, sc]
        covered[gr, gc] = got['cov'][sr, sc]; have[gr, gc] = True; from_cache += 1
        if j % 10 == 0: _emit('stage', name='Loading processed squares', i=j + 1, n=len(sqs))
    if use_sq:
        log(f'  processed squares: {from_cache} of {len(sqs)} from the cache')
    if not use_sq or missing:
        if not tiles: raise RuntimeError('Some processed squares are missing and no lidar files were given.')
        if use_sq:                                     # read lidar for whole missing squares + halo (may reach past the grid)
            bx0 = min(e for e, n in missing) - HALO; bx1 = max(e for e, n in missing) + SQ + HALO
            by0 = min(n for e, n in missing) - HALO; by1 = max(n for e, n in missing) + SQ + HALO
        else:
            bx0, bx1, by0, by1 = x0, x0 + nx * cell, y1 - ny * cell, y1
        bnx, bny = int(round((bx1 - bx0) / cell)), int(round((by1 - by0) / cell))
        bdem, brock, bdens, bcov, total = _layers(tiles, src, crs_h, to_work, unit, bx0, by1, bnx, bny, cell, cell_m, log)
        # paste the read box into the grid where the grid has nothing from the cache
        gc0 = int(round((bx0 - x0) / cell)); gr0 = int(round((y1 - by1) / cell))
        c0, c1 = max(0, gc0), min(nx, gc0 + bnx); r0, r1 = max(0, gr0), min(ny, gr0 + bny)
        if c1 > c0 and r1 > r0:
            bs = (slice(r0 - gr0, r1 - gr0), slice(c0 - gc0, c1 - gc0))
            keep = ~have[r0:r1, c0:c1]
            for a, b in ((dem, bdem), (rock, brock), (dens, bdens), (covered, bcov)):
                a[r0:r1, c0:c1][keep] = b[bs][keep]
        if use_sq:
            sqcache.set_crs(crs_h)
            used = {name: size for name, (size, b) in sqcache.pieces.items()}
            saved = 0
            for e, n in missing:
                sc0 = int(round((e - bx0) / cell)); sr0 = int(round((by1 - (n + SQ)) / cell)); m = int(round(SQ / cell))
                if sc0 < 0 or sr0 < 0 or sc0 + m > bnx or sr0 + m > bny: continue
                win = (slice(sr0, sr0 + m), slice(sc0, sc0 + m))
                sqcache.save(e, n, bdem[win], brock[win], bdens[win], bcov[win], used); saved += 1
            log(f'  saved {saved} new processed squares')
        del bdem, brock, bdens, bcov
    nsolid = int((rock > 0).sum())
    log(f'  solid objects (boulders, ledges, buildings): {nsolid * cell_m * cell_m:,.0f} m2')
    D = ndimage.gaussian_filter(dens, 5.0 / cell_m).astype(np.float32)
    if terrain:
        log('  terrain line of sight (24 directions)')
        W, Wflat = terrain_sweep_width(dem + rock, D, cell_m, w_open, k)
    else:
        Dt, Wt = w_of_d_table(w_open, k)
        W = np.interp(D, Dt, Wt).astype(np.float32); Wflat = W
    yy, xx = np.mgrid[0:ny, 0:nx]
    inside = np.hypot(x0 + (xx + .5) * cell - cx, y1 - (yy + .5) * cell - cy) * unit <= radius
    W[~(covered & inside)] = np.nan
    Wflat = np.where(covered & inside, Wflat, np.nan)
    return W, (x0, y1, cell), crs_h, total, Wflat, nsolid * cell_m * cell_m, from_cache, len(sqs)


BOULDER_MIN_H, BOULDER_MAX_H = 1.5, 12.0


def find_boulders(low, upto, minh, cell_m):
    """Boulders, rock ledges and buildings that the ground classification removed.  A laser gets down through
    leaf-off trees (some returns reach the ground in any few square metres), but not through rock: over a boulder
    the LOWEST return is its top.  A cell counts as solid if, over the 3 x 3 m around it, there are returns but
    none (ground or otherwise) below 1.5 m, and the lowest one is 1.5-12 m up.  Patches under 3 m2 are dropped
    (single dense shrubs / data gaps).  Returns (height added to the bare earth in metres, solid mask)."""
    import numpy as np
    from scipy import ndimage
    w = max(3, int(round(3 / cell_m)) | 1)
    low_n = ndimage.uniform_filter(low, w) * w * w
    all_n = ndimage.uniform_filter(upto, w) * w * w
    h = np.where(np.isfinite(minh), minh, np.nan)
    solid = (low_n < 0.5) & (all_n >= 4) & np.isfinite(h)
    lab, n = ndimage.label(solid)
    if n:
        size = ndimage.sum(np.ones_like(lab), lab, np.arange(1, n + 1)) * cell_m * cell_m
        solid &= np.isin(lab, np.flatnonzero(size >= 3) + 1)
    # the 3 x 3 test trims the edges; grow back into neighbouring cells whose own returns all sit 1.5 m+ up
    own = (low < 0.5) & np.isfinite(h)
    for _ in range(max(1, int(round(2 / cell_m)))):
        solid |= ndimage.binary_dilation(solid) & own
    # height: the cell's own lowest return, filled from neighbours for solid cells with no return of their own
    hh = ndimage.grey_erosion(np.where(np.isfinite(h), h, np.inf), size=3)
    hh = np.where(np.isfinite(h), h, hh)
    rock = np.where(solid & np.isfinite(hh), hh, 0).astype(np.float32)
    return rock, solid


def terrain_sweep_width(dem, D, cell_m, w_open, k, p0=0.8, eye=1.6, target=0.3, ndir=24):
    """Local sweep width with terrain: searchers standing at distance d (eye 1.6 m) in 24 directions around a spot
    look at a subject on the ground there (0.3 m).  Each look counts if the bare earth (incl. ground-classified
    boulders) doesn't block the straight sight line, weighted by the detection curve and thinned by understory:
        W = W_open * sum_dir sum_d w(d) * V_terrain(dir, d) * exp(-K D d / 10) / (ndir * sum_d w(d))
    With flat ground (V = 1) this is exactly the no-terrain table, so terrain can only lower W.
    dem in metres.  Returns (W with terrain, W without)."""
    import numpy as np
    s = w_open / (2.4755 * p0); dmax = 2.5 * s
    steps = np.arange(1, int(dmax / cell_m) + 1); dist = steps * cell_m
    wd = p0 * np.exp(-dist ** 2 / (2 * s * s)) * dist; wsum = wd.sum() * ndir
    R = int(steps[-1]) + 1; ny, nx = dem.shape
    zp = np.pad(dem.astype(np.float32), R, mode='edge')
    zt = dem.astype(np.float32) + target
    def at(dr, dc): return zp[R + dr:R + dr + ny, R + dc:R + dc + nx]
    acc = np.zeros((ny, nx), np.float32); flat = np.zeros((ny, nx), np.float32)
    for j, a in enumerate(np.arange(ndir) * 2 * np.pi / ndir):
        _check(); _emit('stage', name='Terrain sight lines', i=j + 1, n=ndir)
        ca, sa = np.cos(a), np.sin(a)
        M = np.full((ny, nx), -np.inf, np.float32)           # steepest slope from the subject to any ground before d
        for st, d, w in zip(steps, dist, wd):
            z = at(int(round(-st * sa)), int(round(st * ca)))
            att = w * np.exp(-k * D * d / 10)
            vis = (z + eye - zt) / d >= M
            acc += att * vis; flat += att
            np.maximum(M, (z - zt) / d, out=M)
    return (w_open * acc / wsum).astype(np.float32), (w_open * flat / wsum).astype(np.float32)


# ---------------------------------------------------------------- outputs
HIDDEN = [(0.75, 0.90, (0xFD, 0xAE, 0x61), 150, 'Terrain cuts 10-25%'), (0.50, 0.75, (0xF4, 0x6D, 0x43), 190, 'Terrain cuts 25-50%'),
          (0.0, 0.50, (0xA5, 0x00, 0x26), 220, 'Terrain cuts 50%+')]


def write_rgba_3857(rgba, tf, src_crs, path):
    import numpy as np, rasterio
    from rasterio.warp import calculate_default_transform, reproject, Resampling
    ny, nx = rgba.shape[1:]
    left, top = tf.c, tf.f; right, bottom = left + nx * tf.a, top + ny * tf.e
    t2, w2, h2 = calculate_default_transform(src_crs, 'EPSG:3857', nx, ny, left, bottom, right, top)
    out = np.zeros((4, h2, w2), np.uint8)
    for i in range(4):
        reproject(rgba[i], out[i], src_transform=tf, src_crs=src_crs, dst_transform=t2, dst_crs='EPSG:3857', resampling=Resampling.nearest)
    with rasterio.open(path, 'w', driver='GTiff', width=w2, height=h2, count=4, dtype='uint8',
                       crs='EPSG:3857', transform=t2, compress='deflate', photometric='RGB', alpha='YES') as d:
        d.write(out)
    return out, (t2.c, t2.f + h2 * t2.e, t2.c + w2 * t2.a, t2.f)        # RGBA + (xmin, ymin, xmax, ymax) in 3857


def write_hidden(W, Wflat, geo, crs_h, base):
    """Hidden-ground layer: where terrain alone (ledges, knobs, gullies, boulders) cuts the sweep width."""
    import numpy as np, rasterio
    from rasterio.transform import from_origin
    x0, y1, cell = geo; ny, nx = W.shape
    r = np.where(np.isnan(W) | np.isnan(Wflat) | (Wflat <= 0), 1.0, W / np.where(Wflat > 0, Wflat, 1))
    rgba = np.zeros((4, ny, nx), np.uint8)
    for lo, hi, rgb, a, _ in HIDDEN:
        m = (r >= lo) & (r < hi)
        for i in range(3): rgba[i][m] = rgb[i]
        rgba[3][m] = a
    prev = write_rgba_3857(rgba, from_origin(x0, y1, cell, cell), rasterio.crs.CRS.from_wkt(crs_h.to_wkt()), base + '_hidden_ground_webmercator.tif')
    v = r[~np.isnan(W)]
    return {c[4]: 100 * float(((v >= c[0]) & (v < c[1])).mean()) for c in HIDDEN}, prev


def write_obstruction(W, geo, crs_h, base, w_open):
    """0-10 obstruction index per cell: 0 = wide open (keeps all of open-woods visibility), 10 = a wall.
    = round(10 x (1 - W / W_open)).  Depends on K and terrain, not on the W_open value itself."""
    import numpy as np, rasterio
    from rasterio.transform import from_origin
    x0, y1, cell = geo; ny, nx = W.shape
    idx = np.where(np.isnan(W), 255, np.clip(np.rint(10 * (1 - W / w_open)), 0, 10)).astype(np.uint8)
    with rasterio.open(base + '_obstruction_0to10.tif', 'w', driver='GTiff', width=nx, height=ny, count=1, dtype='uint8',
                       crs=rasterio.crs.CRS.from_wkt(crs_h.to_wkt()), transform=from_origin(x0, y1, cell, cell),
                       nodata=255, compress='deflate') as d:
        d.write(idx, 1)
    v = idx[idx != 255]
    return float(np.median(v)) if v.size else float('nan')


def write_outputs(W, geo, crs_h, base, log=print):
    import numpy as np, rasterio
    from rasterio.transform import from_origin
    from rasterio.warp import calculate_default_transform, reproject, Resampling
    x0, y1, cell = geo; ny, nx = W.shape
    tf = from_origin(x0, y1, cell, cell)
    src_crs = rasterio.crs.CRS.from_wkt(crs_h.to_wkt())
    with rasterio.open(base + '_sweepwidth_DATA_for_QGIS.tif', 'w', driver='GTiff', width=nx, height=ny, count=1, dtype='float32',
                       crs=src_crs, transform=tf, nodata=float('nan'), compress='deflate', predictor=3) as d:
        d.write(W, 1)
    rgba = np.zeros((4, ny, nx), np.uint8); valid = ~np.isnan(W)
    for lo, hi, rgb, a, _ in CLASSES:
        m = valid & (W >= lo) & (W < hi)
        for i in range(3): rgba[i][m] = rgb[i]
        rgba[3][m] = a
    left, bottom, right, top = x0, y1 - ny * cell, x0 + nx * cell, y1
    t2, w2, h2 = calculate_default_transform(src_crs, 'EPSG:3857', nx, ny, left, bottom, right, top)
    out = np.zeros((4, h2, w2), np.uint8)
    for i in range(4):
        reproject(rgba[i], out[i], src_transform=tf, src_crs=src_crs, dst_transform=t2, dst_crs='EPSG:3857', resampling=Resampling.nearest)
    with rasterio.open(base + '_sweepwidth_webmercator.tif', 'w', driver='GTiff', width=w2, height=h2, count=4, dtype='uint8',
                       crs='EPSG:3857', transform=t2, compress='deflate', photometric='RGB', alpha='YES') as d:
        d.write(out)
    v = W[valid]
    return ({c[4]: 100 * float(((v >= c[0]) & (v < c[1])).mean()) for c in CLASSES}, float(np.median(v)),
            (out, (t2.c, t2.f + h2 * t2.e, t2.c + w2 * t2.a, t2.f)))


def run_job(export, title, lon, lat, radius, plan, cache, w_open=15.0, k=6.0, cell=None, terrain=True, workers=16,
            out_dir=None, log=print, reuse=True):
    """Download what the plan needs, build the maps, write the files.  Used by the command line and the window."""
    import numpy as np
    CANCEL.clear()
    cell = cell or (1.0 if radius <= 2500 else 2.0)
    sqc = SquareCache(cache, plan) if reuse else None
    if sqc and squares_ready(plan, cache, lon, lat, radius, cell):
        log('All of this ground is already processed; no lidar needed.')
        show_plan(plan); paths = []
    else:
        paths = fetch_lidar(plan, workers, log)
    t0 = time.time()
    W, geo, crs_h, npts, Wflat, solid_m2, nsq_cached, nsq = build_sweep_width(paths, lon, lat, radius, w_open, k, cell, log=log,
                                                             terrain=terrain, src_epsg=plan['epsg'], sqcache=sqc)
    _emit('stage', name='Writing maps', i=0, n=2)
    out_dir = out_dir or (os.path.dirname(os.path.abspath(export)) if export else os.path.join(os.path.expanduser('~'), 'Downloads'))
    os.makedirs(out_dir, exist_ok=True)
    stem = re.sub(r'\.(geo)?json$', '', os.path.basename(export), flags=re.I) if export else 'SARPOD'
    base = os.path.join(out_dir, f'{stem}_{re.sub(r"[^A-Za-z0-9]+", "_", title).strip("_")[:32]}_{radius:g}m')
    shares, med, prev_w = write_outputs(W, geo, crs_h, base)
    obst = write_obstruction(W, geo, crs_h, base, w_open)
    _emit('stage', name='Writing maps', i=1, n=2)
    hid, prev_h = write_hidden(W, Wflat, geo, crs_h, base) if terrain else ({}, None)
    _emit('stage', name='Writing maps', i=2, n=2)
    proj = plan['name']
    lines = [f'Sweep width map around "{title}" ({lat:.5f}, {lon:.5f}), radius {radius:g} m, {cell:g} m cells',
             f'Made {dt.datetime.now():%Y-%m-%d %H:%M} from lidar {proj} via {plan["source"]} ({len(paths)} files, {npts:,} points read)'
             + (f'; {nsq_cached} of {nsq} processed 500 m squares came from the cache' if nsq else '') + '.',
             f'Open-woods sweep width {w_open:g} m and understory factor K {k:g} are PLACEHOLDERS (not yet calibrated).',
             f'Median local sweep width {med:.1f} m (placeholder units); median obstruction index {obst:.0f} of 10.'] + [f'  {kk:22} {v:5.1f}% of ground' for kk, v in shares.items()] + (
             [f'Terrain: included.  Without it the median would be {float(np.nanmedian(Wflat)):.1f} m; terrain cuts sweep width by'
              f' 10%+ on {100 * float(np.nanmean((W / Wflat)[~np.isnan(W)] < 0.90)):.1f}% and 25%+ on {100 * float(np.nanmean((W / Wflat)[~np.isnan(W)] < 0.75)):.1f}% of the ground'
              f' (knolls, ledges, gullies, boulders).',
              f'Solid objects found above the bare earth (boulders, rock ledges, buildings): {solid_m2:,.0f} m2.',
              'Hidden ground layer (*_hidden_ground_webmercator.tif, terrain effect only; ledge bases and lips, behind knobs and boulders):']
             + [f'  {kk:22} {v:5.1f}% of ground' for kk, v in hid.items()]
             if terrain else ['Terrain: not included.']) + [
             'Upload *_sweepwidth_webmercator.tif (and *_hidden_ground_webmercator.tif) to CalTopo as map sheets (Add Layer > Map Sheet); they place themselves.',
             '*_sweepwidth_DATA_for_QGIS.tif holds the continuous value in metres for QGIS (it is not a picture).  *_obstruction_0to10.tif: 0 = wide open, 10 = a wall.']
    with open(base + '_sweepwidth_summary.txt', 'w', encoding='utf-8') as fh: fh.write('\n'.join(lines) + '\n')
    lines.insert(2, f'Processing took {(time.time() - t0) / 60:.1f} min.')
    return {'lines': lines, 'base': base, 'out_dir': out_dir, 'minutes': (time.time() - t0) / 60,
            'preview_sweep': prev_w, 'preview_hidden': prev_h}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('export', nargs='?', help='SARTopo / CalTopo map export (.json / .geojson).  Leave off to pick it in a window.')
    ap.add_argument('--at', help='center by coordinates instead of a marker, e.g. "39.61654, -79.82290" (no export needed)')
    ap.add_argument('--marker', help='marker title (partial match), e.g. IPP, LKP, PLS.  Leave off to pick from a list.')
    ap.add_argument('--radius', type=float, help='metres around the marker (e.g. 2000).  Asked for if left off.')
    ap.add_argument('--out', help='output folder (default: next to the export)')
    ap.add_argument('--cache', default=default_cache(), help=f'lidar tile cache folder (default {default_cache()})')
    ap.add_argument('--project', help='use the lidar project whose name contains this (default: best coverage, newest)')
    ap.add_argument('--w-open', type=float, default=15.0, help='sweep width in open woods, m (placeholder, default 15)')
    ap.add_argument('--k', type=float, default=6.0, help='how fast understory blocks sight (placeholder, default 6)')
    ap.add_argument('--cell', type=float, help='cell size, m (default 1 m up to 2.5 km radius, then 2 m)')
    ap.add_argument('--no-terrain', action='store_true', help='skip the terrain line-of-sight step (faster)')
    ap.add_argument('--workers', type=int, default=16, help='downloads at once (default 16; USGS limits each one to ~100 KB/s)')
    ap.add_argument('--source', choices=['auto', 'aws', 'usgs'], default='auto',
                    help='where to get lidar: auto (default: USGS tiles if already cached, else Amazon, else USGS), aws, usgs')
    ap.add_argument('--no-reuse', action='store_true', help="don't use or save processed 500 m squares (rebuild from lidar)")
    ap.add_argument('--set-cache', action='store_true', help='pick the lidar cache folder for this computer (e.g. in Google Drive) and stop')
    ap.add_argument('--list-only', action='store_true', help='just show the lidar available and stop')
    a = ap.parse_args()
    if a.set_cache:
        set_cache_folder(); input('\nPress Enter to close.'); return
    print(f'Lidar cache folder: {a.cache}')
    asked = not ((a.marker or a.at) and a.radius)
    if a.at:
        la_, lo_ = parse_latlon(a.at); a.export = a.export or ''
    if not a.export and not a.at:
        try:
            import tkinter, tkinter.filedialog
            print('A file picker is opening (it may be behind other windows): pick the SARTopo export.')
            root = tkinter.Tk(); root.withdraw(); root.attributes('-topmost', True)
            a.export = tkinter.filedialog.askopenfilename(parent=root, title='Pick the SARTopo export',
                                                          filetypes=[('SARTopo export', '*.json *.geojson'), ('All files', '*.*')])
        except Exception:
            pass
        while not a.export or not os.path.exists(a.export):
            a.export = input('Path to the SARTopo export (.json): ').strip().strip('"')
    if a.at:
        title, lon, lat = f'Point {la_:.5f} {lo_:.5f}', lo_, la_
    else:
        markers = read_markers(a.export)
        if not markers: sys.exit('No markers in this export.')
        title, lon, lat = pick_marker(markers, a.marker)
    if not a.radius:
        while True:
            v = input('Radius around the marker in metres (e.g. 2000): ').strip()
            try: a.radius = float(v); break
            except ValueError: pass
    cell = a.cell or (1.0 if a.radius <= 2500 else 2.0)
    print(f'\nMarker "{title}" at {lat:.5f}, {lon:.5f}; radius {a.radius:g} m; cell {cell:g} m')
    bbox = bbox_around(lon, lat, a.radius) if a.no_reuse else area_bbox(lon, lat, a.radius)
    try:
        plan = plan_lidar(bbox, a.cache, a.source, a.project, cover_bbox=bbox_around(lon, lat, a.radius, 0))
    except RuntimeError as e:
        sys.exit(str(e))
    if a.list_only: return
    job = run_job(a.export, title, lon, lat, a.radius, plan, a.cache, a.w_open, a.k, cell, not a.no_terrain, a.workers, a.out,
                  reuse=not a.no_reuse)
    print('\n' + '\n'.join(job['lines']))
    print(f"\nFiles: {job['base']}_*.tif")
    if asked: input('\nDone.  Press Enter to close.')


if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        raise
    except (DamagedTile, DownloadIncomplete, Cancelled) as e:
        print('\n' + str(e))
        try: input('Press Enter to close.')
        except EOFError: pass
    except Exception:
        import traceback; traceback.print_exc()
        print('\nSomething went wrong (details above).')
        try: input('Press Enter to close.')
        except EOFError: pass
