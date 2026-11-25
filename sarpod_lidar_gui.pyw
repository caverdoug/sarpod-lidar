#!/usr/bin/env python3
"""
SARPOD Lidar - understory visibility and line-of-sight map window.  EXPERIMENTAL; not validated.

Pick a SARTopo / CalTopo export, a marker and a radius.  The map shows the circle and every lidar piece the run
needs, coloured as it downloads (grey waiting, orange downloading, green done, teal already cached, red failed).
When the run finishes, the sweep width and hidden-ground layers are drawn on the map and written next to the
export as GeoTIFFs for CalTopo.

Background map: OpenTopoMap (CC-BY-SA) or OpenStreetMap, fetched only for the area on screen.
"""
import base64, math, os, queue, struct, sys, threading, time, traceback, urllib.request, zlib
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_sweep_map as eng

VERSION = '2026-11-23'
R = 20037508.342789244
BASEMAPS = {
    'Topo (OpenTopoMap)': ('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png', 'abc', 17, 'Map: (c) OpenStreetMap contributors, SRTM | (c) OpenTopoMap (CC-BY-SA)'),
    'Streets (OpenStreetMap)': ('https://tile.openstreetmap.org/{z}/{x}/{y}.png', '', 19, 'Map: (c) OpenStreetMap contributors'),
    'None': (None, '', 22, ''),
}
UA = f'SARPOD-lidar/{VERSION} (search and rescue planning tool)'
COLORS = {'pending': ('#8a8a8a', 'gray12'), 'busy': ('#ff8c00', 'gray50'), 'wait': ('#e6c200', 'gray50'),
          'done': ('#1a9850', ''), 'cached': ('#2c7fb8', ''), 'fail': ('#d7191c', 'gray50'),
          'partial': ('#7fbf3f', 'gray25')}   # '' = outline only; partial = some pieces done, some still waiting


def piece_style(state):
    col, st = COLORS[state]
    return dict(outline=col, fill=col if st else '', stipple=st)


# ------------------------------------------------------------------ small helpers
def keep_awake(on):
    """Stop Windows going to sleep while a run is going (the screen can still turn off)."""
    if sys.platform != 'win32': return
    try:
        import ctypes
        ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | (ES_SYSTEM_REQUIRED if on else 0))
    except Exception:
        pass


def png_bytes(rgba):
    """RGBA uint8 array (h, w, 4) -> PNG bytes (Tk 8.6 reads PNG without Pillow)."""
    import numpy as np
    h, w = rgba.shape[:2]
    raw = np.concatenate([np.zeros((h, 1), np.uint8), rgba.reshape(h, w * 4)], axis=1).tobytes()
    def chunk(t, d): return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(raw, 1)) + chunk(b'IEND', b''))


def lonlat_to_world(lon, lat, z):
    n = 256 * 2 ** z
    lat = max(min(lat, 85.0511), -85.0511)
    return (lon + 180) / 360 * n, (1 - math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi) / 2 * n


def merc_to_world(mx, my, z):
    n = 256 * 2 ** z
    return (mx + R) / (2 * R) * n, (R - my) / (2 * R) * n


def world_to_lonlat(x, y, z):
    n = 256 * 2 ** z
    lon = x / n * 360 - 180
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


# ------------------------------------------------------------------ map widget
class MapView(ttk.Frame):
    def __init__(self, master, **kw):
        super().__init__(master, **kw)
        self.canvas = tk.Canvas(self, bg='#e8e8e8', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.z, self.cx, self.cy = 7, *lonlat_to_world(-80.2, 38.9, 7)
        self.basemap = 'Topo (OpenTopoMap)'
        self.tiles, self.want, self.tq = {}, set(), queue.Queue()
        self.pool_sem = threading.Semaphore(6)
        self.marker = None; self.radius = 0
        self.pieces = {}; self.drawn = {}; self.piece_filter = None
        self.processed = []; self.processed_on = True
        self.box_mode = False; self.box = None; self._box_start = None; self.on_box = None
        self.overlays = {}; self.overlay_on = {}; self._ov_imgs = []
        self._drag = None; self._redraw_pending = False
        c = self.canvas
        c.bind('<Configure>', lambda e: self.redraw())
        c.bind('<ButtonPress-1>', self._press); c.bind('<B1-Motion>', self._move); c.bind('<ButtonRelease-1>', self._release)
        c.bind('<Double-Button-1>', self._dbl); self.on_place = None
        c.bind('<MouseWheel>', lambda e: self.zoom_at(1 if e.delta > 0 else -1, e.x, e.y))
        c.bind('<Button-4>', lambda e: self.zoom_at(1, e.x, e.y)); c.bind('<Button-5>', lambda e: self.zoom_at(-1, e.x, e.y))
        bar = ttk.Frame(self); bar.place(x=8, y=8)
        ttk.Button(bar, text='+', width=3, command=lambda: self.zoom_at(1)).pack(side='left')
        ttk.Button(bar, text='-', width=3, command=lambda: self.zoom_at(-1)).pack(side='left')
        ttk.Button(bar, text='Fit circle', command=self.fit).pack(side='left', padx=(4, 0))
        self.bm = tk.StringVar(value=self.basemap)
        ttk.Combobox(bar, textvariable=self.bm, values=list(BASEMAPS), width=22, state='readonly').pack(side='left', padx=4)
        self.bm.trace_add('write', lambda *a: self._set_basemap())
        self.attrib = ttk.Label(self, text='', font=('Segoe UI', 7), background='#ffffff')
        self.attrib.place(relx=1.0, rely=1.0, anchor='se')
        self.after(100, self._poll_tiles)
        self._set_basemap()

    # --- view
    def size(self): return max(self.canvas.winfo_width(), 50), max(self.canvas.winfo_height(), 50)

    def to_screen(self, wx, wy):
        w, h = self.size(); return wx - self.cx + w / 2, wy - self.cy + h / 2

    def zoom_at(self, dz, sx=None, sy=None):
        maxz = BASEMAPS[self.basemap][2]
        nz = min(max(self.z + dz, 3), maxz)
        if nz == self.z: return
        w, h = self.size(); sx = w / 2 if sx is None else sx; sy = h / 2 if sy is None else sy
        wx, wy = self.cx + sx - w / 2, self.cy + sy - h / 2
        f = 2 ** (nz - self.z)
        self.cx, self.cy, self.z = wx * f - (sx - w / 2), wy * f - (sy - h / 2), nz
        self.redraw()

    def fit(self):
        if not self.marker: return
        lon, lat = self.marker[1], self.marker[2]
        w, h = self.size(); r = max(self.radius, 200) * 1.25
        mpp = r * 2 / min(w, h)                              # metres per pixel wanted
        z = math.log2(156543.03 * math.cos(math.radians(lat)) / mpp)
        self.z = int(min(max(math.floor(z), 3), BASEMAPS[self.basemap][2]))
        self.cx, self.cy = lonlat_to_world(lon, lat, self.z)
        self.redraw()

    def screen_to_lonlat(self, x, y):
        w, h = self.size(); return world_to_lonlat(self.cx + x - w / 2, self.cy + y - h / 2, self.z)

    def start_box(self):
        self.box_mode = True; self.canvas.config(cursor='crosshair')

    def _press(self, e):
        if self.box_mode:
            self._box_start = (e.x, e.y); return
        self._drag = (e.x, e.y, self.cx, self.cy)

    def _move(self, e):
        if self.box_mode and self._box_start:
            self.canvas.delete('boxdrag')
            self.canvas.create_rectangle(*self._box_start, e.x, e.y, outline='#0050c8', width=2, dash=(6, 3), tags='boxdrag')
            return
        if not self._drag: return
        x0, y0, cx, cy = self._drag
        self.cx, self.cy = cx - (e.x - x0), cy - (e.y - y0)
        self.canvas.move('all', 0, 0)
        self.redraw(fast=True)

    def _dbl(self, e):
        if not self.on_place: return
        w, h = self.size()
        lon, lat = world_to_lonlat(self.cx + e.x - w / 2, self.cy + e.y - h / 2, self.z)
        self.on_place(lat, lon)

    def _release(self, e):
        if self.box_mode and self._box_start:
            (x0, y0), x1, y1 = self._box_start, e.x, e.y
            self.canvas.delete('boxdrag'); self._box_start = None; self.box_mode = False; self.canvas.config(cursor='')
            if abs(x1 - x0) > 8 and abs(y1 - y0) > 8:
                lo0, la0 = self.screen_to_lonlat(min(x0, x1), max(y0, y1)); lo1, la1 = self.screen_to_lonlat(max(x0, x1), min(y0, y1))
                self.box = (lo0, la0, lo1, la1)
                if self.on_box: self.on_box(self.box)
            self.redraw(); return
        self._drag = None; self.redraw()

    def _set_basemap(self):
        self.basemap = self.bm.get(); self.attrib.config(text=BASEMAPS[self.basemap][3])
        if self.z > BASEMAPS[self.basemap][2]: self.zoom_at(BASEMAPS[self.basemap][2] - self.z)
        self.redraw()

    # --- tiles
    def _fetch(self, key, url):
        try:
            req = urllib.request.Request(url, headers={'User-Agent': UA})
            with urllib.request.urlopen(req, timeout=20) as r: data = r.read()
            self.tq.put((key, data))
        except Exception:
            self.tq.put((key, None))
        finally:
            self.pool_sem.release()

    def _poll_tiles(self):
        got = False
        try:
            while True:
                key, data = self.tq.get_nowait()
                self.want.discard(key)
                if data:
                    try: self.tiles[key] = tk.PhotoImage(data=base64.b64encode(data)); got = True
                    except tk.TclError: self.tiles[key] = None
                else:
                    self.tiles[key] = None
        except queue.Empty:
            pass
        if got: self.redraw(tiles_only=True)
        self.after(150, self._poll_tiles)

    def _need(self, key, url):
        if key in self.tiles or key in self.want: return
        if len(self.tiles) > 600:                            # keep memory in check
            for k in list(self.tiles)[:200]: self.tiles.pop(k, None)
        self.want.add(key)
        def go():
            self.pool_sem.acquire(); self._fetch(key, url)
        threading.Thread(target=go, daemon=True).start()

    def draw_tiles(self):
        c = self.canvas; c.delete('tile')
        tmpl, subs, maxz, _ = BASEMAPS[self.basemap]
        if not tmpl: return
        w, h = self.size(); z = self.z; n = 2 ** z
        x0, y0 = self.cx - w / 2, self.cy - h / 2
        for ty in range(int(y0 // 256), int((y0 + h) // 256) + 1):
            if ty < 0 or ty >= n: continue
            for tx in range(int(x0 // 256), int((x0 + w) // 256) + 1):
                key = (self.basemap, z, tx % n, ty)
                img = self.tiles.get(key)
                if img:
                    c.create_image(tx * 256 - x0, ty * 256 - y0, image=img, anchor='nw', tags='tile')
                elif key not in self.tiles:
                    s = subs[(tx + ty) % len(subs)] if subs else ''
                    self._need(key, tmpl.format(s=s, z=z, x=tx % n, y=ty))
        c.tag_lower('tile')

    # --- overlays
    def set_marker(self, marker, radius):
        self.marker, self.radius = marker, radius; self.redraw()

    def set_pieces(self, pieces):
        """pieces: list of (fn, (lon0, lat0, lon1, lat1), depth, cached).  Few big pieces (USGS tiles) are drawn
        as themselves; many small ones (Amazon) are summarised on a ~30 x 30 progress grid so it reads at any zoom."""
        pcs = [(fn, b, d, c) for fn, b, d, c in pieces if b and None not in b]
        self.pieces = {}; self.cells = {}; self.drawn = {}
        if len(pcs) <= 400:
            for fn, b, d, cached in pcs:
                self.cells[fn] = {'b': b, 'n': {'cached' if cached else 'pending': 1}}
                self.pieces[fn] = (fn, 'cached' if cached else 'pending')
        elif pcs:
            # grid over the smallest pieces' extent (the big shallow ones cover far more than the area)
            depths = sorted({d for _, _, d, _ in pcs}); deep = [p for p in pcs if p[2] >= depths[len(depths) // 2]] or pcs
            lo0 = min(p[1][0] for p in deep); la0 = min(p[1][1] for p in deep)
            lo1 = max(p[1][2] for p in deep); la1 = max(p[1][3] for p in deep)
            nc = 30; dlo = (lo1 - lo0) / nc or 1e-6; dla = (la1 - la0) / nc or 1e-6
            for fn, b, d, cached in pcs:
                cx_, cy_ = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
                i = min(nc - 1, max(0, int((cx_ - lo0) / dlo))); j = min(nc - 1, max(0, int((cy_ - la0) / dla)))
                key = (i, j); st = 'cached' if cached else 'pending'
                cell = self.cells.setdefault(key, {'b': (lo0 + i * dlo, la0 + j * dla, lo0 + (i + 1) * dlo, la0 + (j + 1) * dla), 'n': {}})
                cell['n'][st] = cell['n'].get(st, 0) + 1
                self.pieces[fn] = (key, st)
        self.redraw()

    @staticmethod
    def cell_state(n):
        if n.get('fail'): return 'fail'
        if n.get('busy') or n.get('wait'): return 'busy'
        if n.get('pending'): return 'pending' if not n.get('done') else 'partial'
        return 'done' if n.get('done') else 'cached'

    def set_piece_state(self, fn, state):
        p = self.pieces.get(fn)
        if not p: return
        key, old = p
        if old == state: return
        n = self.cells[key]['n']; n[old] = n.get(old, 1) - 1; n[state] = n.get(state, 0) + 1
        self.pieces[fn] = (key, state)
        iid = self.drawn.get(key)
        if iid: self.canvas.itemconfig(iid, **piece_style(self.cell_state(n)))

    def set_processed(self, squares):
        self.processed = squares; self.redraw()

    def show_processed(self, on):
        self.processed_on = on; self.redraw()

    def fit_processed(self):
        if not self.processed: return False
        lons = [p[0] for _, c in self.processed for p in c]; lats = [p[1] for _, c in self.processed for p in c]
        w, h = self.size()
        for z in range(BASEMAPS[self.basemap][2], 3, -1):
            x0, y0 = lonlat_to_world(min(lons), max(lats), z); x1, y1 = lonlat_to_world(max(lons), min(lats), z)
            if x1 - x0 < w * 0.9 and y1 - y0 < h * 0.9: break
        self.z = z; self.cx, self.cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.redraw(); return True

    def show_pieces(self, on):
        self.pieces_on = on; self.redraw()


    def set_overlay(self, name, rgba_bounds):
        self.overlays[name] = rgba_bounds; self.overlay_on.setdefault(name, True); self.redraw()

    def show_overlay(self, name, on):
        self.overlay_on[name] = on; self.redraw()

    def draw_overlays(self):
        import numpy as np
        c = self.canvas; c.delete('ov'); self._ov_imgs = []
        w, h = self.size()
        for name in ('sweep', 'hidden'):
            ob = self.overlays.get(name)
            if not ob or not self.overlay_on.get(name): continue
            rgba, (mx0, my0, mx1, my1) = ob                  # rgba (4, rows, cols) in Web Mercator
            ax0, ay0 = self.to_screen(*merc_to_world(mx0, my1, self.z))
            ax1, ay1 = self.to_screen(*merc_to_world(mx1, my0, self.z))
            sx0, sy0, sx1, sy1 = max(0, int(ax0)), max(0, int(ay0)), min(w, int(ax1) + 1), min(h, int(ay1) + 1)
            if sx1 <= sx0 or sy1 <= sy0: continue
            rows, cols = rgba.shape[1:]
            xs = ((np.arange(sx0, sx1) + 0.5 - ax0) / (ax1 - ax0) * cols).astype(int).clip(0, cols - 1)
            ys = ((np.arange(sy0, sy1) + 0.5 - ay0) / (ay1 - ay0) * rows).astype(int).clip(0, rows - 1)
            img = np.ascontiguousarray(rgba[:, ys][:, :, xs].transpose(1, 2, 0))
            ph = tk.PhotoImage(data=base64.b64encode(png_bytes(img)))
            self._ov_imgs.append(ph)
            c.create_image(sx0, sy0, image=ph, anchor='nw', tags='ov')

    def draw_vectors(self, fast=False):
        c = self.canvas; c.delete('vec'); self.drawn = {}
        w, h = self.size()
        if self.processed and self.processed_on:
            for coll, corners in self.processed:
                pts = []
                for lo, la in corners: pts += self.to_screen(*lonlat_to_world(lo, la, self.z))
                xs, ys = pts[0::2], pts[1::2]
                if max(xs) < 0 or max(ys) < 0 or min(xs) > w or min(ys) > h: continue
                c.create_polygon(*pts, outline='#7b3294', fill='#7b3294', stipple='gray12', width=1, tags='vec')
        if getattr(self, 'cells', None) and not fast and getattr(self, 'pieces_on', True):
            for key, cell in self.cells.items():
                b = cell['b']
                x0, y0 = self.to_screen(*lonlat_to_world(b[0], b[3], self.z))
                x1, y1 = self.to_screen(*lonlat_to_world(b[2], b[1], self.z))
                if x1 < 0 or y1 < 0 or x0 > w or y0 > h: continue
                self.drawn[key] = c.create_rectangle(x0, y0, x1, y1, width=1, tags='vec', **piece_style(self.cell_state(cell['n'])))
        if self.box:
            b = self.box
            x0_, y0_ = self.to_screen(*lonlat_to_world(b[0], b[3], self.z)); x1_, y1_ = self.to_screen(*lonlat_to_world(b[2], b[1], self.z))
            c.create_rectangle(x0_, y0_, x1_, y1_, outline='#0050c8', width=3, dash=(8, 4), tags='vec')
        if self.marker:
            title, lon, lat = self.marker
            mx, my = self.to_screen(*lonlat_to_world(lon, lat, self.z))
            if self.radius > 0:
                pts = []
                for i in range(73):
                    t = 2 * math.pi * i / 72
                    dlat = self.radius * math.cos(t) / 110540.0; dlon = self.radius * math.sin(t) / (111320.0 * math.cos(math.radians(lat)))
                    pts += self.to_screen(*lonlat_to_world(lon + dlon, lat + dlat, self.z))
                c.create_line(*pts, fill='#000000', width=2, dash=(6, 3), tags='vec')
            c.create_oval(mx - 6, my - 6, mx + 6, my + 6, fill='#d7191c', outline='#ffffff', width=2, tags='vec')
            c.create_text(mx + 10, my - 10, text=title, anchor='sw', font=('Segoe UI', 9, 'bold'), fill='#000000', tags='vec')

    def redraw(self, tiles_only=False, fast=False):
        if tiles_only:
            self.draw_tiles(); return
        if fast:
            self.draw_tiles(); self.draw_vectors(fast=True); self.canvas.delete('ov'); return
        if self._redraw_pending: return
        self._redraw_pending = True
        def go():
            self._redraw_pending = False
            self.draw_tiles(); self.draw_overlays(); self.draw_vectors()
            self.canvas.tag_raise('ov'); self.canvas.tag_raise('vec')
        self.after_idle(go)


# ------------------------------------------------------------------ main window
class App(tk.Tk):
    def __init__(self, export=None):
        super().__init__()
        self.title(f'SARPOD Lidar - understory visibility and line of sight (experimental)   ({VERSION})')
        self.geometry('1320x900'); self.minsize(1000, 700)
        st = ttk.Style(self)
        try: st.theme_use('vista' if sys.platform == 'win32' else 'clam')
        except tk.TclError: pass
        st.configure('Bar.Horizontal.TProgressbar', background='#2e8b57', troughcolor='#e4e4e4', thickness=14)
        self.q = queue.Queue(); self.worker = None; self.plan = None; self.plan_key = None; self.markers = []
        self.job = None
        eng.HOOK = lambda ev, **kw: self.q.put((ev, kw))

        left = ttk.Frame(self, padding=8); left.pack(side='left', fill='y')
        self.map = MapView(self); self.map.pack(side='left', fill='both', expand=True)

        f = ttk.LabelFrame(left, text='Area', padding=6); f.pack(fill='x')
        self.export = tk.StringVar(value=export or '')
        r = ttk.Frame(f); r.pack(fill='x')
        ttk.Entry(r, textvariable=self.export, width=34).pack(side='left', fill='x', expand=True)
        ttk.Button(r, text='Browse...', command=self.browse).pack(side='left', padx=(4, 0))
        ttk.Label(f, text='Center (a marker from the export, or double-click the map, or type coordinates)',
                  wraplength=330).pack(anchor='w', pady=(6, 0))
        self.marker = tk.StringVar(); self.marker_box = ttk.Combobox(f, textvariable=self.marker, state='readonly', width=44)
        self.marker_box.pack(fill='x'); self.marker_box.bind('<<ComboboxSelected>>', lambda e: self.area_changed(fit=True))
        r = ttk.Frame(f); r.pack(fill='x', pady=(4, 0))
        self.coord = tk.StringVar()
        ce = ttk.Entry(r, textvariable=self.coord, width=26); ce.pack(side='left', fill='x', expand=True)
        ce.bind('<Return>', lambda e: self.go_coord())
        ttk.Button(r, text='Go to lat, lon', command=self.go_coord).pack(side='left', padx=(4, 0))
        r = ttk.Frame(f); r.pack(fill='x', pady=(6, 0))
        ttk.Label(r, text='Radius (m)').pack(side='left')
        self.radius = tk.StringVar(value='1000')
        e = ttk.Entry(r, textvariable=self.radius, width=8); e.pack(side='left', padx=4)
        e.bind('<KeyRelease>', lambda ev: self.area_changed()); e.bind('<FocusOut>', lambda ev: self.area_changed(fit=True))

        f = ttk.LabelFrame(left, text='Options', padding=6); f.pack(fill='x', pady=(8, 0))
        ttk.Label(f, text='Lidar from').pack(anchor='w')
        self.source = tk.StringVar(value='Auto: cached USGS tiles, else Amazon, else USGS')
        ttk.Combobox(f, textvariable=self.source, state='readonly', width=44,
                     values=['Auto: cached USGS tiles, else Amazon, else USGS', 'Amazon (fast, only the circle)',
                             'USGS tiles (slow download)']).pack(fill='x')
        self.terrain = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text='Include terrain (ledges, gullies, boulders)', variable=self.terrain).pack(anchor='w', pady=(4, 0))
        self.reuse = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text='Reuse processed ground (much faster near earlier runs)', variable=self.reuse).pack(anchor='w')
        r = ttk.Frame(f); r.pack(fill='x', pady=(4, 0))
        self.cache = tk.StringVar(value=eng.default_cache())
        ttk.Button(r, text='Lidar cache...', width=14, command=self.change_cache).pack(side='left')
        ttk.Label(r, textvariable=self.cache, foreground='#555555', wraplength=210).pack(side='left', padx=(4, 0))
        r = ttk.Frame(f); r.pack(fill='x', pady=(4, 0))
        self.out = tk.StringVar(value='')
        ttk.Button(r, text='Save maps in...', width=14, command=self.change_out).pack(side='left')
        self.out_label = ttk.Label(r, text='(next to the export)', foreground='#555555', wraplength=210); self.out_label.pack(side='left', padx=(4, 0))

        r = ttk.Frame(left); r.pack(fill='x', pady=8)
        self.b_check = ttk.Button(r, text='Check lidar', command=self.check); self.b_check.pack(side='left', expand=True, fill='x')
        self.b_run = ttk.Button(r, text='Make map', command=self.run); self.b_run.pack(side='left', expand=True, fill='x', padx=4)
        self.b_stop = ttk.Button(r, text='Stop', command=self.stop, state='disabled'); self.b_stop.pack(side='left', expand=True, fill='x')

        f = ttk.LabelFrame(left, text='One map of all processed ground in a box', padding=6); f.pack(fill='x', pady=(0, 8))
        r = ttk.Frame(f); r.pack(fill='x')
        ttk.Button(r, text='Draw box', command=self.draw_box).pack(side='left', expand=True, fill='x')
        self.b_proc = ttk.Button(r, text='Process all ground', command=self.process_box_run, state='disabled')
        self.b_proc.pack(side='left', expand=True, fill='x', padx=(4, 0))
        self.b_box = ttk.Button(r, text='Map processed ground', command=self.run_box, state='disabled')
        self.b_box.pack(side='left', expand=True, fill='x', padx=(4, 0))
        self.box_text = tk.StringVar(value='Process all ground: downloads + processes the whole box, then maps it.  Safe to leave overnight.')
        ttk.Label(f, textvariable=self.box_text, foreground='#555555', wraplength=330).pack(anchor='w', pady=(4, 0))

        f = ttk.LabelFrame(left, text='Progress', padding=6); f.pack(fill='x')
        self.dl_text = tk.StringVar(value='Download: -')
        ttk.Label(f, textvariable=self.dl_text).pack(anchor='w')
        self.dl_bar = ttk.Progressbar(f, length=320, mode='determinate', maximum=1000, style='Bar.Horizontal.TProgressbar'); self.dl_bar.pack(fill='x')
        self.st_text = tk.StringVar(value='Processing: -')
        ttk.Label(f, textvariable=self.st_text).pack(anchor='w', pady=(6, 0))
        self.st_bar = ttk.Progressbar(f, length=320, mode='determinate', maximum=1000, style='Bar.Horizontal.TProgressbar'); self.st_bar.pack(fill='x')
        lg = ttk.Frame(f); lg.pack(fill='x', pady=(6, 0))
        for state, label in (('pending', 'waiting'), ('busy', 'downloading'), ('done', 'done'), ('cached', 'cached'), ('fail', 'failed')):
            tk.Label(lg, text='  ', bg=COLORS[state][0]).pack(side='left'); ttk.Label(lg, text=label + ' ').pack(side='left')

        f = ttk.LabelFrame(left, text='Show on map', padding=6); f.pack(fill='x', pady=(8, 0))
        self.show_sweep = tk.BooleanVar(value=True); self.show_hidden = tk.BooleanVar(value=True)
        self.show_proc = tk.BooleanVar(value=True); self.show_pieces = tk.BooleanVar(value=True)
        g = ttk.Frame(f); g.pack(fill='x')
        for i, (txt, var, cmd) in enumerate((
                ('Understory (greens)', self.show_sweep, lambda: self.map.show_overlay('sweep', self.show_sweep.get())),
                ('Hidden ground (orange/red)', self.show_hidden, lambda: self.map.show_overlay('hidden', self.show_hidden.get())),
                ('Processed ground (purple)', self.show_proc, lambda: self.map.show_processed(self.show_proc.get())),
                ('Download grid', self.show_pieces, lambda: self.map.show_pieces(self.show_pieces.get())))):
            ttk.Checkbutton(g, text=txt, variable=var, command=cmd).grid(row=i // 2, column=i % 2, sticky='w', padx=(0, 8))
        self.b_open = ttk.Button(f, text='Open output folder', command=self.open_out, state='disabled'); self.b_open.pack(fill='x', pady=(4, 0))

        self.log = ScrolledText(left, width=46, height=7, font=('Consolas', 8), wrap='word'); self.log.pack(fill='both', expand=True, pady=(8, 0))
        self.map.on_place = self.place
        self.map.on_box = self.box_drawn
        self.say(f'Lidar cache: {self.cache.get()}')
        self.say('Pick a center (export marker, double-click the map, or type lat, lon) and a radius.  '
                 '"Check lidar" shows what is needed; "Make map" runs it.')
        self.after(100, self.poll)
        self.after(200, lambda: self.scan_processed(fit=not (export and os.path.exists(export))))
        if export and os.path.exists(export): self.after(300, lambda: self.load_export(export))

    # --- helpers
    def say(self, msg):
        self.log.insert('end', msg.rstrip() + '\n'); self.log.see('end')

    def browse(self):
        p = filedialog.askopenfilename(title='SARTopo / CalTopo export', filetypes=[('SARTopo export', '*.json *.geojson'), ('All files', '*.*')])
        if p: self.export.set(p); self.load_export(p)

    def scan_processed(self, fit=False):
        root = self.cache.get()
        def work():
            try: sq = eng.list_processed(root)
            except Exception: sq = []
            self.q.put(('processed', {'squares': sq, 'fit': fit}))
        threading.Thread(target=work, daemon=True).start()

    def draw_box(self):
        self.map.start_box(); self.say('Drag a box on the map around the processed (purple) ground you want in one map.')

    def box_drawn(self, b):
        w = (b[2] - b[0]) * 111.32 * math.cos(math.radians((b[1] + b[3]) / 2)); h = (b[3] - b[1]) * 110.54
        self.say(f'Box drawn: about {w:.1f} x {h:.1f} km.  Click "Map processed ground in box".')
        self.b_box.config(state='normal'); self.b_proc.config(state='normal')

    def process_box_run(self):
        b = self.map.box
        if not b: return
        out = self.out.get() or os.path.join(os.path.expanduser('~'), 'Downloads')
        terrain = self.terrain.get(); root = self.cache.get(); src = self.source_key()
        stamp = time.strftime('%Y%m%d_%H%M')
        logf = os.path.join(out, f'SARPOD_box_{stamp}_log.txt')
        log = lambda m: self.q.put(('log', {'msg': m}))
        def work():
            try:
                os.makedirs(out, exist_ok=True)
                eng.process_box(root, b, src, None, 16, log, log_file=logf)
                log('Making the map of the whole box...')
                job = eng.map_processed(root, b, out, terrain=terrain, log=log, name=f'SARPOD_box_{stamp}')
                try:
                    with open(logf, 'a', encoding='utf-8') as fh: fh.write('\n'.join(job['lines']) + '\n')
                except OSError: pass
                self.q.put(('result', {'job': job}))
            except eng.Cancelled:
                self.q.put(('stopped', {}))
            except Exception as e:
                self.q.put(('error', {'msg': str(e), 'tb': traceback.format_exc()}))
        self.say(f'Processing the whole box.  A log is kept in {logf}')
        self.dl_bar['value'] = 0; self.st_bar['value'] = 0
        if self.worker and self.worker.is_alive(): return
        eng.CANCEL.clear(); self.busy(True); self.planning(True, 'Working through the box...')
        self.worker = threading.Thread(target=work, daemon=True); self.worker.start()

    def run_box(self):
        b = self.map.box
        if not b: return
        out = self.out.get() or os.path.join(os.path.expanduser('~'), 'Downloads')
        terrain = self.terrain.get(); root = self.cache.get()
        log = lambda m: self.q.put(('log', {'msg': m}))
        def work():
            try:
                job = eng.map_processed(root, b, out, terrain=terrain, log=log)
                self.q.put(('result', {'job': job}))
            except eng.Cancelled:
                self.q.put(('stopped', {}))
            except Exception as e:
                self.q.put(('error', {'msg': str(e), 'tb': traceback.format_exc()}))
        self.dl_bar['value'] = 0; self.st_bar['value'] = 0; self.dl_text.set('Download: none needed (processed ground only)')
        if self.worker and self.worker.is_alive(): return
        eng.CANCEL.clear(); self.busy(True)
        self.worker = threading.Thread(target=work, daemon=True); self.worker.start()

    def place(self, lat, lon, label=None):
        """Center from a map double-click or typed coordinates (no export needed)."""
        title = label or f'Point {lat:.5f} {lon:.5f}'
        self.markers = [m for m in self.markers if not m[0].startswith('Point ')] + [(title, lon, lat)]
        self.marker_box['values'] = [f'{t}   ({la:.5f}, {lo:.5f})' for t, lo, la in self.markers]
        self.marker_box.current(len(self.markers) - 1)
        self.coord.set(f'{lat:.5f}, {lon:.5f}')
        self.area_changed(fit=False); self.say(f'Center set to {lat:.5f}, {lon:.5f}.')

    def go_coord(self):
        try:
            lat, lon = eng.parse_latlon(self.coord.get())
        except ValueError as e:
            messagebox.showinfo('Coordinates', str(e)); return
        self.place(lat, lon); self.area_changed(fit=True)

    def load_export(self, p):
        try:
            self.markers = eng.read_markers(p)
        except Exception as e:
            messagebox.showerror('Export', f'Could not read the export:\n{e}'); return
        if not self.markers:
            messagebox.showwarning('Export', 'No markers in this export.'); return
        self.marker_box['values'] = [f'{t}   ({la:.5f}, {lo:.5f})' for t, lo, la in self.markers]
        pick = next((i for i, m in enumerate(self.markers) if m[0].upper() in ('IPP', 'LKP', 'PLS')), 0)
        self.marker_box.current(pick); self.area_changed(fit=True)
        self.say(f'{os.path.basename(p)}: {len(self.markers)} markers.')

    def area(self):
        i = self.marker_box.current()
        if i < 0 or not self.markers: return None
        try: r = float(self.radius.get())
        except ValueError: return None
        if r <= 0: return None
        return self.markers[i], r

    def area_changed(self, fit=False):
        a = self.area()
        if not a: return
        self.map.set_marker(a[0], a[1])
        if fit: self.map.fit()

    def source_key(self):
        s = self.source.get()
        return 'aws' if s.startswith('Amazon') else 'usgs' if s.startswith('USGS') else 'auto'

    def change_cache(self):
        p = filedialog.askdirectory(title='Folder for the lidar (e.g. in Google Drive)', initialdir=self.cache.get())
        if not p: return
        p = os.path.normpath(p); os.makedirs(os.path.dirname(eng.SETTING), exist_ok=True)
        with open(eng.SETTING, 'w', encoding='utf-8') as fh: fh.write(p)
        self.cache.set(p); self.plan = None; self.say(f'Lidar cache is now {p} (remembered on this computer).')
        self.scan_processed(fit=not self.map.marker)

    def change_out(self):
        p = filedialog.askdirectory(title='Save the maps in')
        if p: self.out.set(os.path.normpath(p)); self.out_label.config(text=self.out.get())

    def open_out(self):
        d = (self.job or {}).get('out_dir')
        if not d: return
        if sys.platform == 'win32': os.startfile(d)
        elif sys.platform == 'darwin': os.system(f'open "{d}"')
        else: os.system(f'xdg-open "{d}" &')

    def busy(self, on):
        for b in (self.b_check, self.b_run): b.config(state='disabled' if on else 'normal')
        if hasattr(self, 'b_box'):
            for bb in (self.b_box, self.b_proc): bb.config(state='disabled' if on or not self.map.box else 'normal')
        keep_awake(on)
        self.b_stop.config(state='normal' if on else 'disabled')

    # --- actions
    def planning(self, on, text='Looking up the lidar for this area...'):
        if on:
            self.dl_bar.config(mode='indeterminate'); self.dl_bar.start(15); self.dl_text.set(text)
        elif str(self.dl_bar.cget('mode')) == 'indeterminate':
            self.dl_bar.stop(); self.dl_bar.config(mode='determinate'); self.dl_bar['value'] = 0

    def _start(self, fn):
        if self.worker and self.worker.is_alive(): return
        eng.CANCEL.clear(); self.busy(True)
        if not (self.plan and self.plan_key and self.area() and self._key(self.area()) == self.plan_key): self.planning(True)
        self.worker = threading.Thread(target=fn, daemon=True); self.worker.start()

    def _key(self, a):
        (title, lon, lat), r = a
        return (title, lon, lat, r, self.source_key(), self.cache.get(), self.reuse.get())

    def _plan(self, a, log):
        (title, lon, lat), r = a
        key = self._key(a)
        if self.plan and self.plan_key == key: return self.plan
        box = eng.area_bbox(lon, lat, r) if self.reuse.get() else eng.bbox_around(lon, lat, r)
        plan = eng.plan_lidar(box, self.cache.get(), self.source_key(), None, log, cover_bbox=eng.bbox_around(lon, lat, r, 0))
        self.plan, self.plan_key = plan, key
        return plan

    def check(self):
        a = self.area()
        if not a: messagebox.showinfo('Area', 'Pick a center and a radius first.'); return
        log = lambda m: self.q.put(('log', {'msg': m}))
        def work():
            try:
                plan = self._plan(a, log); eng.show_plan(plan)
                n = len(plan['items']); have = sum(1 for it in plan['items'] if os.path.exists(eng.piece_path(it)))
                size = sum(int(it.get('sizeInBytes') or 0) for it in plan['items'] if not os.path.exists(eng.piece_path(it)))
                log(f'{plan["source"]}: {plan["name"]}.  {n} pieces, {have} already cached' +
                    (f', {size / 1e6:,.0f} MB to download.' if size else '.'))
                self.q.put(('idle', {}))
            except Exception as e:
                self.q.put(('error', {'msg': str(e), 'tb': traceback.format_exc()}))
        self._start(work)

    def run(self):
        a = self.area(); exp = self.export.get().strip()
        if not a: messagebox.showinfo('Area', 'Pick a center and a radius first.'); return
        if exp and not os.path.exists(exp): exp = ''
        if not exp and not self.out.get():
            self.out.set(os.path.join(os.path.expanduser('~'), 'Downloads')); self.out_label.config(text=self.out.get())
        (title, lon, lat), r = a
        log = lambda m: self.q.put(('log', {'msg': m}))
        terrain = self.terrain.get(); out = self.out.get() or None; reuse = self.reuse.get()
        def work():
            try:
                plan = self._plan(a, log)
                job = eng.run_job(exp, title, lon, lat, r, plan, self.cache.get(), terrain=terrain, out_dir=out, log=log, reuse=reuse)
                self.q.put(('result', {'job': job}))
            except eng.Cancelled:
                self.q.put(('stopped', {}))
            except (eng.DownloadIncomplete, eng.DamagedTile) as e:
                self.q.put(('error', {'msg': str(e), 'tb': ''}))
            except Exception as e:
                self.q.put(('error', {'msg': str(e), 'tb': traceback.format_exc()}))
        self.dl_bar['value'] = 0; self.st_bar['value'] = 0
        self._start(work)

    def stop(self):
        eng.CANCEL.set(); self.say('Stopping... (downloads finished so far are kept)')

    # --- events from the worker
    def poll(self):
        states = {}
        try:
            for _ in range(5000):
                ev, kw = self.q.get_nowait()
                if ev == 'log': self.say(kw['msg'])
                elif ev == 'pieces':
                    self.planning(False); self.map.set_pieces(kw['pieces'])
                    n = len(kw['pieces']); have = sum(1 for p in kw['pieces'] if p[3])
                    self.dl_text.set(f'Download: {n - have:,} of {n:,} pieces to get ({have:,} already cached)')
                elif ev == 'piece': states[kw['fn']] = kw['state']
                elif ev == 'bytes':
                    self.planning(False)
                    done, total, files, nfiles = kw['done'], kw['total'], kw['files'], kw['nfiles']
                    rate = kw['rate'] / 1e6
                    frac = done / total if total else (files / nfiles if nfiles else 1)
                    self.dl_bar['value'] = 1000 * min(frac, 1)
                    left = ''
                    if rate > 0 and total: left = f', about {max(total - done, 0) / 1e6 / rate / 60:.0f} min left'
                    self.dl_text.set(f'Download: {done / 1e6:,.0f}' + (f' of {total / 1e6:,.0f} MB' if total else ' MB') +
                                     f',  {files} of {nfiles} files,  {rate:.1f} MB/s,  {kw["at_once"]} at a time{left}')
                elif ev == 'stage' and kw['n'] == 0:          # planning: count only
                    self.dl_text.set(f"{kw['name']}: {kw['i']:,}")
                elif ev == 'stage':
                    self.planning(False)
                    n = max(kw['n'], 1)
                    self.st_bar['value'] = 1000 * kw['i'] / n
                    self.st_text.set(f'Processing: {kw["name"]}  {kw["i"]} of {kw["n"]}')
                    if self.dl_bar['value'] < 1000 and kw['name'] != 'Writing maps': self.dl_bar['value'] = 1000
                elif ev == 'result':
                    self.job = kw['job']; self.busy(False); self.b_open.config(state='normal')
                    self.st_bar['value'] = 1000; self.st_text.set('Processing: done')
                    self.say('\n'.join(self.job['lines']))
                    self.say(f"Saved: {self.job['base']}_*.tif")
                    if self.job.get('preview_sweep'): self.map.set_overlay('sweep', self.job['preview_sweep'])
                    if self.job.get('preview_hidden'): self.map.set_overlay('hidden', self.job['preview_hidden'])
                    self.map.show_overlay('sweep', self.show_sweep.get()); self.map.show_overlay('hidden', self.show_hidden.get())
                    self.scan_processed()
                elif ev == 'chunk':
                    el = kw['elapsed']; i, n = kw['i'], kw['n']
                    left = f',  about {el / max(i - 1, 1) * (n - i + 1) / 60:.0f} min to go' if i > 1 else ''
                    self.box_text.set(f"Box: chunk {i} of {n}  ({kw['done']} processed, {kw['skipped']} already done, "
                                      f"{kw['failed']} no lidar){left}")
                elif ev == 'processed':
                    sq = kw['squares']; self.map.set_processed(sq)
                    if sq:
                        km2 = len(sq) * (eng.SQ / 1000) ** 2
                        colls = sorted({c for c, _ in sq})
                        self.say(f'Processed ground in the cache: {len(sq)} squares ({km2:.1f} km2), '
                                 f'{len(colls)} lidar collection(s).  Shown in purple.')
                    if kw.get('fit') and sq and not self.map.marker: self.after(100, self.map.fit_processed)
                elif ev == 'stopped':
                    self.busy(False); self.planning(False); self.say('Stopped.  Run "Make map" again to carry on; nothing downloaded is lost.')
                elif ev == 'idle':
                    self.busy(False); self.planning(False)
                elif ev == 'error':
                    self.busy(False); self.planning(False); self.say('\n' + kw['msg'])
                    if kw.get('tb'): self.say(kw['tb'])
                    messagebox.showerror('SARPOD Lidar', kw['msg'][:800])
        except queue.Empty:
            pass
        try:
            for fn, st in states.items(): self.map.set_piece_state(fn, st)
        except Exception:
            traceback.print_exc()
        self.after(150, self.poll)


def main():
    if sys.platform == 'win32':
        try:
            import ctypes; ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    App(sys.argv[1] if len(sys.argv) > 1 else None).mainloop()


if __name__ == '__main__':
    main()
