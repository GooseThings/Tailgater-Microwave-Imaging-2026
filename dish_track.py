# dish_track.py
# Real-time satellite tracker for the Tailgater dish antenna
# Downloads TLE data from Celestrak, lets you pick a satellite from a GUI,
# then drives the dish motors over serial to follow the satellite in real time.
#
# Requirements: pip install skyfield pyserial pillow requests tkinter
#
# Usage: python3 dish_track.py

import serial
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import requests
import re as stdlib_re

from skyfield.api import load, EarthSatellite, Topos

# ─────────────────────────────────────────────────────────────────
# USER CONFIGURATION — edit these before running
# ─────────────────────────────────────────────────────────────────

SERIAL_PORT   = '/dev/ttyACM0'   # Change if your dish is on a different port
BAUD_RATE     = 9600

OBSERVER_LAT  = 42.87            # Your latitude  (decimal degrees)
OBSERVER_LON  = -85.68           # Your longitude (decimal degrees)

# Minimum elevation to track (degrees). Below this the satellite is too low.
MIN_ELEVATION = 5
MAX_ELEVATION = 70

# How often to update the dish position (seconds)
UPDATE_INTERVAL = 2.0

# Celestrak TLE catalogue URLs — the GUI lets you pick which group to load
CELESTRAK_SOURCES = {
    "Space Stations (ISS, CSS…)":     "https://celestrak.org/SOCRATES/query.php?CATALOG=stations&FORMAT=TLE",
    "Active Satellites":               "https://celestrak.org/SOCRATES/query.php?CATALOG=active&FORMAT=TLE",
    "Amateur Radio":                   "https://celestrak.org/SOCRATES/query.php?CATALOG=amateur&FORMAT=TLE",
    "Weather":                         "https://celestrak.org/SOCRATES/query.php?CATALOG=weather&FORMAT=TLE",
    "NOAA":                            "https://celestrak.org/SOCRATES/query.php?CATALOG=noaa&FORMAT=TLE",
    "GPS":                             "https://celestrak.org/SOCRATES/query.php?CATALOG=gps-ops&FORMAT=TLE",
    "Iridium":                         "https://celestrak.org/SOCRATES/query.php?CATALOG=iridium&FORMAT=TLE",
    "Starlink":                        "https://celestrak.org/SOCRATES/query.php?CATALOG=starlink&FORMAT=TLE",
    "Geostationary":                   "https://celestrak.org/SOCRATES/query.php?CATALOG=geo&FORMAT=TLE",
    "All (visual, large list)":        "https://celestrak.org/SOCRATES/query.php?CATALOG=visual&FORMAT=TLE",
}

# ─────────────────────────────────────────────────────────────────
# DISH SERIAL HELPERS  (mirrors dish_scan.py protocol exactly)
# ─────────────────────────────────────────────────────────────────

def send_str(dish, text):
    """Send a string one byte at a time — the Tailgater only accepts single chars."""
    for ch in text:
        dish.write(ch.encode())
    dish.write(b'\r')
    time.sleep(0.05)


def send_azangle(dish, degrees):
    """Command the dish to an absolute azimuth (0-360, CCW from coax jack)."""
    send_str(dish, f"azangle {int(degrees)}")
    time.sleep(0.1)


def send_elangle(dish, degrees):
    """Command the dish to an absolute elevation (5-70 degrees)."""
    degrees = max(MIN_ELEVATION, min(MAX_ELEVATION, int(degrees)))
    send_str(dish, f"elangle {int(degrees)}")
    time.sleep(0.1)


def dish_az_from_compass(compass_az):
    """
    Convert a standard compass azimuth (CW from North) to the dish's
    CCW coordinate system.  The dish treats its coax jack as North/0°
    and increments counter-clockwise.
    """
    return (360 - compass_az) % 360


# ─────────────────────────────────────────────────────────────────
# TLE DOWNLOAD
# ─────────────────────────────────────────────────────────────────

def fetch_tle_catalogue(url):
    """
    Download a TLE file from Celestrak and parse it into a list of
    (name, line1, line2) tuples.
    """
    try:
        resp = requests.get(url, timeout=15)
        resp.raise_for_status()
    except requests.RequestException as e:
        raise RuntimeError(f"Failed to download TLE data:\n{e}")

    lines = [l.strip() for l in resp.text.splitlines() if l.strip()]
    satellites = []
    i = 0
    while i + 2 < len(lines):
        name = lines[i]
        l1   = lines[i + 1]
        l2   = lines[i + 2]
        if l1.startswith('1 ') and l2.startswith('2 '):
            satellites.append((name, l1, l2))
            i += 3
        else:
            i += 1
    return satellites


# ─────────────────────────────────────────────────────────────────
# SATELLITE TRACKER CORE
# ─────────────────────────────────────────────────────────────────

class SatelliteTracker:
    def __init__(self, tle_name, tle_line1, tle_line2, observer_lat, observer_lon):
        self.ts        = load.timescale()
        self.observer  = Topos(latitude_degrees=observer_lat,
                               longitude_degrees=observer_lon)
        self.satellite = EarthSatellite(tle_line1, tle_line2, tle_name, self.ts)
        self.name      = tle_name

    def get_position(self):
        """Return current (azimuth_deg, elevation_deg, distance_km)."""
        t           = self.ts.now()
        difference  = self.satellite - self.observer
        topocentric = difference.at(t)
        alt, az, dist = topocentric.altaz()
        return az.degrees, alt.degrees, dist.km

    def predict_pass(self, minutes=90, step_seconds=30):
        """
        Return a list of (time_str, az, el) tuples for the next `minutes` minutes,
        sampled every `step_seconds` seconds — only positions where el > MIN_ELEVATION.
        """
        ts    = self.ts
        now   = ts.now()
        steps = int((minutes * 60) / step_seconds)
        path  = []
        for i in range(steps):
            t          = ts.tt_jd(now.tt + (i * step_seconds) / 86400.0)
            diff       = self.satellite - self.observer
            top        = diff.at(t)
            alt, az, _ = top.altaz()
            if alt.degrees > MIN_ELEVATION:
                path.append((t.utc_strftime('%H:%M:%S'), az.degrees, alt.degrees))
        return path


# ─────────────────────────────────────────────────────────────────
# TRACKING THREAD
# ─────────────────────────────────────────────────────────────────

class TrackingThread(threading.Thread):
    def __init__(self, tracker, dish, status_callback):
        super().__init__(daemon=True)
        self.tracker         = tracker
        self.dish            = dish
        self.status_callback = status_callback
        self.running         = True

    def run(self):
        while self.running:
            try:
                compass_az, el, dist = self.tracker.get_position()
                dish_az = dish_az_from_compass(compass_az)

                msg = (f"Az (compass): {compass_az:.1f}°  |  "
                       f"El: {el:.1f}°  |  "
                       f"Dist: {dist:.0f} km  |  "
                       f"Dish cmd Az: {dish_az:.0f}°")

                if el < MIN_ELEVATION:
                    msg = f"[BELOW HORIZON]  El: {el:.1f}°  —  dish holding position"
                    self.status_callback(msg, below_horizon=True)
                else:
                    # Send position commands to dish
                    send_azangle(self.dish, dish_az)
                    send_elangle(self.dish, el)
                    self.dish.flush()
                    self.dish.reset_output_buffer()
                    self.status_callback(msg, below_horizon=False)

            except Exception as e:
                self.status_callback(f"Error: {e}", below_horizon=False)

            time.sleep(UPDATE_INTERVAL)

    def stop(self):
        self.running = False


# ─────────────────────────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────────────────────────

class TrackerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Tailgater Satellite Tracker")
        self.resizable(True, True)
        self.configure(bg="#1e1e2e")

        self.satellites      = []   # List of (name, l1, l2)
        self.dish            = None
        self.tracking_thread = None
        self.tracker         = None

        self._build_ui()

    # ── UI CONSTRUCTION ──────────────────────────────────────────

    def _build_ui(self):
        pad = dict(padx=10, pady=6)

        # ── Title bar ──
        title = tk.Label(self, text="🛰  Tailgater Satellite Tracker",
                         font=("Helvetica", 16, "bold"),
                         bg="#1e1e2e", fg="#cdd6f4")
        title.pack(fill="x", padx=10, pady=(12, 4))

        # ── Top frame: catalogue + serial ──
        top = tk.Frame(self, bg="#1e1e2e")
        top.pack(fill="x", padx=10, pady=4)

        # Catalogue selector
        cat_frame = tk.LabelFrame(top, text=" TLE Catalogue ", bg="#1e1e2e",
                                  fg="#89b4fa", font=("Helvetica", 10, "bold"))
        cat_frame.grid(row=0, column=0, sticky="ew", padx=(0, 8), pady=2)
        top.columnconfigure(0, weight=3)

        self.cat_var = tk.StringVar(value=list(CELESTRAK_SOURCES.keys())[0])
        cat_menu = ttk.Combobox(cat_frame, textvariable=self.cat_var,
                                values=list(CELESTRAK_SOURCES.keys()), width=36,
                                state="readonly")
        cat_menu.pack(side="left", padx=6, pady=4)

        self.load_btn = tk.Button(cat_frame, text="Download TLEs",
                                  command=self._download_tles,
                                  bg="#313244", fg="#a6e3a1",
                                  activebackground="#45475a",
                                  relief="flat", padx=8)
        self.load_btn.pack(side="left", padx=4, pady=4)

        self.tle_status = tk.Label(cat_frame, text="Not loaded",
                                   bg="#1e1e2e", fg="#6c7086", font=("Helvetica", 9))
        self.tle_status.pack(side="left", padx=6)

        # Serial port config
        ser_frame = tk.LabelFrame(top, text=" Serial Port ", bg="#1e1e2e",
                                   fg="#89b4fa", font=("Helvetica", 10, "bold"))
        ser_frame.grid(row=0, column=1, sticky="ew", pady=2)
        top.columnconfigure(1, weight=1)

        tk.Label(ser_frame, text="Port:", bg="#1e1e2e", fg="#cdd6f4").pack(side="left", padx=(6,2))
        self.port_var = tk.StringVar(value=SERIAL_PORT)
        port_entry = tk.Entry(ser_frame, textvariable=self.port_var, width=14,
                              bg="#313244", fg="#cdd6f4", insertbackground="#cdd6f4",
                              relief="flat")
        port_entry.pack(side="left", padx=2, pady=4)

        self.connect_btn = tk.Button(ser_frame, text="Connect Dish",
                                     command=self._connect_dish,
                                     bg="#313244", fg="#89b4fa",
                                     activebackground="#45475a",
                                     relief="flat", padx=8)
        self.connect_btn.pack(side="left", padx=6, pady=4)

        self.conn_status = tk.Label(ser_frame, text="⬤ Disconnected",
                                    bg="#1e1e2e", fg="#f38ba8", font=("Helvetica", 9))
        self.conn_status.pack(side="left", padx=4)

        # ── Search + satellite list ──
        list_frame = tk.LabelFrame(self, text=" Satellite List ", bg="#1e1e2e",
                                   fg="#89b4fa", font=("Helvetica", 10, "bold"))
        list_frame.pack(fill="both", expand=True, padx=10, pady=4)

        search_row = tk.Frame(list_frame, bg="#1e1e2e")
        search_row.pack(fill="x", padx=6, pady=(4, 2))
        tk.Label(search_row, text="Filter:", bg="#1e1e2e", fg="#cdd6f4").pack(side="left")
        self.search_var = tk.StringVar()
        self.search_var.trace_add("write", self._filter_list)
        search_entry = tk.Entry(search_row, textvariable=self.search_var, width=30,
                                bg="#313244", fg="#cdd6f4", insertbackground="#cdd6f4",
                                relief="flat")
        search_entry.pack(side="left", padx=6)

        self.sat_count_label = tk.Label(search_row, text="",
                                        bg="#1e1e2e", fg="#6c7086", font=("Helvetica", 9))
        self.sat_count_label.pack(side="left")

        list_inner = tk.Frame(list_frame, bg="#1e1e2e")
        list_inner.pack(fill="both", expand=True, padx=6, pady=4)

        scrollbar = tk.Scrollbar(list_inner)
        scrollbar.pack(side="right", fill="y")

        self.sat_listbox = tk.Listbox(list_inner, yscrollcommand=scrollbar.set,
                                       bg="#313244", fg="#cdd6f4",
                                       selectbackground="#89b4fa",
                                       selectforeground="#1e1e2e",
                                       font=("Courier", 10),
                                       height=12, relief="flat",
                                       activestyle="dotbox")
        self.sat_listbox.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.sat_listbox.yview)

        # ── Pass prediction table ──
        pass_frame = tk.LabelFrame(self, text=" Upcoming Pass Preview (next 90 min) ",
                                   bg="#1e1e2e", fg="#89b4fa",
                                   font=("Helvetica", 10, "bold"))
        pass_frame.pack(fill="x", padx=10, pady=4)

        cols = ("Time (UTC)", "Azimuth (°)", "Elevation (°)")
        self.pass_tree = ttk.Treeview(pass_frame, columns=cols, show="headings", height=5)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview",
                         background="#313244", foreground="#cdd6f4",
                         fieldbackground="#313244", rowheight=22)
        style.configure("Treeview.Heading", background="#45475a", foreground="#89b4fa")
        for col in cols:
            self.pass_tree.heading(col, text=col)
            self.pass_tree.column(col, width=140, anchor="center")
        self.pass_tree.pack(fill="x", padx=6, pady=4)

        # ── Control buttons ──
        btn_row = tk.Frame(self, bg="#1e1e2e")
        btn_row.pack(fill="x", padx=10, pady=4)

        self.preview_btn = tk.Button(btn_row, text="Preview Pass",
                                      command=self._preview_pass,
                                      bg="#313244", fg="#fab387",
                                      activebackground="#45475a",
                                      relief="flat", padx=12, pady=4,
                                      state="disabled")
        self.preview_btn.pack(side="left", padx=4)

        self.track_btn = tk.Button(btn_row, text="▶  Start Tracking",
                                    command=self._start_tracking,
                                    bg="#a6e3a1", fg="#1e1e2e",
                                    activebackground="#94e2d5",
                                    font=("Helvetica", 11, "bold"),
                                    relief="flat", padx=16, pady=4,
                                    state="disabled")
        self.track_btn.pack(side="left", padx=4)

        self.stop_btn = tk.Button(btn_row, text="■  Stop",
                                   command=self._stop_tracking,
                                   bg="#f38ba8", fg="#1e1e2e",
                                   activebackground="#eba0ac",
                                   font=("Helvetica", 11, "bold"),
                                   relief="flat", padx=12, pady=4,
                                   state="disabled")
        self.stop_btn.pack(side="left", padx=4)

        # ── Live status bar ──
        status_frame = tk.Frame(self, bg="#181825")
        status_frame.pack(fill="x", side="bottom")

        self.status_label = tk.Label(status_frame,
                                      text="Ready — download a TLE catalogue to begin.",
                                      bg="#181825", fg="#6c7086",
                                      font=("Courier", 10), anchor="w")
        self.status_label.pack(fill="x", padx=10, pady=6)

    # ── EVENT HANDLERS ───────────────────────────────────────────

    def _download_tles(self):
        self.tle_status.config(text="Downloading…", fg="#f9e2af")
        self.load_btn.config(state="disabled")
        self.update_idletasks()

        url = CELESTRAK_SOURCES[self.cat_var.get()]

        def do_download():
            try:
                sats = fetch_tle_catalogue(url)
                self.satellites = sats
                self.after(0, lambda: self._populate_list(sats))
            except RuntimeError as e:
                self.after(0, lambda: messagebox.showerror("Download Failed", str(e)))
                self.after(0, lambda: self.tle_status.config(text="Failed", fg="#f38ba8"))
            finally:
                self.after(0, lambda: self.load_btn.config(state="normal"))

        threading.Thread(target=do_download, daemon=True).start()

    def _populate_list(self, sats):
        self.sat_listbox.delete(0, tk.END)
        for name, _, _ in sats:
            self.sat_listbox.insert(tk.END, name)
        self.tle_status.config(text=f"{len(sats)} satellites loaded", fg="#a6e3a1")
        self.sat_count_label.config(text=f"({len(sats)} shown)")
        self.preview_btn.config(state="normal")
        self._update_track_btn_state()

    def _filter_list(self, *_):
        query = self.search_var.get().lower()
        self.sat_listbox.delete(0, tk.END)
        shown = 0
        for name, _, _ in self.satellites:
            if query in name.lower():
                self.sat_listbox.insert(tk.END, name)
                shown += 1
        self.sat_count_label.config(text=f"({shown} shown)")

    def _connect_dish(self):
        port = self.port_var.get().strip()
        try:
            if self.dish and self.dish.is_open:
                self.dish.close()
            self.dish = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.conn_status.config(text=f"⬤ Connected ({port})", fg="#a6e3a1")
            self.connect_btn.config(text="Disconnect", command=self._disconnect_dish)
            self._update_track_btn_state()
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", str(e))

    def _disconnect_dish(self):
        self._stop_tracking()
        if self.dish and self.dish.is_open:
            self.dish.close()
        self.conn_status.config(text="⬤ Disconnected", fg="#f38ba8")
        self.connect_btn.config(text="Connect Dish", command=self._connect_dish)
        self._update_track_btn_state()

    def _update_track_btn_state(self):
        dish_ok = self.dish is not None and self.dish.is_open
        sats_ok = len(self.satellites) > 0
        if dish_ok and sats_ok:
            self.track_btn.config(state="normal")
        else:
            self.track_btn.config(state="disabled")

    def _get_selected_satellite(self):
        sel = self.sat_listbox.curselection()
        if not sel:
            messagebox.showwarning("No Selection", "Please select a satellite from the list.")
            return None
        name = self.sat_listbox.get(sel[0])
        # Find matching TLE in full list
        for sat_name, l1, l2 in self.satellites:
            if sat_name == name:
                return sat_name, l1, l2
        return None

    def _preview_pass(self):
        result = self._get_selected_satellite()
        if not result:
            return
        name, l1, l2 = result
        tracker = SatelliteTracker(name, l1, l2, OBSERVER_LAT, OBSERVER_LON)
        path = tracker.predict_pass(minutes=90, step_seconds=30)

        # Clear old data
        for row in self.pass_tree.get_children():
            self.pass_tree.delete(row)

        if not path:
            messagebox.showinfo("No Pass",
                                f"{name} does not rise above {MIN_ELEVATION}° "
                                f"in the next 90 minutes from your location.")
            return

        for time_str, az, el in path:
            self.pass_tree.insert("", tk.END, values=(time_str, f"{az:.1f}", f"{el:.1f}"))

        self.status_label.config(
            text=f"Pass preview for {name}: {len(path)} points above {MIN_ELEVATION}° in next 90 min",
            fg="#fab387")

    def _start_tracking(self):
        result = self._get_selected_satellite()
        if not result:
            return
        name, l1, l2 = result

        if not self.dish or not self.dish.is_open:
            messagebox.showerror("No Dish", "Connect to the dish first.")
            return

        # Stop any existing tracking
        self._stop_tracking()

        self.tracker = SatelliteTracker(name, l1, l2, OBSERVER_LAT, OBSERVER_LON)

        # Check current position
        az, el, dist = self.tracker.get_position()
        if el < MIN_ELEVATION:
            proceed = messagebox.askyesno(
                "Below Horizon",
                f"{name} is currently below {MIN_ELEVATION}° elevation (El: {el:.1f}°).\n\n"
                "The tracker will wait and begin commanding the dish when the satellite rises.\n\n"
                "Start tracking anyway?"
            )
            if not proceed:
                return

        self.tracking_thread = TrackingThread(
            self.tracker, self.dish, self._on_status_update
        )
        self.tracking_thread.start()

        self.track_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_label.config(
            text=f"Tracking {name}…", fg="#a6e3a1")

    def _stop_tracking(self):
        if self.tracking_thread and self.tracking_thread.running:
            self.tracking_thread.stop()
            self.tracking_thread = None
        self.track_btn.config(state="normal" if (self.dish and self.dish.is_open and self.satellites) else "disabled")
        self.stop_btn.config(state="disabled")

    def _on_status_update(self, message, below_horizon=False):
        color = "#f9e2af" if below_horizon else "#a6e3a1"
        self.after(0, lambda: self.status_label.config(text=message, fg=color))

    def on_closing(self):
        self._stop_tracking()
        if self.dish and self.dish.is_open:
            self.dish.close()
        self.destroy()


# ─────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = TrackerApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
