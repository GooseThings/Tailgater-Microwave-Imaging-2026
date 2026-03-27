# dish_track.py
# Real-time satellite tracker for the Tailgater dish antenna
# dish_track.py authored by Goose N8GMZ | http://github.com/GooseThings/
#
# Downloads TLE data from Celestrak, lets you pick a satellite from a GUI, then drives the dish motors over serial to follow the satellite in real time.
#
# Requirements: pip install skyfield pyserial pillow requests tkinter
#
# Usage: python3 dish_track.py
import serial
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import requests

from skyfield.api import load, EarthSatellite, wgs84

# ─────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

OBSERVER_LAT = 42.87
OBSERVER_LON = -85.68

MIN_ELEVATION = 5
MAX_ELEVATION = 70

UPDATE_INTERVAL = 2.0

# Movement threshold (degrees)
AZ_THRESHOLD = 1.0
EL_THRESHOLD = 1.0

CELESTRAK_SOURCES = {
    "Active Satellites": "https://celestrak.org/SOCRATES/query.php?CATALOG=active&FORMAT=TLE",
}

# ─────────────────────────────────────────────────────────────────
# SERIAL HELPERS
# ─────────────────────────────────────────────────────────────────

def send_command(dish, text):
    """Safer command send"""
    cmd = (text + '\r').encode()
    dish.write(cmd)
    dish.flush()


def send_azangle(dish, degrees):
    send_command(dish, f"azangle {int(degrees)}")


def send_elangle(dish, degrees):
    degrees = max(MIN_ELEVATION, min(MAX_ELEVATION, int(degrees)))
    send_command(dish, f"elangle {int(degrees)}")


def dish_az_from_compass(compass_az):
    return (360 - compass_az) % 360


# ─────────────────────────────────────────────────────────────────
# TLE
# ─────────────────────────────────────────────────────────────────

def fetch_tle_catalogue(url):
    resp = requests.get(url, timeout=15)
    resp.raise_for_status()

    lines = [l.strip() for l in resp.text.splitlines() if l.strip()]
    sats = []

    i = 0
    while i + 2 < len(lines):
        name, l1, l2 = lines[i:i+3]
        if l1.startswith('1 ') and l2.startswith('2 '):
            sats.append((name, l1, l2))
            i += 3
        else:
            i += 1

    return sats


# ─────────────────────────────────────────────────────────────────
# TRACKER
# ─────────────────────────────────────────────────────────────────

class SatelliteTracker:
    def __init__(self, name, l1, l2, lat, lon):
        self.ts = load.timescale()
        self.observer = wgs84.latlon(lat, lon)
        self.satellite = EarthSatellite(l1, l2, name, self.ts)
        self.diff = self.satellite - self.observer  # precompute

    def get_position(self):
        t = self.ts.now()
        top = self.diff.at(t)
        alt, az, dist = top.altaz()
        return az.degrees, alt.degrees, dist.km


# ─────────────────────────────────────────────────────────────────
# THREAD
# ─────────────────────────────────────────────────────────────────

class TrackingThread(threading.Thread):
    def __init__(self, tracker, dish, callback):
        super().__init__(daemon=True)
        self.tracker = tracker
        self.dish = dish
        self.callback = callback
        self.stop_event = threading.Event()

        self.prev_az = None
        self.prev_el = None

    def run(self):
        while not self.stop_event.is_set():
            try:
                az, el, dist = self.tracker.get_position()
                dish_az = dish_az_from_compass(az)

                if el < MIN_ELEVATION:
                    self.callback(f"[BELOW HORIZON] El: {el:.1f}°", True)
                else:
                    move = False

                    if self.prev_az is None:
                        move = True
                    else:
                        if abs(self.prev_az - dish_az) > AZ_THRESHOLD:
                            move = True
                        if abs(self.prev_el - el) > EL_THRESHOLD:
                            move = True

                    if move:
                        send_azangle(self.dish, dish_az)
                        send_elangle(self.dish, el)
                        self.prev_az = dish_az
                        self.prev_el = el

                    self.callback(
                        f"Az: {az:.1f}° | El: {el:.1f}° | Dist: {dist:.0f} km",
                        False
                    )

            except serial.SerialException:
                self.callback("Serial disconnected — stopping", False)
                break
            except Exception as e:
                self.callback(f"Error: {e}", False)

            self.stop_event.wait(UPDATE_INTERVAL)

    def stop(self):
        self.stop_event.set()


# ─────────────────────────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────────────────────────

class TrackerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Satellite Tracker")

        self.satellites = []
        self.dish = None
        self.thread = None

        self._build()

    def _build(self):
        self.listbox = tk.Listbox(self)
        self.listbox.pack(fill="both", expand=True)

        tk.Button(self, text="Load", command=self.load).pack()
        tk.Button(self, text="Connect", command=self.connect).pack()
        tk.Button(self, text="Track", command=self.track).pack()
        tk.Button(self, text="Stop", command=self.stop).pack()

        self.status = tk.Label(self, text="Ready")
        self.status.pack()

    def load(self):
        sats = fetch_tle_catalogue(list(CELESTRAK_SOURCES.values())[0])
        self.satellites = sats

        self.listbox.delete(0, tk.END)
        for i, (name, _, _) in enumerate(sats):
            self.listbox.insert(tk.END, f"{i}: {name}")

    def connect(self):
        self.dish = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.status.config(text="Connected")

    def track(self):
        sel = self.listbox.curselection()
        if not sel:
            return

        idx = sel[0]
        name, l1, l2 = self.satellites[idx]

        tracker = SatelliteTracker(name, l1, l2, OBSERVER_LAT, OBSERVER_LON)

        self.thread = TrackingThread(
            tracker,
            self.dish,
            lambda msg, _: self.after(0, lambda: self.status.config(text=msg))
        )
        self.thread.start()

    def stop(self):
        if self.thread:
            self.thread.stop()
            self.thread = None

    def on_close(self):
        self.stop()
        if self.dish:
            self.dish.close()
        self.destroy()


# ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = TrackerApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
