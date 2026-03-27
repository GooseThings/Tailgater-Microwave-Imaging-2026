# dish_image.py
# Tailgater scan visualizer with interpolation

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Optional: scipy for high-quality interpolation
try:
    from scipy.ndimage import zoom
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


# ────────────────────────────────────────────────────────────────
# SETTINGS
# ────────────────────────────────────────────────────────────────

INTERPOLATION_FACTOR = 4   # Increase for smoother image (e.g. 2–8)


# ────────────────────────────────────────────────────────────────
# INPUT VALIDATION
# ────────────────────────────────────────────────────────────────

if len(sys.argv) < 2:
    print("Usage: python dish_image_interp.py <datafile>")
    sys.exit(1)

filepath = sys.argv[1]
basename = os.path.basename(filepath)

print("Loading data file...")
sky_data = np.loadtxt(filepath)

# ────────────────────────────────────────────────────────────────
# PARSE TIMESTAMP FROM FILENAME
# ────────────────────────────────────────────────────────────────

parts = basename.split('-')
if len(parts) < 3:
    raise ValueError("Unexpected filename format")

timestamp = parts[1] + '-' + parts[2].split('.')[0]

# ────────────────────────────────────────────────────────────────
# LOAD SCAN PARAMETERS
# ────────────────────────────────────────────────────────────────

print("Loading scan parameters...")
settings_file = f"scan-settings-{timestamp}.txt"

try:
    scan_params = np.loadtxt(settings_file)
except OSError:
    raise FileNotFoundError(f"Missing scan settings file: {settings_file}")

az_start, az_end, el_start, el_end, resolution = map(int, scan_params)

# ────────────────────────────────────────────────────────────────
# CLEAN DATA
# ────────────────────────────────────────────────────────────────

if sky_data.shape[0] < 2 or sky_data.shape[1] < 2:
    raise ValueError("Data too small to process")

cleaned_data = sky_data[1:, 1:]

# Replace NaNs (if any)
cleaned_data = np.nan_to_num(cleaned_data, nan=0.0)

# ────────────────────────────────────────────────────────────────
# INTERPOLATION
# ────────────────────────────────────────────────────────────────

print("Applying interpolation...")

if INTERPOLATION_FACTOR > 1:
    if SCIPY_AVAILABLE:
        # High-quality interpolation
        interpolated_data = zoom(
            cleaned_data,
            INTERPOLATION_FACTOR,
            order=3  # cubic interpolation
        )
    else:
        print("WARNING: scipy not installed, using basic interpolation")
        interpolated_data = np.repeat(
            np.repeat(cleaned_data, INTERPOLATION_FACTOR, axis=0),
            INTERPOLATION_FACTOR, axis=1
        )
else:
    interpolated_data = cleaned_data


# ────────────────────────────────────────────────────────────────
# AXIS SETUP
# ────────────────────────────────────────────────────────────────

az_range_total = az_end - az_start
el_range_total = el_end - el_start

scale = INTERPOLATION_FACTOR

if resolution == 1:
    x = [0,
         (az_range_total - 1) * scale // 2,
         (az_range_total - 2) * scale]

    az_range = [az_end, (az_start + az_end) // 2, az_start]

    y = [0,
         (el_range_total - 1) * scale // 2,
         (el_range_total - 1) * scale]

    el_range = [el_end, (el_start + el_end) // 2, el_start]

elif resolution == 2:
    AZ_SCALE = 5
    EL_SCALE = 3

    x = [0,
         ((az_range_total * AZ_SCALE) - 1) * scale // 2,
         ((az_range_total * AZ_SCALE) - 2) * scale]

    az_range = [az_end, (az_start + az_end) // 2, az_start]

    y = [0,
         ((el_range_total * EL_SCALE) - 1) * scale // 2,
         ((el_range_total * EL_SCALE)) * scale]

    el_range = [el_end, (el_start + el_end) // 2, el_start]

else:
    raise ValueError(f"Unsupported resolution: {resolution}")

plt.xticks(x, az_range)
plt.yticks(y, el_range)

# ────────────────────────────────────────────────────────────────
# PLOT
# ────────────────────────────────────────────────────────────────

print("Rendering heatmap...")

plt.imshow(
    interpolated_data,
    cmap='inferno',
    origin='lower',
    aspect='auto'
)

plt.colorbar(location='bottom', label='RF Signal Strength')
plt.xlabel("Azimuth (dish uses CCW heading)")
plt.ylabel("Elevation")
plt.title("Ku Band Scan " + timestamp)

plt.tight_layout()
plt.show()
