#!/usr/bin/env python3
"""
csv_to_plan.py - convert a GNSS track CSV into a QGroundControl .plan file.

Usage:
    python3 csv_to_plan.py                      # search for CSVs interactively
    python3 csv_to_plan.py track.csv            # use a given CSV
    python3 csv_to_plan.py track.csv -o my_mission
    python3 csv_to_plan.py --dir ~/logs         # search a specific directory

Requires simplify.py, rm_outliers.py, utm.py and transverse_mercator.py in the same folder.
"""

import argparse
import csv
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path

try:
    from simplify import TrackSimplifier
    from rm_outliers import OutlierRemover
except ImportError as exc:
    sys.exit(f"Cannot import a helper module ({exc}). Keep simplify.py, rm_outliers.py, "
             "utm.py and transverse_mercator.py in the same folder as this script.")

# =============================================================================
# CONFIG A - PROGRAM BEHAVIOUR (how the script works)
# =============================================================================
DEFAULT_CSV_DIR = r"C:\\Users\\aless\\shared_drone\\drone_gps_log_and_antenna_daniele\\logs"  # None -> current working directory
OUTPUT_DIR = r"C:\\Users\\aless\\Documents\\QGroundControl\\Missions"               # None -> same folder as the input CSV
OUTPUT_NAME_MODE = "csv"        # "csv" (CSV file name) | "custom" | "timestamp"
CUSTOM_OUTPUT_NAME = "mission"  # used when OUTPUT_NAME_MODE == "custom"
TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"
OVERWRITE_POLICY = "ask"        # "ask" | "overwrite" | "increment" (name_1, name_2, ...)
INTERACTIVE = True              # False -> never prompt, fail with an error instead

# CSV column names. Change the values to match your file; the keys are fixed.
COLUMNS = {
    "lat": "lat_wgs84_deg",
    "lon": "lon_wgs84_deg",
    "alt": "alt_wgs84_m",       # only needed if ALTITUDE_SOURCE == "csv_relative"
    "time": "timestamp_s",      # [s], only needed if REMOVE_OUTLIERS is True
}

REMOVE_OUTLIERS = True          # speed-based outlier removal before simplification
OUTLIER_MAX_REJECTS = 5         # after this many rejections in a row, accept the next point

SIMPLIFY_METHOD = "rdp"         # "none" | "rdp" | "vw"
RDP_EPSILON_M = 1.5             # RDP: max deviation from the track [m]
VW_MAX_WAYPOINTS = 15           # VW: max waypoints (None -> use VW_MIN_AREA_M2)
VW_MIN_AREA_M2 = 2.0

# =============================================================================
# CONFIG B - VEHICLE / HARDWARE / ENVIRONMENT
# =============================================================================
TRACK_MAX_SPEED_MS = 1.5        # max realistic speed while the track was recorded (walking pace)
VEHICLE_TYPE = "multirotor"     # "multirotor" | "fixedwing"
FIRMWARE = "px4"                # "px4" | "ardupilot"
ALTITUDE_SOURCE = "fixed"       # "fixed" | "csv_relative" (csv alt - first alt + MISSION_ALTITUDE_M)
MISSION_ALTITUDE_M = 20.0       # altitude above the takeoff point [m]
CRUISE_SPEED_MS = 5.0
HOVER_SPEED_MS = 5.0            # used by multirotors
FW_TAKEOFF_PITCH_DEG = 15.0     # fixed wing takeoff pitch
ADD_TAKEOFF = True              # first track point becomes a takeoff item
ADD_RTL = True                  # return-to-launch item at the end
HOME_LAT = None                 # None -> first waypoint is used as planned home
HOME_LON = None                 # (set both or neither)

# =============================================================================
# MAVLink / QGC constants (plain numbers, no pymavlink needed)
# =============================================================================
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_TAKEOFF = 22
MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
QGC_ALT_MODE_RELATIVE = 1
FIRMWARE_IDS = {"px4": 12, "ardupilot": 3}          # MAV_AUTOPILOT_*
VEHICLE_IDS = {"fixedwing": 1, "multirotor": 2}     # MAV_TYPE_*


class UserAbort(Exception):
    """Raised when the user types 'q' at a prompt."""


def check_config() -> None:
    """Stop early with a clear message if a config value is not valid."""
    options = {
        "OUTPUT_NAME_MODE": (OUTPUT_NAME_MODE, ("csv", "custom", "timestamp")),
        "OVERWRITE_POLICY": (OVERWRITE_POLICY, ("ask", "overwrite", "increment")),
        "SIMPLIFY_METHOD": (SIMPLIFY_METHOD, ("none", "rdp", "vw")),
        "VEHICLE_TYPE": (VEHICLE_TYPE, tuple(VEHICLE_IDS)),
        "FIRMWARE": (FIRMWARE, tuple(FIRMWARE_IDS)),
        "ALTITUDE_SOURCE": (ALTITUDE_SOURCE, ("fixed", "csv_relative")),
    }
    for name, (value, allowed) in options.items():
        if value not in allowed:
            sys.exit(f"Config error: {name} = {value!r}, expected one of {allowed}.")
    if (HOME_LAT is None) != (HOME_LON is None):
        sys.exit("Config error: set both HOME_LAT and HOME_LON, or neither.")
    missing_keys = {"lat", "lon", "alt", "time"} - set(COLUMNS)
    if missing_keys:
        sys.exit(f"Config error: COLUMNS is missing the key(s) {sorted(missing_keys)}.")
    if REMOVE_OUTLIERS and TRACK_MAX_SPEED_MS <= 0:
        sys.exit("Config error: TRACK_MAX_SPEED_MS must be positive.")


# =============================================================================
# CSV SELECTION (runs once: CLI argument first, then interactive search)
# =============================================================================


def select_csv(cli_path: str | None, cli_dir: str | None) -> Path:
    """Return a validated CSV path.

    1. If a path was given on the command line, validate it.
    2. If there is none (or it is invalid), list the CSVs in the start directory
       (--dir, then DEFAULT_CSV_DIR, then the current directory) and let the
       user pick one, search another directory ('e') or quit ('q').
    """
    required = [COLUMNS["lat"], COLUMNS["lon"]]
    if ALTITUDE_SOURCE == "csv_relative":
        required.append(COLUMNS["alt"])
    if REMOVE_OUTLIERS:
        required.append(COLUMNS["time"])

    directory = Path(cli_dir or DEFAULT_CSV_DIR or Path.cwd()).expanduser()
    if not directory.is_dir():
        print(f"Directory '{directory}' is not valid, using the current one.")
        directory = Path.cwd()
    directory = directory.resolve()
    candidate = Path(cli_path).expanduser() if cli_path else None

    while True:
        # --- 1. no candidate yet: choose one interactively -------------------
        if candidate is None:
            if not INTERACTIVE:
                sys.exit("No valid CSV given and interactive mode is disabled.")
            try:
                csvs = sorted(p for p in directory.iterdir()
                              if p.is_file() and p.suffix.lower() == ".csv")
            except OSError as exc:
                print(f"Cannot read '{directory}': {exc}")
                csvs = []

            print(f"\nCSV files in {directory}:")
            for i, p in enumerate(csvs, 1):
                print(f"  [{i}] {p.name}")
            if not csvs:
                print("  (none found)")

            choice = input("Number to use, 'e' for another directory, 'q' to quit: ").strip().lower()
            if choice == "q":
                raise UserAbort
            if choice == "e":
                text = input("Directory to search: ").strip()
                if text and Path(text).expanduser().is_dir():
                    directory = Path(text).expanduser().resolve()
                else:
                    print(f"  '{text}' is not a valid directory.")
                continue
            if choice.isdigit() and 1 <= int(choice) <= len(csvs):
                candidate = csvs[int(choice) - 1]
            else:
                print("  Invalid choice.")
                continue

        # --- 2. validate the candidate ---------------------------------------
        problem = None
        if not candidate.is_file():
            problem = "does not exist or is not a file"
        elif candidate.suffix.lower() != ".csv":
            problem = "is not a .csv file"
        else:
            try:
                with open(candidate, newline="", encoding="utf-8") as f:
                    header = csv.DictReader(f).fieldnames or []
                missing = [c for c in required if c not in header]
                if missing:
                    problem = f"is missing column(s) {missing} (found: {header})"
            except (OSError, UnicodeDecodeError) as exc:
                problem = f"cannot be read ({exc})"

        if problem is None:
            return candidate.resolve()

        print(f"'{candidate}' {problem}.")
        if not INTERACTIVE:
            sys.exit(1)
        candidate = None  # fall back to the interactive search


# =============================================================================
# OUTPUT NAMING
# =============================================================================


def sanitize_name(name: str) -> str:
    """Keep letters, digits, dash, underscore and dot; replace the rest."""
    name = re.sub(r"[^\w.\-]", "_", name.strip())
    return name or "mission"


def build_output_path(csv_path: Path, cli_name: str | None) -> Path:
    """Compose the output .plan path from config, CSV and optional CLI name."""
    if cli_name:
        base = Path(cli_name).stem if cli_name.endswith(".plan") else cli_name
    elif OUTPUT_NAME_MODE == "custom":
        base = CUSTOM_OUTPUT_NAME
    elif OUTPUT_NAME_MODE == "timestamp":
        base = datetime.now().strftime(TIMESTAMP_FORMAT)
    else:
        base = csv_path.stem
    base = sanitize_name(base)

    out_dir = Path(OUTPUT_DIR).expanduser() if OUTPUT_DIR else csv_path.parent
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f"Cannot use output directory '{out_dir}' ({exc}); using current directory.")
        out_dir = Path.cwd()

    target = out_dir / f"{base}.plan"
    if not target.exists():
        return target

    policy = OVERWRITE_POLICY
    if policy == "ask" and INTERACTIVE:
        answer = input(f"'{target.name}' exists. [o]verwrite, [i]ncrement, [q]uit: ").strip().lower()
        policy = {"o": "overwrite", "i": "increment"}.get(answer)
        if policy is None:
            raise UserAbort
    elif policy == "ask":
        policy = "increment"

    if policy == "overwrite":
        return target
    n = 1
    while (out_dir / f"{base}_{n}.plan").exists():
        n += 1
    return out_dir / f"{base}_{n}.plan"


# =============================================================================
# TRACK -> WAYPOINTS -> PLAN
# =============================================================================


def load_track(csv_path: Path) -> list[tuple[float, float, float, float]]:
    """Read (lat, lon, alt, t_s) rows using COLUMNS.

    Only the columns that are actually needed are read. Rows with a missing or
    invalid value in one of them (e.g. NaN when there was no GNSS fix) are skipped.
    """
    need_alt = ALTITUDE_SOURCE == "csv_relative"
    points, skipped = [], 0
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                lat = float(row[COLUMNS["lat"]])
                lon = float(row[COLUMNS["lon"]])
                alt = float(row[COLUMNS["alt"]]) if need_alt else 0.0
                t = float(row[COLUMNS["time"]]) if REMOVE_OUTLIERS else 0.0
            except (ValueError, TypeError):
                skipped += 1
                continue
            if any(math.isnan(v) for v in (lat, lon, alt, t)):
                skipped += 1
                continue
            points.append((lat, lon, alt, t))
    if skipped:
        print(f"Skipped {skipped} row(s) with missing or invalid values.")
    if len(points) < 2:
        raise ValueError(f"only {len(points)} valid point(s) found, at least 2 are needed.")
    return points


def simplify_track(track: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    """Reduce the track with the method set in CONFIG A, using simplify.py."""
    if SIMPLIFY_METHOD == "none":
        return track
    simplifier = TrackSimplifier(track)  # raises ValueError if < 2 points
    if SIMPLIFY_METHOD == "rdp":
        waypoints, _ = simplifier.simplify_rdp(epsilon_m=RDP_EPSILON_M)
    else:
        waypoints, _ = simplifier.simplify_visvalingam_whyatt(
            max_waypoints=VW_MAX_WAYPOINTS, min_area_m2=VW_MIN_AREA_M2)
    return waypoints


def mission_item(seq: int, command: int, frame: int, params: list, alt: float) -> dict:
    """One QGC 'SimpleItem'. seq is 1-based (doJumpId)."""
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": alt,
        "AltitudeMode": QGC_ALT_MODE_RELATIVE,
        "autoContinue": True,
        "command": command,
        "doJumpId": seq,
        "frame": frame,
        "params": params,
        "type": "SimpleItem",
    }


def build_plan(waypoints: list[tuple[float, float, float]]) -> dict:
    """Build the QGC plan dictionary from (lat, lon, alt) waypoints."""
    if ALTITUDE_SOURCE == "csv_relative":
        alt0 = waypoints[0][2]
        alts = [round(w[2] - alt0 + MISSION_ALTITUDE_M, 2) for w in waypoints]
    else:
        alts = [MISSION_ALTITUDE_M] * len(waypoints)

    items = []
    first = 0
    if ADD_TAKEOFF:
        # The takeoff item replaces the first track point (same position).
        lat, lon, _ = waypoints[0]
        pitch = FW_TAKEOFF_PITCH_DEG if VEHICLE_TYPE == "fixedwing" else 0
        items.append(mission_item(
            len(items) + 1, MAV_CMD_NAV_TAKEOFF, MAV_FRAME_GLOBAL_RELATIVE_ALT,
            [pitch, 0, 0, None, round(lat, 7), round(lon, 7), alts[0]], alts[0]))
        first = 1

    for (lat, lon, _), alt in list(zip(waypoints, alts))[first:]:
        items.append(mission_item(
            len(items) + 1, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT,
            [0, 0, 0, None, round(lat, 7), round(lon, 7), alt], alt))

    if ADD_RTL:
        items.append(mission_item(
            len(items) + 1, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_FRAME_MISSION,
            [0, 0, 0, 0, 0, 0, 0], 0))

    home_lat = HOME_LAT if HOME_LAT is not None else waypoints[0][0]
    home_lon = HOME_LON if HOME_LON is not None else waypoints[0][1]

    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": CRUISE_SPEED_MS,
            "firmwareType": FIRMWARE_IDS[FIRMWARE],
            "hoverSpeed": HOVER_SPEED_MS,
            "items": items,
            "plannedHomePosition": [round(home_lat, 7), round(home_lon, 7), 0],
            "vehicleType": VEHICLE_IDS[VEHICLE_TYPE],
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def write_plan(plan: dict, out_path: Path) -> bool:
    """Write the plan as JSON. Returns False (with a message) on failure."""
    try:
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(plan, f, indent=4)
    except OSError as exc:
        print(f"Could not write '{out_path}': {exc}")
        return False
    return True


# =============================================================================
# MAIN
# =============================================================================


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Convert a GNSS track CSV to a QGC .plan file.")
    p.add_argument("csv", nargs="?", help="path to the input CSV (optional)")
    p.add_argument("-d", "--dir", help="directory to search for CSVs")
    p.add_argument("-o", "--out", help="output name (overrides OUTPUT_NAME_MODE)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    check_config()
    try:
        csv_path = select_csv(args.csv, args.dir)
        out_path = build_output_path(csv_path, args.out)
    except (UserAbort, KeyboardInterrupt, EOFError):
        print("\nAborted.")
        return 1

    print(f"\nInput : {csv_path}")
    print(f"Output: {out_path}")

    try:
        track = load_track(csv_path)
        n_loaded = len(track)
        if REMOVE_OUTLIERS:
            track, removed = OutlierRemover(TRACK_MAX_SPEED_MS, OUTLIER_MAX_REJECTS).filter(track)
            print(f"Outliers removed: {len(removed)} of {n_loaded} points "
                  f"(speed limit {TRACK_MAX_SPEED_MS} m/s).")
        waypoints = simplify_track([p[:3] for p in track])
    except (ValueError, KeyError, OSError) as exc:
        print(f"Could not process '{csv_path.name}': {exc}")
        return 1

    plan = build_plan(waypoints)
    if not write_plan(plan, out_path):
        return 1
    print(f"Track: {n_loaded} points -> {len(track)} after outliers -> {len(waypoints)} waypoints "
          f"({len(plan['mission']['items'])} mission items written).")
    return 0


if __name__ == "__main__":
    sys.exit(main())