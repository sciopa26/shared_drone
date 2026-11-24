#!/usr/bin/env python3
"""
rm_outliers.py - speed-based outlier removal for GNSS tracks.

Library use (as in csv-to-plan.py):
    from rm_outliers import OutlierRemover
    clean, removed = OutlierRemover(max_speed_ms=1.5, max_rejects=5).filter(track)
    # track: list of (lat, lon, alt, t_s)

Standalone demo (plots raw vs filtered track):
    python rm_outliers.py track.csv
"""
import math

from utm import utmconv


class OutlierRemover:
    """Reject points that would require exceeding max_speed_ms from the last accepted point."""

    def __init__(self, max_speed_ms: float, max_rejects: int = 5):
        if max_speed_ms <= 0:
            raise ValueError("max_speed_ms must be positive.")
        self.max_speed = max_speed_ms
        self.max_rejects = max_rejects  # after this many rejections in a row, accept the next point
        self.uc = utmconv()

    def _to_utm(self, track):
        """Convert (lat, lon, ...) points to (easting, northing), forcing a single UTM zone."""
        zone = self.uc.geodetic_to_utm(track[0][0], track[0][1])[1]
        self.uc.set_zone_override(zone)
        xy = []
        for p in track:
            _, _, _, e, n = self.uc.geodetic_to_utm(p[0], p[1])
            xy.append((e, n))
        return xy

    def filter(self, track):
        """Return (clean_track, removed_points). Points keep their original tuple format."""
        if not track:
            return [], []

        xy = self._to_utm(track)
        clean = [track[0]]
        removed = []
        last_xy, last_t = xy[0], track[0][3]
        rejects_in_row = 0

        for p, (e, n) in zip(track[1:], xy[1:]):
            dt = p[3] - last_t
            speed = math.hypot(e - last_xy[0], n - last_xy[1]) / dt if dt > 0 else math.inf

            # Too many rejections in a row means the anchor itself is probably the bad point
            if speed <= self.max_speed or rejects_in_row >= self.max_rejects:
                clean.append(p)
                last_xy, last_t = (e, n), p[3]
                rejects_in_row = 0
            else:
                removed.append(p)
                rejects_in_row += 1

        return clean, removed


def _load_csv(path, lat_col="lat_wgs84_deg", lon_col="lon_wgs84_deg", time_col="timestamp_s"):
    import csv
    track = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                lat, lon, t = float(row[lat_col]), float(row[lon_col]), float(row[time_col])
            except (ValueError, KeyError, TypeError):
                continue
            if not any(math.isnan(v) for v in (lat, lon, t)):
                track.append((lat, lon, 0.0, t))
    return track


if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt

    ap = argparse.ArgumentParser(description="Plot raw vs outlier-filtered GNSS track.")
    ap.add_argument("csv", help="input CSV file")
    ap.add_argument("--speed", type=float, default=1.5, help="max speed [m/s] (default 1.5)")
    ap.add_argument("--rejects", type=int, default=5, help="max consecutive rejects (default 5)")
    args = ap.parse_args()

    raw = _load_csv(args.csv)
    remover = OutlierRemover(args.speed, args.rejects)
    clean, removed = remover.filter(raw)
    print(f"Outliers detected: {len(removed)} of {len(raw)} points")

    def rel_xy(points, ref):
        xy = remover._to_utm(points) if points else []
        return [x - ref[0] for x, _ in xy], [y - ref[1] for _, y in xy]

    ref = remover._to_utm(raw[:1])[0]
    re_, rn_ = rel_xy(raw, ref)
    ce, cn = rel_xy(clean, ref)
    oe, on = rel_xy(removed, ref)

    plt.axis("equal")
    plt.plot(re_, rn_, "r--", linewidth=0.8, alpha=0.5, label="Raw trajectory")
    plt.plot(oe, on, "rx", markersize=6, label="Outliers")
    plt.plot(ce, cn, "b.-", markersize=3, linewidth=1.2, label="Filtered trajectory")
    plt.plot(ce[0], cn[0], "go", markersize=8, label="Start")
    plt.plot(ce[-1], cn[-1], "ko", markersize=6, label="End")
    plt.xlabel("Easting [m]")
    plt.ylabel("Northing [m]")
    plt.legend()
    plt.grid()
    plt.show()