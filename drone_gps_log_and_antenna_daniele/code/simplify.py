#!/usr/bin/env python3
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from utm import utmconv

# Internal configuration parameters
CSV_PATH = '/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/logs/filt_good.csv'
SIMPLIFICATION_METHOD = 'both'  # Options: 'rdp', 'vw', 'both'

# Algorithm thresholds
RDP_EPSILON_M = 1.5             # Max perpendicular distance deviation [m] (criterion 2)
VW_MAX_WAYPOINTS = 15           # Max waypoints allowed (criterion 1); set None to use area threshold
VW_MIN_AREA_M2 = 2.0            # Min triangle area [m^2] when VW_MAX_WAYPOINTS is None


def load_track_from_csv(file_path: str) -> list[tuple[float, float, float]]:
    """Load valid (lat, lon, alt) points from filtered or raw telemetry CSV."""
    points = []
    with open(file_path, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lat = float(row['lat_wgs84_deg'])
            lon = float(row['lon_wgs84_deg'])
            alt = float(row['alt_wgs84_m'])
            if not (math.isnan(lat) or math.isnan(lon)):
                points.append((lat, lon, 0.0 if math.isnan(alt) else alt))
    return points


class TrackSimplifier:
    """Simplify a GNSS track into route waypoints (lat, lon, alt) in 2D UTM space."""

    def __init__(self, geo_points: list[tuple[float, float, float]]):
        if len(geo_points) < 2:
            raise ValueError('At least 2 valid GNSS points are required.')

        self.geo_points = geo_points
        self.uc = utmconv()
        self.utm_points = []
        self.zone_info = None  # (hemisphere, zone)

        for lat, lon, alt in geo_points:
            hemisphere, zone, _, easting, northing = self.uc.geodetic_to_utm(lat, lon)
            if self.zone_info is None:
                self.zone_info = (hemisphere, zone)
            self.utm_points.append((easting, northing, alt))

    @staticmethod
    def _perpendicular_distance(pt: tuple, seg_start: tuple, seg_end: tuple) -> float:
        """Compute 2D Euclidean distance from pt to segment [seg_start, seg_end]."""
        px, py = pt[0], pt[1]
        x1, y1 = seg_start[0], seg_start[1]
        x2, y2 = seg_end[0], seg_end[1]

        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq == 0.0:
            return math.hypot(px - x1, py - y1)

        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    @staticmethod
    def _triangle_area(p1: tuple, p2: tuple, p3: tuple) -> float:
        """Compute 2D area of the triangle formed by three UTM points."""
        return 0.5 * abs(
            p1[0] * (p2[1] - p3[1]) +
            p2[0] * (p3[1] - p1[1]) +
            p3[0] * (p1[1] - p2[1])
        )

    def _rdp_indices(self, start_idx: int, end_idx: int, epsilon: float) -> list[int]:
        """Recursive Ramer-Douglas-Peucker index selection."""
        max_dist = -1.0
        max_idx = -1

        for i in range(start_idx + 1, end_idx):
            dist = self._perpendicular_distance(
                self.utm_points[i], self.utm_points[start_idx], self.utm_points[end_idx]
            )
            if dist > max_dist:
                max_dist = dist
                max_idx = i

        if max_dist > epsilon and max_idx != -1:
            left = self._rdp_indices(start_idx, max_idx, epsilon)
            right = self._rdp_indices(max_idx, end_idx, epsilon)
            return left[:-1] + right
        return [start_idx, end_idx]

    def simplify_rdp(self, epsilon_m: float = 1.5) -> tuple[list[tuple[float, float, float]], list[int]]:
        """Simplify track by bounding maximum perpendicular distance deviation."""
        indices = self._rdp_indices(0, len(self.utm_points) - 1, epsilon_m)
        return self._indices_to_waypoints(indices), indices

    def simplify_visvalingam_whyatt(
        self, max_waypoints: int | None = None, min_area_m2: float = 2.0
    ) -> tuple[list[tuple[float, float, float]], list[int]]:
        """Simplify track by iteratively removing points forming the smallest triangle area."""
        active = list(range(len(self.utm_points)))

        while len(active) > 2:
            if max_waypoints is not None and len(active) <= max_waypoints:
                break

            min_area = float('inf')
            min_pos = -1

            for pos in range(1, len(active) - 1):
                area = self._triangle_area(
                    self.utm_points[active[pos - 1]],
                    self.utm_points[active[pos]],
                    self.utm_points[active[pos + 1]],
                )
                if area < min_area:
                    min_area = area
                    min_pos = pos

            if max_waypoints is None and min_area >= min_area_m2:
                break

            active.pop(min_pos)

        return self._indices_to_waypoints(active), active

    def _indices_to_waypoints(self, indices: list[int]) -> list[tuple[float, float, float]]:
        """Convert selected UTM indices back to geographic (lat, lon, alt) waypoints."""
        hemisphere, zone = self.zone_info
        waypoints = []
        for idx in indices:
            easting, northing, alt = self.utm_points[idx]
            lat, lon = self.uc.utm_to_geodetic(hemisphere, zone, easting, northing)
            waypoints.append((lat, lon, alt))
        return waypoints


if __name__ == '__main__':
    raw_geo_track = load_track_from_csv(CSV_PATH)
    simplifier = TrackSimplifier(raw_geo_track)

    utm_arr = np.array(simplifier.utm_points)
    e0, n0 = utm_arr[0, 0], utm_arr[0, 1]
    e_rel = utm_arr[:, 0] - e0
    n_rel = utm_arr[:, 1] - n0

    plt.figure(figsize=(9, 7))
    plt.plot(e_rel, n_rel, color='lightgray', linewidth=1.5, label=f'Original track ({len(utm_arr)} pts)')
    plt.scatter(e_rel, n_rel, color='gray', s=8, alpha=0.6)

    method = SIMPLIFICATION_METHOD.lower()

    if method in ('rdp', 'both'):
        rdp_wps, rdp_idx = simplifier.simplify_rdp(epsilon_m=RDP_EPSILON_M)
        print(f'RDP Waypoints ({len(rdp_wps)}):')
        for wp in rdp_wps:
            print(f'  Lat: {wp[0]:.7f}, Lon: {wp[1]:.7f}, Alt: {wp[2]:.2f} m')
        plt.plot(
            e_rel[rdp_idx], n_rel[rdp_idx], '-o',
            color='tab:blue', markersize=6, label=f'RDP (eps={RDP_EPSILON_M}m -> {len(rdp_wps)} wps)'
        )

    if method in ('vw', 'both'):
        vw_wps, vw_idx = simplifier.simplify_visvalingam_whyatt(
            max_waypoints=VW_MAX_WAYPOINTS, min_area_m2=VW_MIN_AREA_M2
        )
        print(f'Visvalingam-Whyatt Waypoints ({len(vw_wps)}):')
        for wp in vw_wps:
            print(f'  Lat: {wp[0]:.7f}, Lon: {wp[1]:.7f}, Alt: {wp[2]:.2f} m')
        plt.plot(
            e_rel[vw_idx], n_rel[vw_idx], '--s',
            color='tab:red', markersize=6, label=f'Visvalingam-Whyatt ({len(vw_wps)} wps)'
        )

    plt.axis('equal')
    plt.xlabel('Easting [m]')
    plt.ylabel('Northing [m]')
    plt.title('Track Simplification Comparison (UTM 2D)')
    plt.legend()
    plt.grid(True)
    plt.show()