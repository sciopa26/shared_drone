import csv
from typing import Optional
import numpy as np
import matplotlib.pyplot as plt


def generate_hysteresis_trajectory(
    e_min: float,
    e_max: float,
    n_min: float,
    n_max: float,
    max_width: float,
    rotation_deg: float = 0.0,
    num_points: int = 2000,
    saturation_sharpness: float = 3.0,
    uniform_spacing: bool = True,
    close_loop: bool = True,
    add_noise: bool = False,
    mu_e: float = 0.0,
    var_e: float = 0.0,
    mu_n: float = 0.0,
    var_n: float = 0.0,
    seed: Optional[int] = None,
) -> np.ndarray:
    """
    Generate an (N, 2) array of (e, n) waypoints forming an iron-like hysteresis loop,
    with optional anisotropic Gaussian noise along the East and North axes.

    Parameters
    ----------
    e_min, e_max : float
        Minimum and maximum East coordinates of the unrotated loop bounding box.
    n_min, n_max : float
        Minimum and maximum North coordinates of the unrotated loop bounding box.
    max_width : float
        Maximum horizontal opening width of the unrotated hysteresis loop (at mid-height).
    rotation_deg : float, optional
        Counter-clockwise rotation angle in degrees around the loop's origin (center).
    num_points : int, optional
        Total number of (e, n) waypoints along the closed trajectory.
    saturation_sharpness : float, optional
        Controls how flat the saturation tails are (higher values = sharper knees).
    uniform_spacing : bool, optional
        If True, resamples waypoints at equal arc-length intervals along the loop.
    close_loop : bool, optional
        If True, repeats the starting point at the end to close the nominal trajectory.
    add_noise : bool, optional
        If True, adds independent Gaussian noise to the East and North coordinates.
    mu_e : float, optional
        Mean of the Gaussian noise added along the East direction.
    var_e : float, optional
        Variance (s_e >= 0) of the Gaussian noise added along the East direction.
    mu_n : float, optional
        Mean of the Gaussian noise added along the North direction.
    var_n : float, optional
        Variance (s_n >= 0) of the Gaussian noise added along the North direction.
    seed : int or None, optional
        Random seed for reproducible noise generation.

    Returns
    -------
    np.ndarray
        Array of shape (num_points, 2) containing [e, n] coordinate pairs.
    """
    if e_max <= e_min or n_max <= n_min:
        raise ValueError("e_max and n_max must be strictly greater than e_min and n_min.")
    if num_points < 10:
        raise ValueError("num_points must be at least 10.")
    if var_e < 0.0 or var_n < 0.0:
        raise ValueError("Variances var_e and var_n must be non-negative.")

    # Compute center (loop origin) and half-spans
    e_center = 0.5 * (e_min + e_max)
    n_center = 0.5 * (n_min + n_max)
    half_e = 0.5 * (e_max - e_min)
    half_n = 0.5 * (n_max - n_min)

    # Clamp max_width to a physically valid fraction of the full horizontal span
    max_allowed_width = 0.75 * (e_max - e_min)
    target_width = float(np.clip(max_width, 0.0, max_allowed_width))

    k = saturation_sharpness

    def normalized_branch(s: np.ndarray, shift: float) -> np.ndarray:
        """Evaluate normalized [-1, 1] S-curve pinned exactly at both endpoints."""
        raw = np.tanh(k * (s - shift))
        low = np.tanh(k * (-1.0 - shift))
        high = np.tanh(k * (1.0 - shift))
        return 2.0 * (raw - low) / (high - low) - 1.0

    def loop_width_for_shift(delta: float) -> float:
        """Compute loop width at dn = 0 for a given coercivity shift delta."""
        if delta <= 0.0:
            return 0.0
        low = np.tanh(k * (-1.0 - delta))
        high = np.tanh(k * (1.0 - delta))
        mid_tanh = 0.5 * (low + high)
        s_zero = delta + np.arctanh(mid_tanh) / k
        return 2.0 * half_e * s_zero

    # Solve for the coercivity shift 'delta' via bisection to match target_width
    delta_low, delta_high = 0.0, 0.95
    for _ in range(50):
        delta_mid = 0.5 * (delta_low + delta_high)
        if loop_width_for_shift(delta_mid) < target_width:
            delta_low = delta_mid
        else:
            delta_high = delta_mid
    delta = 0.5 * (delta_low + delta_high)

    # Oversample densely first so arc-length resampling is accurate
    dense_half = max(500, num_points * 4)
    s_asc = np.linspace(-1.0, 1.0, dense_half)
    s_desc = np.linspace(1.0, -1.0, dense_half)

    # Ascending branch (bottom-left saturation -> top-right saturation)
    de_asc = half_e * s_asc
    dn_asc = half_n * normalized_branch(s_asc, shift=+delta)

    # Descending branch (top-right saturation -> bottom-left saturation)
    de_desc = half_e * s_desc
    dn_desc = half_n * normalized_branch(s_desc, shift=-delta)

    # Concatenate both branches into a single closed curve (avoid duplicate tip)
    de = np.concatenate([de_asc, de_desc[1:]])
    dn = np.concatenate([dn_asc, dn_desc[1:]])

    # Resample points uniformly along the trajectory arc length
    if uniform_spacing:
        segment_lengths = np.hypot(np.diff(de), np.diff(dn))
        cumulative_length = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        total_length = cumulative_length[-1]

        sample_count = num_points if close_loop else num_points + 1
        target_distances = np.linspace(0.0, total_length, sample_count)
        if not close_loop:
            target_distances = target_distances[:-1]

        de = np.interp(target_distances, cumulative_length, de)
        dn = np.interp(target_distances, cumulative_length, dn)
    else:
        indices = np.linspace(0, len(de) - 1, num_points, dtype=int)
        de = de[indices]
        dn = dn[indices]
        if close_loop:
            de[-1], dn[-1] = de[0], dn[0]

    # Rotate around the origin (center) of the hysteresis loop
    theta = np.deg2rad(rotation_deg)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    e_rot = e_center + de * cos_t - dn * sin_t
    n_rot = n_center + de * sin_t + dn * cos_t

    # Add independent Gaussian noise along East and North (std = sqrt(variance))
    if add_noise:
        rng = np.random.default_rng(seed)
        e_rot = e_rot + rng.normal(loc=mu_e, scale=np.sqrt(var_e), size=e_rot.shape)
        n_rot = n_rot + rng.normal(loc=mu_n, scale=np.sqrt(var_n), size=n_rot.shape)

    return np.column_stack((e_rot, n_rot))


def csv_export_trajectory(waypoints: np.ndarray, filename: str) -> None:
    """
    Export the generated trajectory waypoints to a CSV file.

    Parameters
    ----------
    waypoints : np.ndarray
        Array of shape (N, 2) containing [e, n] coordinate pairs.
    filename : str
        Path to the output CSV file.
    """
    with open(filename, mode="w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Easting (m)", "Northing (m)"])
        writer.writerows(waypoints)


if __name__ == "__main__":
    # Example parameters
    E_MIN, E_MAX = -200.0, 200.0
    N_MIN, N_MAX = -150.0, 150.0
    MAX_WIDTH = 80.0
    ROTATION_DEG = 20.0

    # Gaussian noise parameters (mean mu and variance s for East and North)
    ADD_NOISE = True
    MU_E, VAR_E = 0.0, 9.0    # East: mean = 0 m, variance = 9 m^2 (std = 3 m)
    MU_N, VAR_N = 1.5, 25.0   # North: mean = 1.5 m, variance = 25 m^2 (std = 5 m)
    POINTS_NUM = 2000
    # Generate clean reference trajectory (for visualization comparison)
    clean_waypoints = generate_hysteresis_trajectory(
        e_min=E_MIN,
        e_max=E_MAX,
        n_min=N_MIN,
        n_max=N_MAX,
        max_width=MAX_WIDTH,
        rotation_deg=ROTATION_DEG,
        num_points=POINTS_NUM,
        saturation_sharpness=3.2,
        uniform_spacing=True,
        close_loop=True,
        add_noise=False,
    )

    # Generate noisy trajectory
    waypoints = generate_hysteresis_trajectory(
        e_min=E_MIN,
        e_max=E_MAX,
        n_min=N_MIN,
        n_max=N_MAX,
        max_width=MAX_WIDTH,
        rotation_deg=ROTATION_DEG,
        num_points=POINTS_NUM,
        saturation_sharpness=3.2,
        uniform_spacing=True,
        close_loop=True,
        add_noise=ADD_NOISE,
        mu_e=MU_E,
        var_e=VAR_E,
        mu_n=MU_N,
        var_n=VAR_N,
        seed=42,
    )

    print(f"Generated {waypoints.shape[0]} [e, n] pairs:")
    print(waypoints[:5])

    csv_export_trajectory(
        waypoints,
        "/home/daniele/Documents/introduction_to_drone_technology/Module 7 - UAS positioning and navigation/hysteresis_trajectory.csv",
    )

    # Visualize the generated trajectory
    plt.figure(figsize=(7, 7))
    plt.plot(
        clean_waypoints[:, 0],
        clean_waypoints[:, 1],
        "--",
        color="gray",
        alpha=0.6,
        label="Nominal Trajectory",
    )
    plt.plot(waypoints[:, 0], waypoints[:, 1], "-o", markersize=3, label="Noisy Trajectory")
    plt.scatter(waypoints[0, 0], waypoints[0, 1], color="green", zorder=5, label="Start")
    plt.scatter(
        0.5 * (E_MIN + E_MAX),
        0.5 * (N_MIN + N_MAX),
        color="red",
        marker="x",
        s=80,
        label="Loop Origin",
    )
    plt.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    plt.axvline(0, color="gray", linewidth=0.5, linestyle="--")
    plt.xlabel("East (e) [m]")
    plt.ylabel("North (n) [m]")
    plt.title(f"Iron Hysteresis Trajectory (Rotation: {ROTATION_DEG} deg, Width: {MAX_WIDTH})")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()