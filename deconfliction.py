import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import interp1d
from datetime import datetime


###############################################
# Drone Trajectory Model
###############################################
class DroneTrajectory:
    """Represents a drone's 3D flight path with linear interpolation."""

    def __init__(self, drone_id: str, waypoints, start_time: datetime, speed_mps: float = 5.0):
        self.drone_id = drone_id
        self.waypoints = np.array(waypoints, dtype=float)  # [[x, y, z], ...]
        self.start_time = start_time
        self.speed = speed_mps

        self.times, self.total_duration = self._calculate_timings()

        # Interpolation functions for x, y, z over time
        self.interp_x = interp1d(self.times, self.waypoints[:, 0], kind="linear", fill_value="extrapolate")
        self.interp_y = interp1d(self.times, self.waypoints[:, 1], kind="linear", fill_value="extrapolate")
        self.interp_z = interp1d(self.times, self.waypoints[:, 2], kind="linear", fill_value="extrapolate")

    def _calculate_timings(self):
        """Assigns timestamps to each waypoint assuming constant speed between segments."""
        distances = [0.0]
        for i in range(1, len(self.waypoints)):
            dist = np.linalg.norm(self.waypoints[i] - self.waypoints[i - 1])
            distances.append(distances[-1] + dist)

        times = np.array(distances) / self.speed
        return times, times[-1]

    def get_position_at(self, t: float):
        """Returns [x, y, z] at relative time t seconds from mission start. Returns None if outside window."""
        if t < 0 or t > self.total_duration:
            return None
        return np.array([
            float(self.interp_x(t)),
            float(self.interp_y(t)),
            float(self.interp_z(t)),
        ])


###############################################
# Deconfliction Authority Service
###############################################
class DeconflictionService:
    """Central authority to verify whether a primary flight is safe."""

    def __init__(self, spatial_buffer: float = 5.0, temporal_buffer: float = 10.0):
        self.spatial_buffer = spatial_buffer    # meters
        self.temporal_buffer = temporal_buffer  # seconds (currently implicit via time alignment)
        self.scheduled_flights = []

    def add_scheduled_flight(self, trajectory: DroneTrajectory):
        self.scheduled_flights.append(trajectory)

    def validate_mission(self, primary_mission: DroneTrajectory):
        """
        Checks the primary mission against all scheduled flights.

        Returns:
            ("CLEAR", [])
            or
            ("CONFLICT", [
                {
                    "other_drone": str,
                    "time_relative": float,
                    "location": [x, y, z],
                    "distance": float
                }, ...
            ])
        """
        conflicts = []

        # Sample flight at 0.5s resolution
        check_intervals = np.arange(0, primary_mission.total_duration, 0.5)

        for other in self.scheduled_flights:
            # Align global clock
            time_offset = (other.start_time - primary_mission.start_time).total_seconds()

            for t in check_intervals:
                p_pos = primary_mission.get_position_at(t)
                o_pos = other.get_position_at(t - time_offset)

                if p_pos is None or o_pos is None:
                    continue

                separation = np.linalg.norm(p_pos - o_pos)

                if separation < self.spatial_buffer:
                    conflicts.append({
                        "other_drone": other.drone_id,
                        "time_relative": round(t, 2),
                        "location": p_pos.tolist(),
                        "distance": round(separation, 2)
                    })
                    break  # First conflict per drone is enough

        if not conflicts:
            return "CLEAR", []
        return "CONFLICT", conflicts


###############################################
# Basic Sanity Tests (non-visual)
###############################################
def _run_basic_tests():
    print("\nRunning sanity tests...")

    service = DeconflictionService(spatial_buffer=10.0)
    now = datetime.now()

    # Drone crossing path (expected conflict)
    conflict_drone = DroneTrajectory(
        "Test_Conflict",
        waypoints=[[0, 50, 20], [100, 50, 20]],
        start_time=now,
    )

    # Drone clearly safe (higher altitude)
    safe_drone = DroneTrajectory(
        "Test_Safe",
        waypoints=[[0, 50, 80], [100, 50, 80]],
        start_time=now,
    )

    service.add_scheduled_flight(conflict_drone)
    service.add_scheduled_flight(safe_drone)

    primary = DroneTrajectory(
        "Primary_Test",
        waypoints=[[50, 0, 20], [50, 100, 20]],
        start_time=now,
    )

    status, conflicts = service.validate_mission(primary)

    assert status == "CONFLICT", "Expected conflict not detected"
    assert any(c["other_drone"] == "Test_Conflict" for c in conflicts), "Conflict drone not reported"

    print("Sanity tests passed ✔️")


###############################################
# Demonstration + Visualization
###############################################
def run_demo_scenarios():
    service = DeconflictionService(spatial_buffer=10.0)
    now = datetime.now()

    # Scheduled drones
    drone_b = DroneTrajectory(
        "Drone_B_Cargo",
        waypoints=[[0, 50, 20], [100, 50, 20]],
        start_time=now,
    )

    drone_c = DroneTrajectory(
        "Drone_C_Survey",
        waypoints=[[50, 0, 80], [50, 100, 80]],
        start_time=now,
    )

    service.add_scheduled_flight(drone_b)
    service.add_scheduled_flight(drone_c)

    # Primary mission
    primary_drone = DroneTrajectory(
        "Primary_Alpha",
        waypoints=[[50, 0, 20], [50, 100, 20]],
        start_time=now,
    )

    print(f"--- Validating Mission: {primary_drone.drone_id} ---")
    status, conflicts = service.validate_mission(primary_drone)

    print(f"Status: {status}")
    if conflicts:
        for c in conflicts:
            print(f"ALERT: Potential collision with {c['other_drone']} at {c['time_relative']}s")
            print(f"Location: {c['location']} | Separation: {c['distance']}m")

    # Visualization
    fig = go.Figure()

    def add_traj_to_plot(traj: DroneTrajectory, color, name, is_primary=False):
        t_steps = np.linspace(0, traj.total_duration, 50)
        path = np.array([traj.get_position_at(t) for t in t_steps])
        fig.add_trace(go.Scatter3d(
            x=path[:, 0], y=path[:, 1], z=path[:, 2],
            mode='lines',
            line=dict(color=color, width=6 if is_primary else 3),
            name=name
        ))

    # Draw paths
    add_traj_to_plot(primary_drone, 'blue', 'Primary Mission (Alpha)', True)
    add_traj_to_plot(drone_b, 'red', 'Scheduled: Drone B (Conflict)')
    add_traj_to_plot(drone_c, 'green', 'Scheduled: Drone C (Safe)')

    # Mark conflict zones if any
    for c in conflicts:
        fig.add_trace(go.Scatter3d(
            x=[c['location'][0]], y=[c['location'][1]], z=[c['location'][2]],
            mode='markers',
            marker=dict(size=10, color='orange', symbol='diamond'),
            name='Conflict Zone'
        ))

    fig.update_layout(
        title="UAV Strategic Deconfliction - 4D Spatiotemporal Analysis",
        scene=dict(xaxis_title='X (m)', yaxis_title='Y (m)', zaxis_title='Altitude (m)'),
        margin=dict(l=0, r=0, b=0, t=40)
    )

    fig.write_html("deconfliction_viz.html")
    print("\nVisualization saved to 'deconfliction_viz.html'. Open this file to see the 4D paths.")


if __name__ == "__main__":
    _run_basic_tests()
    run_demo_scenarios()
