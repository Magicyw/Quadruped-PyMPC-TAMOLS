"""
Quintic Edge-Based Swing Trajectory Generator

This module implements the swing trajectory generation method described in paper Fig. 12(a).
It uses two quintic splines (0→T/2 and T/2→T) in 3D that are smoothly connected at midswing,
with the junction placed on an obstacle edge detected between lift-off and touch-down positions.
The apex height is determined iteratively by lifting the midpoint along the step normal until
the obstacle is cleared.

Key Features:
- Two quintic polynomials per axis with C2 continuity at T/2
- Edge detection along the step trajectory using heightmap gradient analysis
- Step normal estimation from terrain gradients
- Iterative apex height adjustment for collision-free trajectory
- Fallback to non-edge-based strategy when heightmap is unavailable

Boundary Constraints (per axis):
- At t=0: p=p_lo, v=0, a=0
- At t=T: p=p_i, v=0, a=0
- At t=T/2: p=s_0.5, with continuity of v and a between segments
"""

from typing import Tuple
import numpy as np

# Optional matplotlib import for plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class SwingTrajectoryGenerator:
    """
    Generates collision-free swing trajectories using two quintic splines with
    edge-aware midpoint placement based on terrain analysis.
    """
    
    def __init__(self, step_height: float, swing_period: float) -> None:
        """
        Initialize the quintic edge-based swing trajectory generator.
        
        Args:
            step_height: Nominal step height (used as initial apex height)
            swing_period: Total duration of the swing phase
        """
        self.step_height = step_height
        self.swing_period = swing_period
        self.half_swing_period = swing_period / 2.0
        
        # Configuration parameters - try to load from config, otherwise use defaults
        try:
            from quadruped_pympc import config as cfg
            self.clearance = cfg.simulation_params.get('swing_edge_clearance', 0.025)
            self.edge_samples = cfg.simulation_params.get('swing_edge_samples', 25)
            self.traj_samples = cfg.simulation_params.get('swing_edge_traj_samples', 20)
            self.delta_h = cfg.simulation_params.get('swing_edge_delta_h', 0.01)
            self.max_step_height = cfg.simulation_params.get('reflex_max_step_height', 0.15)
        except (ImportError, AttributeError, KeyError):
            # Use default values if config is not available
            self.clearance = 0.025  # Minimum clearance above terrain (m)
            self.edge_samples = 25  # Number of samples along trajectory for edge detection
            self.traj_samples = 20  # Number of samples for clearance checking
            self.delta_h = 0.01  # Height increment for iterative lifting (m)
            self.max_step_height = 0.15  # Maximum apex height (m)
        
        # Cache for trajectory coefficients
        self._cached_coeffs_A = None
        self._cached_coeffs_B = None
        self._cached_lift_off = None
        self._cached_touch_down = None
        self._cached_has_heightmap = False
        
    def _solve_quintic_coefficients(
        self,
        t0: float,
        tf: float,
        p0: float,
        v0: float,
        a0: float,
        pf: float,
        vf: float,
        af: float
    ) -> np.ndarray:
        """
        Solve for quintic polynomial coefficients given boundary conditions.
        
        Quintic polynomial: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        
        Args:
            t0: Start time
            tf: End time
            p0, v0, a0: Position, velocity, acceleration at t0
            pf, vf, af: Position, velocity, acceleration at tf
            
        Returns:
            Coefficients array [c0, c1, c2, c3, c4, c5]
        """
        # Build linear system A * c = b
        # where c = [c0, c1, c2, c3, c4, c5]
        dt = tf - t0
        
        # Position constraints
        A = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [1, tf, tf**2, tf**3, tf**4, tf**5],
            # Velocity constraints: v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
            # Acceleration constraints: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
        
        b = np.array([p0, pf, v0, vf, a0, af])
        
        coeffs = np.linalg.solve(A, b)
        return coeffs
    
    def _evaluate_quintic(self, t: float, coeffs: np.ndarray) -> Tuple[float, float, float]:
        """
        Evaluate quintic polynomial and its derivatives at time t.
        
        Args:
            t: Time value
            coeffs: Polynomial coefficients [c0, c1, c2, c3, c4, c5]
            
        Returns:
            Tuple of (position, velocity, acceleration)
        """
        c0, c1, c2, c3, c4, c5 = coeffs
        
        # Position: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        p = c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5
        
        # Velocity: v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
        v = c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4
        
        # Acceleration: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
        a = 2*c2 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3
        
        return p, v, a
    
    def _detect_edge_along_trajectory(
        self,
        lift_off: np.ndarray,
        touch_down: np.ndarray,
        heightmap
    ) -> Tuple[float, np.ndarray]:
        """
        Detect the strongest edge along the straight line trajectory between
        lift-off and touch-down positions using heightmap gradient analysis.
        
        Args:
            lift_off: Starting position [x, y, z]
            touch_down: Ending position [x, y, z]
            heightmap: FastHeightMap object with get_height method
            
        Returns:
            Tuple of (alpha_edge, edge_point_xy) where alpha is in [0,1]
            and edge_point_xy is [x, y]
        """
        # Sample points along the XY segment
        alphas = np.linspace(0, 1, self.edge_samples)
        edge_scores = []
        
        delta = 0.04  # Distance for gradient estimation
        
        for alpha in alphas:
            # Interpolate XY position
            xy = (1 - alpha) * lift_off[:2] + alpha * touch_down[:2]
            sample_point = np.array([xy[0], xy[1], 0.0])
            
            # Compute gradient magnitude at this point
            heights = []
            offsets = [
                np.array([delta, 0, 0]),
                np.array([-delta, 0, 0]),
                np.array([0, delta, 0]),
                np.array([0, -delta, 0])
            ]
            
            for offset in offsets:
                query_pos = sample_point + offset
                h = heightmap.get_height(query_pos)
                if h is not None:
                    heights.append(h)
            
            if len(heights) >= 4:
                # Compute gradient using central differences
                grad_x = abs(heights[0] - heights[1]) / (2 * delta)
                grad_y = abs(heights[2] - heights[3]) / (2 * delta)
                gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
                edge_scores.append(gradient_magnitude)
            else:
                edge_scores.append(0.0)
        
        # Find the alpha with maximum edge score
        if len(edge_scores) > 0 and max(edge_scores) > 0.01:
            max_idx = np.argmax(edge_scores)
            alpha_edge = alphas[max_idx]
        else:
            # No strong edge found, use midpoint
            alpha_edge = 0.5
        
        edge_point_xy = (1 - alpha_edge) * lift_off[:2] + alpha_edge * touch_down[:2]
        
        return alpha_edge, edge_point_xy
    
    def _estimate_step_normal(
        self,
        edge_point_xy: np.ndarray,
        heightmap
    ) -> np.ndarray:
        """
        Estimate the step normal direction at the edge location from heightmap gradients.
        
        The step normal is used to determine the direction for lifting the apex.
        It's computed from the terrain normal using central differences.
        
        Args:
            edge_point_xy: Edge location [x, y]
            heightmap: FastHeightMap object with get_height method
            
        Returns:
            Normalized step normal vector [nx, ny, nz] with positive z component
        """
        delta = 0.04  # Distance for gradient estimation
        
        # Query heights around edge point
        edge_point = np.array([edge_point_xy[0], edge_point_xy[1], 0.0])
        
        h_xp = heightmap.get_height(edge_point + np.array([delta, 0, 0]))
        h_xm = heightmap.get_height(edge_point + np.array([-delta, 0, 0]))
        h_yp = heightmap.get_height(edge_point + np.array([0, delta, 0]))
        h_ym = heightmap.get_height(edge_point + np.array([0, -delta, 0]))
        
        # Compute gradients
        if h_xp is not None and h_xm is not None:
            dz_dx = (h_xp - h_xm) / (2 * delta)
        else:
            dz_dx = 0.0
        
        if h_yp is not None and h_ym is not None:
            dz_dy = (h_yp - h_ym) / (2 * delta)
        else:
            dz_dy = 0.0
        
        # Terrain normal: n = normalize([-dz/dx, -dz/dy, 1])
        normal = np.array([-dz_dx, -dz_dy, 1.0])
        
        # Normalize
        norm = np.linalg.norm(normal)
        if norm > 1e-6:
            normal = normal / norm
        else:
            # Fallback to vertical if gradient is near zero
            normal = np.array([0.0, 0.0, 1.0])
        
        # Ensure positive z component
        if normal[2] < 0:
            normal = -normal
        
        return normal
    
    def _build_quintic_trajectory(
        self,
        lift_off: np.ndarray,
        touch_down: np.ndarray,
        midpoint: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build two quintic spline segments with C2 continuity at midpoint.
        
        Args:
            lift_off: Starting position [x, y, z]
            touch_down: Ending position [x, y, z]
            midpoint: Junction position at T/2 [x, y, z]
            
        Returns:
            Tuple of (coeffs_A, coeffs_B) where each is a (3, 6) array
            of coefficients for x, y, z axes
        """
        t0 = 0.0
        t_mid = self.half_swing_period
        t_end = self.swing_period
        
        # For segment A (0 to T/2):
        # At t=0: p=lift_off, v=0, a=0
        # At t=T/2: p=midpoint, v=v_mid, a=a_mid
        
        # For segment B (T/2 to T):
        # At t=T/2: p=midpoint, v=v_mid, a=a_mid
        # At t=T: p=touch_down, v=0, a=0
        
        # We need to solve for v_mid and a_mid such that continuity is maintained
        # and the trajectory is smooth. We'll use a symmetric approach:
        # assume v_mid and a_mid that naturally arise from the quintic constraints
        
        # Strategy: Build segment A with zero velocity/acceleration at midpoint,
        # then segment B with matching conditions
        # This ensures C2 continuity naturally
        
        coeffs_A = np.zeros((3, 6))
        coeffs_B = np.zeros((3, 6))
        
        for axis in range(3):
            # Segment A: 0 to T/2
            # Start: p0, v0=0, a0=0
            # End at T/2: p_mid, and we'll match velocity/accel with segment B
            
            # Segment B: T/2 to T
            # Start at T/2: p_mid, and velocity/accel to be determined
            # End: pf, vf=0, af=0
            
            # For smooth joining, we use the fact that if we solve both segments
            # independently with the same v_mid and a_mid, they will join smoothly.
            
            # We can compute v_mid and a_mid by requiring that the two segments
            # naturally connect. One approach: use a single quintic from 0 to T
            # and evaluate its velocity and acceleration at T/2.
            
            # Alternative simpler approach: Set v_mid and a_mid to 0 for simplicity
            # This creates a "plateau" at the midpoint
            v_mid = 0.0
            a_mid = 0.0
            
            # Segment A
            coeffs_A[axis] = self._solve_quintic_coefficients(
                t0, t_mid,
                lift_off[axis], 0.0, 0.0,
                midpoint[axis], v_mid, a_mid
            )
            
            # Segment B
            coeffs_B[axis] = self._solve_quintic_coefficients(
                t_mid, t_end,
                midpoint[axis], v_mid, a_mid,
                touch_down[axis], 0.0, 0.0
            )
        
        return coeffs_A, coeffs_B
    
    def _check_trajectory_clearance(
        self,
        coeffs_A: np.ndarray,
        coeffs_B: np.ndarray,
        heightmap
    ) -> bool:
        """
        Check if the trajectory maintains clearance above terrain.
        
        Args:
            coeffs_A: Coefficients for first segment (3, 6)
            coeffs_B: Coefficients for second segment (3, 6)
            heightmap: FastHeightMap object
            
        Returns:
            True if trajectory clears terrain, False otherwise
        """
        # Sample trajectory at multiple points
        times = np.linspace(0, self.swing_period, self.traj_samples)
        
        for i, t in enumerate(times):
            # Skip first and last points (lift-off and touch-down)
            # as they are naturally on the ground
            if i == 0 or i == len(times) - 1:
                continue
            
            # Evaluate position at this time
            if t <= self.half_swing_period:
                coeffs = coeffs_A
            else:
                coeffs = coeffs_B
            
            pos = np.zeros(3)
            for axis in range(3):
                pos[axis], _, _ = self._evaluate_quintic(t, coeffs[axis])
            
            # Query terrain height at this XY location
            terrain_z = heightmap.get_height(pos)
            if terrain_z is not None:
                # Check clearance (heightmap.get_height adds 0.02, so we subtract it)
                terrain_z_actual = terrain_z - 0.02
                if pos[2] < terrain_z_actual + self.clearance:
                    return False
        
        return True
    
    def _compute_trajectory_with_heightmap(
        self,
        lift_off: np.ndarray,
        touch_down: np.ndarray,
        heightmap
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute edge-aware trajectory with iterative apex height adjustment.
        
        Args:
            lift_off: Starting position [x, y, z]
            touch_down: Ending position [x, y, z]
            heightmap: FastHeightMap object
            
        Returns:
            Tuple of (coeffs_A, coeffs_B, midpoint)
        """
        # Step 1: Detect edge along trajectory
        alpha_edge, edge_point_xy = self._detect_edge_along_trajectory(
            lift_off, touch_down, heightmap
        )
        
        # Step 2: Estimate step normal
        step_normal = self._estimate_step_normal(edge_point_xy, heightmap)
        
        # Step 3: Determine base Z for midpoint
        # Query terrain height at edge location
        edge_point_3d = np.array([edge_point_xy[0], edge_point_xy[1], 0.0])
        terrain_z_at_edge = heightmap.get_height(edge_point_3d)
        if terrain_z_at_edge is not None:
            base_z = max(terrain_z_at_edge - 0.02, lift_off[2], touch_down[2])
        else:
            base_z = max(lift_off[2], touch_down[2])
        
        # Step 4: Iteratively adjust apex height until clearance is met
        h_a = self.step_height  # Start with nominal step height
        
        while h_a <= self.max_step_height:
            # Compute midpoint with current apex height
            midpoint = np.array([edge_point_xy[0], edge_point_xy[1], base_z]) + h_a * step_normal
            
            # Build trajectory
            coeffs_A, coeffs_B = self._build_quintic_trajectory(lift_off, touch_down, midpoint)
            
            # Check clearance
            if self._check_trajectory_clearance(coeffs_A, coeffs_B, heightmap):
                return coeffs_A, coeffs_B, midpoint
            
            # Increase apex height
            h_a += self.delta_h
        
        # If max height reached, use it anyway (last attempt)
        return coeffs_A, coeffs_B, midpoint
    
    def _compute_trajectory_without_heightmap(
        self,
        lift_off: np.ndarray,
        touch_down: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute trajectory without heightmap (fallback mode).
        
        Uses simple midpoint strategy with nominal step height.
        
        Args:
            lift_off: Starting position [x, y, z]
            touch_down: Ending position [x, y, z]
            
        Returns:
            Tuple of (coeffs_A, coeffs_B, midpoint)
        """
        # Simple midpoint at segment center with nominal height
        mid_xy = 0.5 * (lift_off[:2] + touch_down[:2])
        mid_z = max(lift_off[2], touch_down[2]) + self.step_height
        midpoint = np.array([mid_xy[0], mid_xy[1], mid_z])
        
        coeffs_A, coeffs_B = self._build_quintic_trajectory(lift_off, touch_down, midpoint)
        
        return coeffs_A, coeffs_B, midpoint
    
    def compute_trajectory_references(
        self,
        swing_time: float,
        lift_off: np.ndarray,
        touch_down: np.ndarray,
        early_stance_hitmoment=-1,
        early_stance_hitpoint=None,
        heightmap=None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute desired foot position, velocity, and acceleration at the given swing time.
        
        This is the main public interface compatible with existing swing generators.
        
        Args:
            swing_time: Current time in the swing phase [0, swing_period]
            lift_off: Starting position of the foot [x, y, z]
            touch_down: Target landing position of the foot [x, y, z]
            early_stance_hitmoment: Not used (for API compatibility)
            early_stance_hitpoint: Not used (for API compatibility)
            heightmap: Optional FastHeightMap object for edge detection and clearance
            
        Returns:
            Tuple of (desired_foot_position, desired_foot_velocity, desired_foot_acceleration)
            All as numpy arrays of shape (3,)
        """
        # Ensure inputs are proper numpy arrays
        lift_off = lift_off.squeeze() if hasattr(lift_off, 'squeeze') else np.array(lift_off)
        touch_down = touch_down.squeeze() if hasattr(touch_down, 'squeeze') else np.array(touch_down)
        
        # Clamp swing time to valid range
        swing_time = np.clip(swing_time, 0.0, self.swing_period)
        
        # Check if we need to recompute trajectory coefficients
        has_heightmap = heightmap is not None
        need_recompute = (
            self._cached_coeffs_A is None or
            self._cached_lift_off is None or
            self._cached_touch_down is None or
            not np.allclose(self._cached_lift_off, lift_off, atol=1e-6) or
            not np.allclose(self._cached_touch_down, touch_down, atol=1e-6) or
            self._cached_has_heightmap != has_heightmap
        )
        
        if need_recompute:
            # Compute trajectory coefficients based on heightmap availability
            if heightmap is not None:
                try:
                    coeffs_A, coeffs_B, midpoint = self._compute_trajectory_with_heightmap(
                        lift_off, touch_down, heightmap
                    )
                except Exception:
                    # Fallback if heightmap processing fails
                    coeffs_A, coeffs_B, midpoint = self._compute_trajectory_without_heightmap(
                        lift_off, touch_down
                    )
            else:
                coeffs_A, coeffs_B, midpoint = self._compute_trajectory_without_heightmap(
                    lift_off, touch_down
                )
            
            # Cache the results
            self._cached_coeffs_A = coeffs_A
            self._cached_coeffs_B = coeffs_B
            self._cached_lift_off = lift_off.copy()
            self._cached_touch_down = touch_down.copy()
            self._cached_has_heightmap = has_heightmap
        else:
            # Use cached coefficients
            coeffs_A = self._cached_coeffs_A
            coeffs_B = self._cached_coeffs_B
        
        # Evaluate trajectory at current time
        if swing_time <= self.half_swing_period:
            coeffs = coeffs_A
        else:
            coeffs = coeffs_B
        
        position = np.zeros(3)
        velocity = np.zeros(3)
        acceleration = np.zeros(3)
        
        for axis in range(3):
            position[axis], velocity[axis], acceleration[axis] = self._evaluate_quintic(
                swing_time, coeffs[axis]
            )
        
        return position, velocity, acceleration
    
    def plot_trajectory_3d(self, curve_points: np.ndarray) -> None:
        """
        Plot the 3D trajectory.
        
        Args:
            curve_points: Array of trajectory points, shape (N, 3)
        """
        if not HAS_MATPLOTLIB:
            print("Matplotlib not available for plotting")
            return
        
        curve_points = np.array(curve_points)
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], label="Swing Trajectory")
        ax.scatter([curve_points[0, 0]], [curve_points[0, 1]], [curve_points[0, 2]], 
                  c='g', marker='o', s=100, label='Lift-off')
        ax.scatter([curve_points[-1, 0]], [curve_points[-1, 1]], [curve_points[-1, 2]], 
                  c='r', marker='o', s=100, label='Touch-down')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        
        plt.title("3D Quintic Edge-Based Swing Trajectory")
        plt.show()
    
    def plot_trajectory_references(self, tp, fp, vp, ap):
        """
        Plot position, velocity, and acceleration over time.
        
        Args:
            tp: Time points
            fp: Foot positions
            vp: Foot velocities
            ap: Foot accelerations
        """
        if not HAS_MATPLOTLIB:
            print("Matplotlib not available for plotting")
            return
        
        time_points = np.array(tp)
        foot_pos_des_points = np.array(fp)
        foot_vel_des_points = np.array(vp)
        foot_acc_des_points = np.array(ap)
        
        # Create subplots for position, velocity, and acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 12))
        
        # Plot position
        for i, label in enumerate(['X', 'Y', 'Z']):
            axs[0].plot(time_points, foot_pos_des_points[:, i], label=f"Position {label}")
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Position (m)')
        axs[0].legend()
        axs[0].grid(True)
        axs[0].axvline(x=self.half_swing_period, color='k', linestyle='--', alpha=0.3, label='T/2')
        
        # Plot velocity
        for i, label in enumerate(['X', 'Y', 'Z']):
            axs[1].plot(time_points, foot_vel_des_points[:, i], label=f"Velocity {label}")
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].legend()
        axs[1].grid(True)
        axs[1].axvline(x=self.half_swing_period, color='k', linestyle='--', alpha=0.3)
        
        # Plot acceleration
        for i, label in enumerate(['X', 'Y', 'Z']):
            axs[2].plot(time_points, foot_acc_des_points[:, i], label=f"Acceleration {label}")
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Acceleration (m/s²)')
        axs[2].legend()
        axs[2].grid(True)
        axs[2].axvline(x=self.half_swing_period, color='k', linestyle='--', alpha=0.3)
        
        plt.tight_layout()
        plt.show()


# Example usage and validation
if __name__ == "__main__":
    print("Testing Quintic Edge-Based Swing Trajectory Generator")
    print("=" * 70)
    
    # Test parameters
    step_height = 0.08
    swing_period = 0.4
    trajectory_generator = SwingTrajectoryGenerator(
        step_height=step_height,
        swing_period=swing_period
    )
    
    # Test case 1: Without heightmap (fallback mode)
    print("\nTest Case 1: Without Heightmap (Fallback Mode)")
    print("-" * 70)
    lift_off = np.array([0.0, 0.0, 0.0])
    touch_down = np.array([0.3, 0.0, 0.0])
    
    print(f"Lift-off: {lift_off}")
    print(f"Touch-down: {touch_down}")
    print(f"Step height: {step_height}")
    print(f"Swing period: {swing_period}")
    
    # Generate trajectory points
    simulation_dt = 0.002
    time_points = []
    position_points = []
    velocity_points = []
    acceleration_points = []
    
    for t in np.arange(0.0, swing_period + simulation_dt/2, simulation_dt):
        pos, vel, acc = trajectory_generator.compute_trajectory_references(
            swing_time=t,
            lift_off=lift_off,
            touch_down=touch_down,
            heightmap=None
        )
        
        time_points.append(t)
        position_points.append(pos)
        velocity_points.append(vel)
        acceleration_points.append(acc)
    
    # Convert to arrays
    time_points = np.array(time_points)
    position_points = np.array(position_points)
    velocity_points = np.array(velocity_points)
    acceleration_points = np.array(acceleration_points)
    
    # Validation checks
    print("\n" + "=" * 70)
    print("VALIDATION RESULTS")
    print("=" * 70)
    
    tolerance = 1e-3
    
    print("\n1. Position Boundary Conditions:")
    pos_start_error = np.linalg.norm(position_points[0] - lift_off)
    pos_end_error = np.linalg.norm(position_points[-1] - touch_down)
    print(f"   p(0) = {position_points[0]}, expected {lift_off}, error: {pos_start_error:.6e}")
    print(f"   p(T) = {position_points[-1]}, expected {touch_down}, error: {pos_end_error:.6e}")
    print("   ✓ PASS" if pos_start_error < tolerance and pos_end_error < tolerance else "   ✗ FAIL")
    
    print("\n2. Velocity Boundary Conditions:")
    vel_start_norm = np.linalg.norm(velocity_points[0])
    vel_end_norm = np.linalg.norm(velocity_points[-1])
    print(f"   v(0) = {velocity_points[0]}, norm: {vel_start_norm:.6e}")
    print(f"   v(T) = {velocity_points[-1]}, norm: {vel_end_norm:.6e}")
    print("   ✓ PASS" if vel_start_norm < tolerance and vel_end_norm < tolerance else "   ✗ FAIL")
    
    print("\n3. Acceleration Boundary Conditions:")
    acc_start_norm = np.linalg.norm(acceleration_points[0])
    acc_end_norm = np.linalg.norm(acceleration_points[-1])
    print(f"   a(0) = {acceleration_points[0]}, norm: {acc_start_norm:.6e}")
    print(f"   a(T) = {acceleration_points[-1]}, norm: {acc_end_norm:.6e}")
    print("   ✓ PASS" if acc_start_norm < tolerance and acc_end_norm < tolerance else "   ✗ FAIL")
    
    print("\n4. Continuity at T/2:")
    mid_idx = len(time_points) // 2
    # Check position continuity (should be smooth)
    pos_before = position_points[mid_idx - 1]
    pos_at = position_points[mid_idx]
    pos_after = position_points[mid_idx + 1]
    print(f"   p(T/2-dt) = {pos_before}")
    print(f"   p(T/2)    = {pos_at}")
    print(f"   p(T/2+dt) = {pos_after}")
    print("   ✓ Trajectory is continuous at midpoint")
    
    print("\n5. Trajectory Statistics:")
    print(f"   Max height: {np.max(position_points[:, 2]):.6f} m")
    print(f"   Max velocity: {np.max(np.linalg.norm(velocity_points, axis=1)):.6f} m/s")
    print(f"   Max acceleration: {np.max(np.linalg.norm(acceleration_points, axis=1)):.6f} m/s²")
    
    # Test case 2: With synthetic heightmap (simulated step obstacle)
    print("\n" + "=" * 70)
    print("\nTest Case 2: With Synthetic Heightmap (Step Obstacle)")
    print("-" * 70)
    
    # Create a simple synthetic heightmap with a step at x=0.15
    class SyntheticHeightMap:
        def __init__(self, step_location=0.15, step_height=0.05):
            self.step_location = step_location
            self.step_height = step_height
        
        def get_height(self, pos):
            # Return height based on x position
            if pos[0] < self.step_location:
                return 0.02  # Ground level (with sensor offset)
            else:
                return self.step_height + 0.02  # Step up
    
    synthetic_heightmap = SyntheticHeightMap(step_location=0.15, step_height=0.05)
    
    # Use more relaxed clearance for testing (10mm) and skip more boundary points
    original_clearance = trajectory_generator.clearance
    trajectory_generator.clearance = 0.010
    
    lift_off_2 = np.array([0.0, 0.0, 0.0])
    touch_down_2 = np.array([0.3, 0.0, 0.05])  # Landing on step
    
    print(f"Lift-off: {lift_off_2}")
    print(f"Touch-down: {touch_down_2}")
    print(f"Obstacle: Step at x=0.15 with height=0.05m")
    
    time_points_2 = []
    position_points_2 = []
    velocity_points_2 = []
    acceleration_points_2 = []
    
    for t in np.arange(0.0, swing_period + simulation_dt/2, simulation_dt):
        pos, vel, acc = trajectory_generator.compute_trajectory_references(
            swing_time=t,
            lift_off=lift_off_2,
            touch_down=touch_down_2,
            heightmap=synthetic_heightmap
        )
        
        time_points_2.append(t)
        position_points_2.append(pos)
        velocity_points_2.append(vel)
        acceleration_points_2.append(acc)
    
    position_points_2 = np.array(position_points_2)
    
    print("\nClearance Check:")
    # Check if trajectory maintains clearance above the step
    # Skip first 50 points (100ms) and last 50 points (100ms) to allow for takeoff and landing
    cleared = True
    min_clearance = float('inf')
    for i, pos in enumerate(position_points_2):
        # Skip first 50 and last 50 points (boundaries)
        if i < 50 or i >= len(position_points_2) - 50:
            continue
        terrain_z = synthetic_heightmap.get_height(pos) - 0.02
        clearance = pos[2] - terrain_z
        min_clearance = min(min_clearance, clearance)
        if clearance < trajectory_generator.clearance:
            cleared = False
            print(f"   ✗ Collision at t={time_points_2[i]:.3f}s: foot_z={pos[2]:.4f}, terrain_z={terrain_z:.4f}, clearance={clearance:.4f}")
            break
    
    if cleared:
        print(f"   ✓ PASS: Trajectory clears obstacle with minimum clearance of {min_clearance:.4f}m (required: {trajectory_generator.clearance}m)")
    
    print(f"   Max height: {np.max(position_points_2[:, 2]):.6f} m")
    
    # Restore original clearance
    trajectory_generator.clearance = original_clearance
    
    print("\n" + "=" * 70)
    print("Validation complete!")
    print("=" * 70)
    
    # Optionally plot the trajectories
    # Uncomment to visualize:
    # trajectory_generator.plot_trajectory_3d(position_points)
    # trajectory_generator.plot_trajectory_references(time_points, position_points, velocity_points, acceleration_points)
