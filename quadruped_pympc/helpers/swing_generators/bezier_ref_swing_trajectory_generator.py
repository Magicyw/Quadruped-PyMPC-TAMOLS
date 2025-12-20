"""
Bezier Reference Swing Trajectory Generator

This module implements a 6th-degree 3D Bézier curve-based swing trajectory generator
for quadruped locomotion. The trajectory satisfies the following constraints:

Boundary Constraints:
- p(0) = lift_off: Starting position
- p(T) = touch_down: Ending position  
- v(0) = 0, a(0) = 0: Zero velocity and acceleration at start
- v(T) = 0, a(T) = 0: Zero velocity and acceleration at end

Midpoint Constraints:
- p_z(T/2) = z_mid where z_mid = max(lift_off.z, touch_down.z) + step_height
- P3_xy = 0.5 * (lift_off_xy + touch_down_xy): Midpoint in x-y plane

Control Points Construction:
Given the constraints v(0)=0 and a(0)=0, we get:
- P0 = lift_off
- P1 = P0 (from v(0)=0)
- P2 = P0 (from a(0)=0)

Given the constraints v(T)=0 and a(T)=0, we get:
- P6 = touch_down
- P5 = P6 (from v(T)=0)
- P4 = P6 (from a(T)=0)

The midpoint constraint p_z(T/2) determines P3_z, and we set P3_xy to the midpoint.
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
    Generates swing trajectories using 6th-degree Bézier curves.
    
    The trajectory ensures smooth transitions with zero velocity and acceleration
    at the start and end points, while reaching a specified height at the midpoint.
    """
    
    def __init__(self, step_height: float, swing_period: float) -> None:
        """
        Initialize the Bézier swing trajectory generator.
        
        Args:
            step_height: Maximum height above the ground during swing phase
            swing_period: Total duration of the swing phase
        """
        self.step_height = step_height
        self.swing_period = swing_period
        
    def _compute_control_points(
        self, lift_off: np.ndarray, touch_down: np.ndarray
    ) -> np.ndarray:
        """
        Compute the 7 control points (P0 to P6) for the 6th-degree Bézier curve.
        
        Args:
            lift_off: Starting position [x, y, z]
            touch_down: Ending position [x, y, z]
            
        Returns:
            Control points array of shape (7, 3)
        """
        # Ensure inputs are proper numpy arrays
        p0 = lift_off.squeeze()
        pf = touch_down.squeeze()
        
        # Initialize control points
        control_points = np.zeros((7, 3))
        
        # Start constraints: v(0)=0, a(0)=0
        # This gives us P0=P1=P2=lift_off
        control_points[0] = p0  # P0
        control_points[1] = p0  # P1 = P0 (for v(0)=0)
        control_points[2] = p0  # P2 = P0 (for a(0)=0)
        
        # End constraints: v(T)=0, a(T)=0
        # This gives us P6=P5=P4=touch_down
        control_points[6] = pf  # P6
        control_points[5] = pf  # P5 = P6 (for v(T)=0)
        control_points[4] = pf  # P4 = P6 (for a(T)=0)
        
        # Midpoint constraint: p_z(T/2) = z_mid
        # For a Bézier curve, p(0.5) = (1/64) * (P0 + 6*P1 + 15*P2 + 20*P3 + 15*P4 + 6*P5 + P6)
        # With our constraints: P0=P1=P2 and P4=P5=P6
        # p(0.5) = (1/64) * (22*P0 + 20*P3 + 22*P6)
        # Solving for P3: P3 = (64*p(0.5) - 22*P0 - 22*P6) / 20
        
        # Compute target midpoint height
        z_mid = max(p0[2], pf[2]) + self.step_height
        
        # Midpoint x,y coordinates
        p_mid_xy = 0.5 * (p0[:2] + pf[:2])
        
        # Solve for P3_z using the Bézier midpoint formula
        # At s=0.5, the Bernstein basis values for degree 6 are:
        # B_{0,6}(0.5) = 1/64, B_{1,6}(0.5) = 6/64, B_{2,6}(0.5) = 15/64, B_{3,6}(0.5) = 20/64
        # B_{4,6}(0.5) = 15/64, B_{5,6}(0.5) = 6/64, B_{6,6}(0.5) = 1/64
        # p_z(0.5) = (1/64)*P0_z + (6/64)*P1_z + (15/64)*P2_z + (20/64)*P3_z + (15/64)*P4_z + (6/64)*P5_z + (1/64)*P6_z
        # With P0=P1=P2 and P4=P5=P6:
        # p_z(0.5) = (1/64)*P0_z + (6/64)*P0_z + (15/64)*P0_z + (20/64)*P3_z + (15/64)*P6_z + (6/64)*P6_z + (1/64)*P6_z
        # p_z(0.5) = (22/64)*P0_z + (20/64)*P3_z + (22/64)*P6_z
        # 64*p_z(0.5) = 22*P0_z + 20*P3_z + 22*P6_z
        # P3_z = (64*p_z(0.5) - 22*P0_z - 22*P6_z) / 20
        
        p3_z = (64 * z_mid - 22 * p0[2] - 22 * pf[2]) / 20
        
        # Set P3 with midpoint x,y and computed z
        control_points[3] = np.array([p_mid_xy[0], p_mid_xy[1], p3_z])
        
        return control_points
        
    def _bernstein_basis(self, s: float, i: int, n: int) -> float:
        """
        Compute the Bernstein basis polynomial B_{i,n}(s).
        
        Args:
            s: Parameter value in [0, 1]
            i: Index of the basis function
            n: Degree of the polynomial
            
        Returns:
            Value of the basis function
        """
        from math import comb
        return comb(n, i) * (s ** i) * ((1 - s) ** (n - i))
    
    def _bernstein_basis_derivative(self, s: float, i: int, n: int) -> float:
        """
        Compute the first derivative of Bernstein basis polynomial dB_{i,n}(s)/ds.
        
        Args:
            s: Parameter value in [0, 1]
            i: Index of the basis function
            n: Degree of the polynomial
            
        Returns:
            Value of the derivative of the basis function
        """
        if n == 0:
            return 0.0
        from math import comb
        result = 0.0
        if i > 0:
            result += i * comb(n, i) * (s ** (i-1)) * ((1 - s) ** (n - i))
        if i < n:
            result -= (n - i) * comb(n, i) * (s ** i) * ((1 - s) ** (n - i - 1))
        return result
    
    def _bernstein_basis_second_derivative(self, s: float, i: int, n: int) -> float:
        """
        Compute the second derivative of Bernstein basis polynomial d²B_{i,n}(s)/ds².
        
        Args:
            s: Parameter value in [0, 1]
            i: Index of the basis function
            n: Degree of the polynomial
            
        Returns:
            Value of the second derivative of the basis function
        """
        if n <= 1:
            return 0.0
        from math import comb
        result = 0.0
        
        # d²B/ds² = d/ds[dB/ds]
        # Using the derivative formula, we compute each term
        if i >= 2:
            result += i * (i - 1) * comb(n, i) * (s ** (i-2)) * ((1 - s) ** (n - i))
        if i >= 1 and i < n:
            result -= 2 * i * (n - i) * comb(n, i) * (s ** (i-1)) * ((1 - s) ** (n - i - 1))
        if i < n - 1:
            result += (n - i) * (n - i - 1) * comb(n, i) * (s ** i) * ((1 - s) ** (n - i - 2))
            
        return result
    
    def _evaluate_bezier_curve(
        self, s: float, control_points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Evaluate position, velocity (w.r.t. s), and acceleration (w.r.t. s) at parameter s.
        
        Args:
            s: Normalized time parameter in [0, 1]
            control_points: Control points array of shape (7, 3)
            
        Returns:
            Tuple of (position, velocity_s, acceleration_s) where velocity and acceleration
            are with respect to the parameter s (not time t)
        """
        n = 6  # Degree of Bézier curve
        
        # Initialize outputs
        position = np.zeros(3)
        velocity_s = np.zeros(3)
        acceleration_s = np.zeros(3)
        
        # Sum over all control points
        for i in range(7):  # 0 to 6 for degree 6
            b_i = self._bernstein_basis(s, i, n)
            db_i = self._bernstein_basis_derivative(s, i, n)
            d2b_i = self._bernstein_basis_second_derivative(s, i, n)
            
            position += b_i * control_points[i]
            velocity_s += db_i * control_points[i]
            acceleration_s += d2b_i * control_points[i]
        
        return position, velocity_s, acceleration_s
    
    def compute_trajectory_references(
        self,
        swing_time: float,
        lift_off: np.ndarray,
        touch_down: np.ndarray,
        early_stance_hitmoment=-1,
        early_stance_hitpoint=None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute desired foot position, velocity, and acceleration at the given swing time.
        
        Args:
            swing_time: Current time in the swing phase [0, swing_period]
            lift_off: Starting position of the foot
            touch_down: Target landing position of the foot
            early_stance_hitmoment: Not used in this implementation (for compatibility)
            early_stance_hitpoint: Not used in this implementation (for compatibility)
            
        Returns:
            Tuple of (desired_foot_position, desired_foot_velocity, desired_foot_acceleration)
            All as numpy arrays of shape (3,)
        """
        # Normalize time to [0, 1]
        s = np.clip(swing_time / self.swing_period, 0.0, 1.0)
        
        # Compute control points
        control_points = self._compute_control_points(lift_off, touch_down)
        
        # Evaluate Bézier curve
        position, velocity_s, acceleration_s = self._evaluate_bezier_curve(s, control_points)
        
        # Convert derivatives from s-domain to t-domain
        # ds/dt = 1/T, where T is swing_period
        # dp/dt = (dp/ds) * (ds/dt) = (dp/ds) / T
        # d²p/dt² = d/dt[dp/dt] = d/dt[(dp/ds) / T] = (d²p/ds²) * (ds/dt)² = (d²p/ds²) / T²
        
        velocity = velocity_s / self.swing_period
        acceleration = acceleration_s / (self.swing_period ** 2)
        
        return (
            position.reshape((3,)),
            velocity.reshape((3,)),
            acceleration.reshape((3,))
        )
    
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
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        
        plt.title("3D Bézier Swing Trajectory")
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
        fig, axs = plt.subplots(3, 1, figsize=(8, 12))
        
        # Plot position
        for i, label in enumerate(['X', 'Y', 'Z']):
            axs[0].plot(time_points, foot_pos_des_points[:, i], label=f"Position {label}")
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Position (m)')
        axs[0].legend()
        axs[0].grid(True)
        
        # Plot velocity
        for i, label in enumerate(['X', 'Y', 'Z']):
            axs[1].plot(time_points, foot_vel_des_points[:, i], label=f"Velocity {label}")
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].legend()
        axs[1].grid(True)
        
        # Plot acceleration
        for i, label in enumerate(['X', 'Y', 'Z']):
            axs[2].plot(time_points, foot_acc_des_points[:, i], label=f"Acceleration {label}")
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Acceleration (m/s²)')
        axs[2].legend()
        axs[2].grid(True)
        
        plt.tight_layout()
        plt.show()


# Example usage and validation
if __name__ == "__main__":
    print("Testing Bézier Reference Swing Trajectory Generator")
    print("=" * 60)
    
    # Test parameters
    step_height = 0.08
    swing_period = 0.4
    trajectory_generator = SwingTrajectoryGenerator(
        step_height=step_height, 
        swing_period=swing_period
    )
    
    # Test case 1: Horizontal movement
    lift_off = np.array([0.0, 0.0, 0.0])
    touch_down = np.array([0.2, 0.0, 0.0])
    
    print("\nTest Case 1: Horizontal Movement")
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
            touch_down=touch_down
        )
        
        time_points.append(t)
        position_points.append(pos)
        velocity_points.append(vel)
        acceleration_points.append(acc)
    
    # Convert to arrays for analysis
    time_points = np.array(time_points)
    position_points = np.array(position_points)
    velocity_points = np.array(velocity_points)
    acceleration_points = np.array(acceleration_points)
    
    # Validation checks
    print("\n" + "=" * 60)
    print("VALIDATION RESULTS")
    print("=" * 60)
    
    # Check boundary conditions
    tolerance = 1e-6
    
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
    
    print("\n4. Midpoint Height Constraint:")
    mid_idx = len(time_points) // 2
    z_mid_actual = position_points[mid_idx, 2]
    z_mid_expected = max(lift_off[2], touch_down[2]) + step_height
    z_mid_error = abs(z_mid_actual - z_mid_expected)
    print(f"   p_z(T/2) = {z_mid_actual:.6f}, expected: {z_mid_expected:.6f}, error: {z_mid_error:.6e}")
    print("   ✓ PASS" if z_mid_error < 1e-3 else "   ✗ FAIL")
    
    print("\n5. Trajectory Statistics:")
    print(f"   Max height: {np.max(position_points[:, 2]):.6f} m")
    print(f"   Max velocity: {np.max(np.linalg.norm(velocity_points, axis=1)):.6f} m/s")
    print(f"   Max acceleration: {np.max(np.linalg.norm(acceleration_points, axis=1)):.6f} m/s²")
    
    print("\n" + "=" * 60)
    print("Validation complete!")
    print("=" * 60)
    
    # Optionally plot the trajectory
    # Uncomment to visualize:
    # trajectory_generator.plot_trajectory_3d(position_points)
    # trajectory_generator.plot_trajectory_references(time_points, position_points, velocity_points, acceleration_points)
