# Bézier Reference Swing Trajectory Generator

This document describes the `bezier_ref` swing trajectory generator added to the Quadruped-PyMPC-TAMOLS project.

## Overview

The `bezier_ref` generator implements a 6th-degree 3D Bézier curve-based swing trajectory for quadruped foot motion during the swing phase. This generator provides smooth trajectories with zero velocity and acceleration at the start and end points.

## Features

- **6th-degree 3D Bézier curve**: Provides smooth, continuous trajectories in all three dimensions (x, y, z)
- **Zero boundary conditions**: Ensures v(0)=0, a(0)=0, v(T)=0, a(T)=0 for smooth transitions
- **Configurable step height**: Midpoint height constraint ensures clearance over obstacles
- **Deterministic**: Pure analytical solution without iterative optimization

## Mathematical Formulation

### Control Points Construction

Given boundary constraints:
- Position: `p(0) = lift_off`, `p(T) = touch_down`
- Velocity: `v(0) = 0`, `v(T) = 0`
- Acceleration: `a(0) = 0`, `a(T) = 0`
- Midpoint height: `p_z(T/2) = max(lift_off.z, touch_down.z) + step_height`
- Midpoint x-y: `P3_xy = 0.5 * (lift_off_xy + touch_down_xy)`

The 7 control points (P0 to P6) are determined as:
- **P0 = P1 = P2 = lift_off** (ensures v(0)=0 and a(0)=0)
- **P6 = P5 = P4 = touch_down** (ensures v(T)=0 and a(T)=0)
- **P3** is computed to satisfy the midpoint height constraint

### Trajectory Evaluation

The trajectory is evaluated using Bernstein basis polynomials:
- **Position**: `p(s) = Σ B_i,6(s) * P_i` for i=0..6
- **Velocity**: `dp/dt = (dp/ds) / T`
- **Acceleration**: `d²p/dt² = (d²p/ds²) / T²`

where `s = t/T` is the normalized time parameter in [0, 1].

## Usage

### In Configuration File

Set the swing generator in your config file:

```python
# In quadruped_pympc/config.py or your custom config
simulation_params = {
    'swing_generator': 'bezier_ref',  # Options: 'scipy', 'explicit', 'bezier_ref'
    'step_height': 0.06,  # 6 cm step height
    # ... other parameters
}
```

### Programmatic Usage

```python
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
import numpy as np

# Create controller with bezier_ref generator
controller = SwingTrajectoryController(
    step_height=0.06,
    swing_period=0.4,
    position_gain_fb=np.array([500.0, 500.0, 500.0]),
    velocity_gain_fb=np.array([10.0, 10.0, 10.0]),
    generator="bezier_ref"
)

# Generate trajectory references
lift_off = np.array([0.0, 0.0, 0.0])
touch_down = np.array([0.2, 0.0, 0.0])
swing_time = 0.2  # 200ms into swing phase

pos, vel, acc = controller.swing_generator.compute_trajectory_references(
    swing_time=swing_time,
    lift_off=lift_off,
    touch_down=touch_down
)
```

## Comparison with Other Generators

| Generator | Method | Boundary Conditions | Optimization |
|-----------|--------|-------------------|--------------|
| `explicit` | Two cubic Bézier curves | v(0)≈0, v(T)≈0 | None |
| `scipy` | Cubic spline interpolation | Clamped endpoints | None |
| `bezier_ref` | Single 6th-degree Bézier | v=0, a=0 at endpoints | None |

### Advantages of `bezier_ref`:
- **Exact zero velocity and acceleration** at start/end points
- **Single continuous curve** instead of piecewise segments
- **Analytical derivatives** for velocity and acceleration
- **Smooth acceleration profile** due to higher degree polynomial

### When to Use:
- When you need guaranteed zero velocity/acceleration at boundaries
- For precise trajectory control in challenging terrain
- When smooth acceleration is important for actuator performance

## Validation

The generator includes built-in validation that verifies:
1. Position boundary conditions: `p(0)=lift_off`, `p(T)=touch_down`
2. Velocity boundary conditions: `||v(0)||≈0`, `||v(T)||≈0`
3. Acceleration boundary conditions: `||a(0)||≈0`, `||a(T)||≈0`
4. Midpoint height constraint: `p_z(T/2) ≈ z_mid`

Run the validation script:
```bash
python quadruped_pympc/helpers/swing_generators/bezier_ref_swing_trajectory_generator.py
```

## Implementation Details

- **File**: `quadruped_pympc/helpers/swing_generators/bezier_ref_swing_trajectory_generator.py`
- **Class**: `SwingTrajectoryGenerator`
- **Interface**: Compatible with existing swing generators
- **Dependencies**: `numpy` (matplotlib optional for plotting)

## References

This implementation follows the reference trajectory generation approach described in papers on Bézier curve-based motion planning for legged robots, providing a reference trajectory `p_ref(t)` without QP-based collision avoidance.

## Example Output

For a forward step with step_height=0.06m, swing_period=0.4s:

```
Time (s)  X (m)    Y (m)    Z (m)    ||v|| (m/s)  ||a|| (m/s²)
0.000     0.0000   0.0000   0.0000   0.0000       0.0000
0.100     0.0259   0.0000   0.0253   0.8311       8.9496
0.200     0.1250   0.0000   0.0600   1.1719       9.0000
0.300     0.2241   0.0000   0.0253   0.8311       8.9496
0.400     0.2500   0.0000   0.0000   0.0000       0.0000
```

Note: Zero velocity and acceleration at start (t=0) and end (t=0.4).
