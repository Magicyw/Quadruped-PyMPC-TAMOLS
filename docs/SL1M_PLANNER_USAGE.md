# sl1m High-Level Foothold Planner for Plum Piles Terrain

This document explains how to use the integrated sl1m high-level foothold planner for navigating dense plum piles (stepping stones) terrain in the Quadruped-PyMPC-TAMOLS repository.

## Overview

The sl1m planner integration provides **Mode A** functionality: high-level foothold planning for a crawl gait on challenging plum piles terrain. The planner:

- Plans a rolling horizon of **4 steps** ahead
- Selects safe footholds on cylindrical pile tops (radius 0.08m, height 0.268m)
- Provides constraint regions to ensure NMPC and heightmap adaptation stay within pile boundaries
- Gracefully falls back to heuristic planning if sl1m is not installed
- Reserves interfaces for **Mode B** (contact schedule generation) for future extension

## Installation

### Required Dependencies

```bash
# Core dependencies (already in pyproject.toml)
pip install numpy scipy

# Optional: sl1m for optimization-based planning
# (Without sl1m, the planner uses a simple heuristic fallback)
pip install git+https://github.com/loco-3d/sl1m.git

# Optional: solvers for sl1m
pip install cvxpy
pip install quadprog  # optional, for specific solvers
```

### Verify Installation

```python
# Check if planner modules are available
from quadruped_pympc.high_level_planners.sl1m_planner import Sl1mFootholdPlanner
print("✓ sl1m planner module available")
```

## Configuration

### 1. Enable the High-Level Planner

Edit `quadruped_pympc/config.py`:

```python
simulation_params = {
    # ... other params ...
    
    # Enable high-level planner
    'high_level_planner': {
        'enabled': True,  # Set to True to activate planner
        'planner_type': 'sl1m',
        'planning_horizon': 4,  # Number of future steps
        'use_optimization': True,  # Use sl1m if available, else heuristic
        'mode_b_enabled': False,  # Mode A: footholds only (set True for Mode B)
        'visualize_footholds': True,  # Show colored foothold markers
        
        'plum_piles': {
            'enabled': True,  # Activate plum piles mode
            'x_range': (4.1, 7.1),  # Grid extent
            'y_range': (-0.4, 0.4),
            'x_step': 0.2,  # Pile spacing
            'y_step': 0.2,
            'radius': 0.08,  # Pile radius (m)
            'height': 0.268,  # Pile top height (m)
            'constraint_radius': 0.06,  # Safety margin (m)
        },
    },
}
```

### 2. Set Gait to Crawl

The planner is designed for **crawl gait** (one leg swings at a time):

```python
simulation_params = {
    # ... other params ...
    'gait': 'crawl',  # Required for sl1m planner
}
```

### 3. Configure MPC for Foothold Optimization

Ensure foothold optimization is enabled:

```python
mpc_params = {
    # ... other params ...
    'use_foothold_optimization': True,
    # use_foothold_constraints will be auto-enabled when plum_piles mode is active
}
```

### 4. Load Plum Piles Terrain

Option A: Use pre-generated terrain XML

```python
simulation_params = {
    'scene': '/path/to/plum_piles_terrain.xml',
}
```

Option B: Generate terrain dynamically

```bash
cd simulation
python3 plum_piles_scene.py
# This creates plum_piles_terrain.xml with 80 piles
```

## Usage

### Running a Simulation

```bash
cd simulation
python3 simulation.py
```

With the planner enabled, you should see console output:

```
High-level planner enabled: sl1m foothold planner for plum piles terrain
  Planning horizon: 4 steps
  Pile grid: x=[4.1, 7.1], y=[-0.4, 0.4]
  Constraint radius: 0.06m
```

### Configuration Examples

#### Conservative (Safer)

```python
'high_level_planner': {
    'plum_piles': {
        'constraint_radius': 0.05,  # Tighter safety margin
    },
}
```

#### Aggressive (Faster)

```python
'high_level_planner': {
    'planning_horizon': 6,  # Plan further ahead
    'plum_piles': {
        'constraint_radius': 0.07,  # Use more of pile surface
    },
}
```

## How It Works

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│ WBInterface.update_state_and_reference()                │
├─────────────────────────────────────────────────────────┤
│ 1. Periodic Gait Generator (PGG)                        │
│    └─> Contact sequence (Mode A: from PGG, not sl1m)   │
│                                                          │
│ 2. Foothold Reference Generator (FRG)                   │
│    └─> Nominal footholds (geometric heuristic)          │
│                                                          │
│ 3. *** High-Level Planner (sl1m) ***                    │
│    └─> Override XY with planned pile positions          │
│    └─> Set Z=0 (heightmap will set actual pile height)  │
│    └─> Generate constraint regions (pile top circles)   │
│                                                          │
│ 4. Visual Foothold Adaptation (VFA)                     │
│    └─> Update Z coordinate from heightmap               │
│    └─> Adapt footholds if needed                        │
│    └─> Clamp XY adaptations to constraint regions       │
│                                                          │
│ 5. Swing Trajectory Generator                           │
│    └─> Generate swing trajectory with step_height       │
│    └─> Apex = max(lift_off.z, touch_down.z) + step_height │
│                                                          │
│ 6. Pass to NMPC with foothold constraints               │
└─────────────────────────────────────────────────────────┘
```

**Important Note**: The planner sets foothold XY coordinates only. The Z coordinate is initialized to ground level (0.0) and later adjusted by the heightmap/terrain adaptation. This prevents the swing generator from lifting the leg too high, as it adds `step_height` on top of the maximum Z between lift-off and touch-down positions.

### Mode A: Foothold Planning Only

Currently operational. The planner:
- Receives current robot state (position, velocity, contact)
- Plans footholds for next 4 swing events
- Returns foothold for currently swinging leg + constraint region
- Contact schedule still comes from PGG (crawl gait pattern)

**Configuration**:
```python
'high_level_planner': {
    'enabled': True,
    'mode_b_enabled': False,  # Mode A
}
```

### Mode B: Full Contact Schedule Planning

**Now Implemented!** The planner generates both footholds AND contact schedules.

The planner:
- Plans footholds for next 4 steps
- Generates contact schedule (which leg swings at each step)
- For crawl gait: FL → RR → FR → RL sequence
- Returns contact sequence + phase timing information
- Overrides PGG contact schedule in WBInterface

**Configuration**:
```python
'high_level_planner': {
    'enabled': True,
    'mode_b_enabled': True,  # Enable Mode B
    'planning_horizon': 4,   # Number of steps to plan
}
```

**Contact Schedule Format**:
- `contact_schedule`: List of 4 arrays (one per leg: FL, FR, RL, RR)
- Each array shape: `(horizon,)` with binary values (1=contact, 0=swing)
- `phase_schedule`: Dict with timing info (duty_factor=0.8, swing_duration=0.5s, step_freq=0.5Hz)

**Example**: For 4-step horizon:
```
Step 0: [0, 1, 1, 1]  # FL swings
Step 1: [1, 1, 1, 0]  # RR swings  
Step 2: [1, 0, 1, 1]  # FR swings
Step 3: [1, 1, 0, 1]  # RL swings
```

### Foothold Visualization

**Now Available!** Planned footholds are visualized as colored spheres in MuJoCo.

**Features**:
- Real-time visualization of planned footholds
- Color-coded by leg:
  - **FL**: Red 🔴
  - **FR**: Green 🟢
  - **RL**: Blue 🔵
  - **RR**: Yellow 🟡
- 3cm radius spheres for visibility
- Updates every planning cycle

**Configuration**:
```python
'high_level_planner': {
    'enabled': True,
    'visualize_footholds': True,  # Enable visualization (default: True)
}
```

**In Simulation**: Visualization is automatic when planner is enabled. The spheres show where the robot plans to step next.

### Mode Comparison

| Feature | Mode A | Mode B |
|---------|--------|--------|
| Foothold Planning | ✓ | ✓ |
| Constraint Regions | ✓ | ✓ |
| Contact Schedule | From PGG | From Planner |
| Phase Planning | No | Yes |
| Gait Adaptation | Limited | Full |
| Visualization | ✓ | ✓ |

**When to use Mode B**:
- Complex terrain requiring adaptive gait
- Need precise swing timing control
- Optimizing step sequence for efficiency
- Research on contact planning algorithms

**When Mode A is sufficient**:
- Standard crawl gait works well
- Terrain is predictable
- Simpler implementation
- Current default configuration

## Planner Behavior

### With sl1m Installed (Optimization Mode)

- Uses sl1m's combinatorial L1 solver (when fully integrated)
- Optimizes foothold selection over multiple piles
- Considers kinematic constraints and robot dynamics
- Currently: Placeholder calls heuristic (sl1m API integration pending)

### Without sl1m (Heuristic Fallback)

- Simple "nearest reachable pile" heuristic
- Projects future base position using reference velocity
- Selects pile closest to nominal foothold location
- Always functional, no external dependencies

### Constraint Enforcement

The planner ensures safe footholds via:

1. **Constraint regions**: Each planned foothold gets a circular constraint (radius 0.06m)
2. **Heightmap adaptation clipping**: VFA adaptations are clamped to constraint region
3. **NMPC constraints**: `ref_feet_constraints` passed to MPC prevents foothold optimization from moving off piles

## Terrain Customization

### Adjusting Pile Grid

Edit `simulation/plum_piles_scene.py`:

```python
save_plum_piles_scene(
    "custom_piles.xml",
    plum_piles_x_start=3.0,  # Start further back
    plum_piles_x_end=8.0,    # Extend further forward
    pile_x_step=0.15,        # Denser grid
    pile_radius=0.10,        # Larger piles
)
```

Then update config.py to match:

```python
'plum_piles': {
    'x_range': (3.0, 8.0),
    'x_step': 0.15,
    'radius': 0.10,
    'constraint_radius': 0.08,
}
```

### Adding Ramps/Platforms

The plum piles scene includes:
- Flat starting area (robot spawn)
- Ramp to elevated platform (~0.536m)
- Dense pile grid
- Safety ground plane

Modify `create_plum_piles_scene()` parameters to adjust.

## Troubleshooting

### Planner Not Running

Check console output. If you see:

```
High-level planner enabled but module not available. Continuing without planner.
```

Solution: Verify `high_level_planners` package is importable:
```python
from quadruped_pympc.high_level_planners.sl1m_planner import Sl1mFootholdPlanner
```

### Legs Lifting Too High

**Symptoms**: Robot legs lift extremely high during swing phase (much higher than step_height)

**Root Cause**: This was an issue in earlier versions where the planner set the foothold Z coordinate to the absolute pile height (0.268m). The swing trajectory generator then added step_height on top of this, causing apex heights of 0.268 + step_height ≈ 0.33m.

**Solution** (Fixed in current version):
- The planner now sets foothold Z to ground level (0.0)
- Heightmap adaptation updates Z to actual pile height during planning
- Swing apex is correctly calculated as max(lift_off.z, touch_down.z) + step_height

**If you still experience this**:
1. Verify you're using the latest version of the planner
2. Check `step_height` in config.py: should be `0.2 * hip_height` (about 0.06m for Aliengo)
3. Ensure heightmap adaptation is enabled: `visual_foothold_adaptation` should not be 'blind'
4. Check console output for any warnings about planner failures

### Footholds Off Pile Tops

Symptoms: Robot stepping off piles, falling

Solutions:
1. Reduce `constraint_radius` (more conservative)
2. Check terrain XML matches config (x_range, y_range, spacing)
3. Verify VFA constraint enforcement is active (check `_apply_hl_planner_constraints` is called)

### Robot Stuck / Not Moving

Symptoms: Robot stands still or makes no progress

Solutions:
1. Verify gait is set to 'crawl' (not 'trot' or 'full_stance')
2. Check planning horizon (try 4-6 steps)
3. Ensure reference velocity is non-zero
4. Check pile grid is reachable from start position

### sl1m Import Errors

If you see warnings about sl1m not found:

```
sl1m package not found. Sl1mFootholdPlanner will use heuristic fallback.
```

This is expected if sl1m is not installed. The planner will use the heuristic method, which works but is less optimal.

To install sl1m:
```bash
pip install git+https://github.com/loco-3d/sl1m.git
```

## Performance Tips

- **Planning horizon**: 4 steps is a good default. Higher values (6-8) improve lookahead but increase computation
- **Constraint radius**: Balance between safety (smaller) and success (larger). Start with 0.06m
- **Pile spacing**: 0.2m is challenging but traversable. Increase to 0.3m for easier terrain
- **Reference velocity**: Keep moderate (0.2-0.4 m/s) for crawl gait on piles

## Example: Complete Configuration

### Mode A (Foothold Planning Only)

```python
# config.py
simulation_params = {
    'gait': 'crawl',
    'scene': 'simulation/plum_piles_terrain.xml',
    'visual_foothold_adaptation': 'height',  # or 'tamols' for TAMOLS adaptation
    
    'high_level_planner': {
        'enabled': True,
        'planner_type': 'sl1m',
        'planning_horizon': 4,
        'use_optimization': True,
        'mode_b_enabled': False,  # Mode A: footholds only
        'visualize_footholds': True,
        
        'plum_piles': {
            'enabled': True,
            'x_range': (4.1, 7.1),
            'y_range': (-0.4, 0.4),
            'x_step': 0.2,
            'y_step': 0.2,
            'radius': 0.08,
            'height': 0.268,
            'constraint_radius': 0.06,
        },
    },
}

mpc_params = {
    'use_foothold_optimization': True,
    # use_foothold_constraints auto-enabled
}
```

### Mode B (Full Contact + Foothold Planning)

```python
# config.py  
simulation_params = {
    'gait': 'crawl',
    'scene': 'simulation/plum_piles_terrain.xml',
    'visual_foothold_adaptation': 'height',
    
    'high_level_planner': {
        'enabled': True,
        'planner_type': 'sl1m',
        'planning_horizon': 4,
        'use_optimization': True,
        'mode_b_enabled': True,  # Mode B: contact schedule + footholds
        'visualize_footholds': True,
        
        'plum_piles': {
            'enabled': True,
            'x_range': (4.1, 7.1),
            'y_range': (-0.4, 0.4),
            'x_step': 0.2,
            'y_step': 0.2,
            'radius': 0.08,
            'height': 0.268,
            'constraint_radius': 0.06,
        },
    },
}

mpc_params = {
    'use_foothold_optimization': True,
}
```

**Key Difference**: `mode_b_enabled: True` enables contact schedule generation from planner instead of PGG.

## Testing

Basic smoke test:

```python
from quadruped_pympc.high_level_planners.sl1m_planner import Sl1mFootholdPlanner
import numpy as np

pile_config = {
    'x_range': (4.1, 7.1),
    'y_range': (-0.4, 0.4),
    'x_step': 0.2,
    'y_step': 0.2,
    'radius': 0.08,
    'height': 0.268,
    'constraint_radius': 0.06,
}

planner = Sl1mFootholdPlanner(pile_config=pile_config, planning_horizon=4, use_sl1m=False)
print(f"✓ Planner created with {len(planner.piles)} piles")
```

## References

- sl1m repository: https://github.com/loco-3d/sl1m
- Problem formulation: Combinatorial foothold planning on discrete surface patches
- This implementation: Mode A (footholds) with Mode B (contact schedule) interfaces reserved

## Support

For issues or questions:
1. Check this documentation
2. Verify configuration matches examples
3. Test with heuristic mode first (`use_optimization: False`)
4. Check console output for warnings/errors
