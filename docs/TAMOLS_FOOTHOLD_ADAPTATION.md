# TAMOLS-Inspired Foothold Adaptation

## Overview

The TAMOLS (Terrain-Aware Model-based Optimal Locomotion Strategy) foothold adaptation strategy provides terrain-aware foothold selection for quadruped locomotion. It is inspired by the reference implementation in `ianpedroza/tamols-rl`.

## How It Works

The TAMOLS strategy performs a local search around each leg's seed foothold position and selects the best candidate based on multiple terrain-aware cost metrics:

### 1. Candidate Generation
- Generates a grid of candidate footholds in XY around the seed position
- Search radius and resolution are configurable
- Queries heightmap to set Z coordinate for each candidate

### 2. Cost Metrics

The total cost for each candidate combines six TAMOLS-inspired metrics based on the reference implementation (`ianpedroza/tamols-rl`):

#### Edge Avoidance
- **Purpose**: Avoid placing footholds near edges, steps, or high-gradient terrain
- **Implementation**: Uses finite differences to estimate local terrain gradient magnitude
- **TAMOLS Reference**: `tamols/costs.py:add_edge_avoidance_cost` (lines 164-185)
- **Weight Parameter**: `weight_edge_avoidance` (default: 5.0)

#### Roughness Penalty
- **Purpose**: Prefer smooth, flat terrain over rough/irregular surfaces
- **Implementation**: Computes height variance over a small patch around the candidate
- **TAMOLS Reference**: Related to terrain smoothing in `tamols/map_processing.py`
- **Weight Parameter**: `weight_roughness` (default: 2.0)

#### Previous Solution Tracking
- **Purpose**: Minimize deviation from the seed foothold (previous solution)
- **Implementation**: Penalizes squared distance from candidate to seed: `||candidate - seed||²`
- **TAMOLS Reference**: `tamols/costs.py:add_previous_solution_cost` (lines 187-215)
- **Weight Parameter**: `weight_previous_solution` (default: 0.01)

#### Kinematic Reachability
- **Purpose**: Ensure foothold is within leg's kinematic reach
- **Implementation**: Enforces `l_min ≤ ||hip - foot|| ≤ l_max` via quadratic penalty
- **TAMOLS Reference**: `tamols/constraints.py:add_kinematic_constraints` (lines 130-154)
- **Weight Parameter**: `weight_kinematic` (default: 10.0)

#### Nominal Kinematic Cost (GIA) **UPDATED**
- **Purpose**: Maintain desired hip height and nominal leg configuration
- **Implementation**: Penalizes deviation from desired base-to-foot vector: `||hip - (candidate - [0,0,-h_des])||²`
- **TAMOLS Reference**: `tamols/costs.py:add_nominal_kinematic_cost` (lines 101-130)
- **GIA Principle**: Maintains consistent kinematic configuration independent of gait phase
- **Weight Parameter**: `weight_nominal_kinematic` (default: 20.0)
- **Configuration**: `h_des` - desired hip height (default: 0.25m for go1/go2, 0.30m for aliengo)

#### Reference Tracking Cost (GIA, Anti-Conservative) **UPDATED**
- **Purpose**: Track reference velocity direction, preventing standing still
- **Implementation**: Penalizes footholds that oppose desired velocity direction (X-direction focus like TAMOLS)
- **TAMOLS Reference**: `tamols/costs.py:add_tracking_cost` (lines 13-34)
- **GIA Principle**: Supports motion in desired direction independent of gait timing
- **Anti-Conservative**: Prevents robot from stalling or moving backward
- **Weight Parameter**: `weight_reference_tracking` (default: 2.0)

#### Swing Clearance Cost **NEW**
- **Purpose**: Prevent swing leg collisions with terrain during leg swing
- **Implementation**: Samples points along straight-line swing path from current position to candidate foothold, penalizes insufficient clearance above terrain
- **Use Case**: Critical for avoiding collisions with step edges, obstacles, and uneven terrain during swing phase
- **Weight Parameter**: `weight_swing_clearance` (default: 8.0)
- **Configuration**: `swing_safety_margin` (default: 0.05m), `swing_path_samples` (default: 10)

### 3. Candidate Selection
- Evaluates all candidates using the cost function
- Selects candidate with minimum total cost
- Falls back to simple height adjustment if no valid candidates found

### 4. Foothold Constraints (Optional)
- Generates a simple box constraint around the chosen foothold
- Provides `[vertex1, vertex2]` representing opposite corners of the box
- Can be used by MPC to constrain foothold optimization

## Configuration

Add TAMOLS configuration to `quadruped_pympc/config.py`:

```python
simulation_params = {
    # Set strategy to 'tamols'
    "visual_foothold_adaptation": 'tamols',
    
    # TAMOLS parameters
    'tamols_params': {
        # Candidate generation
        'search_radius': 0.15,        # [m] search radius around seed
        'search_resolution': 0.03,    # [m] grid step size
        'patch_size': 3,              # heightmap patch size for gradient estimation
        
        # Cost weights (tune these for your terrain/robot)
        'weight_edge_avoidance': 5.0,         # from tamols/costs.py:add_edge_avoidance_cost
        'weight_roughness': 2.0,              # terrain roughness penalty
        'weight_previous_solution': 0.01,     # from tamols/costs.py:add_previous_solution_cost
        'weight_kinematic': 10.0,             # from tamols/constraints.py:add_kinematic_constraints
        'weight_nominal_kinematic': 20.0,     # from tamols/costs.py:add_nominal_kinematic_cost (GIA)
        'weight_reference_tracking': 2.0,     # from tamols/costs.py:add_tracking_cost (GIA, anti-conservative)
        'weight_swing_clearance': 8.0,        # swing leg collision avoidance (NEW)
        
        # Nominal kinematic parameters
        'h_des': 0.25,                        # [m] desired hip height (go1/go2: 0.25, aliengo: 0.30)
        
        # Swing clearance parameters (NEW)
        'swing_safety_margin': 0.05,          # [m] minimum clearance above terrain during swing
        'swing_path_samples': 10,             # number of points sampled along swing path
        
        # Adaptive step height (NEW)
        'adaptive_step_height': True,         # enable dynamic step height adjustment
        'base_step_height': None,             # [m] base step height (None = use simulation default)
        'step_height_gain': 0.5,              # gain for roughness-based height adjustment
        'max_step_height_multiplier': 2.0,    # maximum step height multiplier
        
        # Kinematic bounds per robot (distance from hip to foothold)
        'l_min': {'go1': 0.15, 'go2': 0.15, 'aliengo': 0.18, ...},
        'l_max': {'go1': 0.45, 'go2': 0.45, 'aliengo': 0.55, ...},
        
        # Foothold constraint box (for MPC)
        'constraint_box_dx': 0.05,     # [m] +/- x around chosen foothold
        'constraint_box_dy': 0.05,     # [m] +/- y around chosen foothold
    },
}
```

## Tuning Guide

### For Aggressive Edge Avoidance
- Increase `weight_edge_avoidance` (e.g., 10.0)
- Decrease `search_resolution` for finer search (e.g., 0.02)

### For Smoother Trajectories
- Increase `weight_previous_solution` to stay closer to seed footholds
- Decrease `search_radius` to limit foothold variation

### For Rough Terrain
- Increase `weight_roughness` to prefer flat areas
- Increase `patch_size` for more robust roughness estimation

### For Kinematic Safety
- Ensure `l_min` and `l_max` match your robot's actual leg reach
- Increase `weight_kinematic` if candidates outside reach are still selected
- Adjust `h_des` to match your robot's nominal hip height

### To Prevent Conservative Behavior (Anti-Standing Still)
- Increase `weight_reference_tracking` (e.g., 5.0 or higher)
- This penalizes footholds that oppose desired velocity direction
- Useful when robot tends to stop or take very small steps
- **Caution**: Too high values may cause instability on difficult terrain

### For GIA Compliance (Gait Independent Adaptation)
- Increase `weight_nominal_kinematic` (e.g., 30.0-40.0) to strongly maintain hip height
- Increase `weight_reference_tracking` (e.g., 3.0-5.0) to align with velocity
- These weights help maintain stable, gait-independent kinematic configuration
- **Balance**: Higher GIA weights improve gait stability but reduce terrain adaptability

### Recommended Weight Combinations

**TAMOLS Reference Mode** (matches reference implementation):
```python
'weight_edge_avoidance': 5.0,         # ~3 in TAMOLS
'weight_roughness': 2.0,
'weight_previous_solution': 0.01,     # 0.01 in TAMOLS
'weight_kinematic': 10.0,
'weight_nominal_kinematic': 20.0,     # 20 in TAMOLS
'weight_reference_tracking': 2.0,     # ~2 in TAMOLS (tracking cost weight)
'h_des': 0.25,                        # 0.25 in TAMOLS for go2
```

**Conservative Mode** (safer, smoother):
```python
'weight_edge_avoidance': 8.0,
'weight_roughness': 3.0,
'weight_previous_solution': 0.02,      # Higher - stays closer to seed
'weight_kinematic': 15.0,
'weight_nominal_kinematic': 10.0,      # Lower - more terrain adaptation
'weight_reference_tracking': 1.0,      # Lower - allows careful stepping
```

**Aggressive Mode** (faster, more progress):
```python
'weight_edge_avoidance': 3.0,
'weight_roughness': 1.0,
'weight_previous_solution': 0.005,     # Lower - more exploration
'weight_kinematic': 10.0,
'weight_nominal_kinematic': 15.0,
'weight_reference_tracking': 5.0,      # Higher - strong forward bias
```

**GIA-Optimized Mode** (stable gaits, balanced):
```python
'weight_edge_avoidance': 5.0,
'weight_roughness': 2.0,
'weight_previous_solution': 0.01,
'weight_kinematic': 10.0,
'weight_nominal_kinematic': 30.0,      # High - maintains hip height
'weight_reference_tracking': 3.0,      # High - tracks velocity direction
'h_des': 0.25,                         # Tune for your robot
```

### Performance Tuning
- Decrease `search_radius` or increase `search_resolution` to reduce candidate count
- Current defaults: ~81 candidates per leg, reasonable for real-time control

## Swing Collision Avoidance **NEW**

### Problem

During swing phase, the leg moves through the air from current position to target foothold. On challenging terrain (steps, obstacles, uneven ground), the swing trajectory may intersect terrain features, causing:
- **Collisions with step edges** during upward/downward transitions
- **Contact with obstacles** along the path
- **Premature ground contact** on irregular terrain

### Solution

The implementation provides two complementary approaches:

#### 1. Swing Clearance Cost (Foothold Selection)
Evaluates each candidate foothold by checking if the swing path would collide with terrain:
- Samples N points along straight-line path from current position to candidate
- Checks clearance above terrain at each sample point
- Penalizes candidates requiring low-clearance swings
- **Weight**: `weight_swing_clearance` (default: 8.0)
- **Margin**: `swing_safety_margin` (default: 0.05m above terrain)

#### 2. Adaptive Step Height (Swing Trajectory)
Dynamically adjusts swing apex height based on terrain difficulty:
- Estimates terrain roughness from heightmap variance
- Increases step height proportionally: `height = base + roughness × gain`
- Clamps to maximum: `base × max_multiplier`
- **Parameters**: `step_height_gain` (0.5), `max_step_height_multiplier` (2.0)

### Usage

Enable both features in configuration:
```python
'tamols_params': {
    # Swing clearance cost
    'weight_swing_clearance': 8.0,        # Penalize collision-prone footholds
    'swing_safety_margin': 0.05,          # Minimum clearance [m]
    'swing_path_samples': 10,             # Path sampling density
    
    # Adaptive step height
    'adaptive_step_height': True,         # Enable dynamic adjustment
    'step_height_gain': 0.5,              # Roughness sensitivity
    'max_step_height_multiplier': 2.0,    # Maximum height increase
}
```

### Integration with Swing Controller

The adaptive step height is exposed via:
```python
adaptive_height = vfa.get_adaptive_step_height(base_step_height)
# Use adaptive_height for swing trajectory generation
```

Integrate this in your swing controller or main loop to update step height based on terrain.

### Tuning Guide

**Increase `weight_swing_clearance`** (8.0 → 12.0) if:
- Swing leg still hits obstacles
- Robot frequently trips on step edges

**Increase `swing_safety_margin`** (0.05 → 0.08) if:
- Clearance is insufficient on rough terrain
- Robot grazes obstacles during swing

**Increase `step_height_gain`** (0.5 → 0.8) if:
- Adaptive height doesn't increase enough on difficult terrain
- Robot needs more clearance on obstacles

**Decrease `step_height_gain`** (0.5 → 0.3) if:
- Step height increases too aggressively
- Robot wastes energy with unnecessarily high swings

## Usage Example

```python
from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
from quadruped_pympc import config as cfg

# Set TAMOLS strategy
cfg.simulation_params['visual_foothold_adaptation'] = 'tamols'

# Initialize with TAMOLS
vfa = VisualFootholdAdaptation(
    legs_order=['FL', 'FR', 'RL', 'RR'],
    adaptation_strategy='tamols'
)

# Compute adaptation (called during swing apex)
success = vfa.compute_adaptation(
    legs_order=legs_order,
    reference_footholds=reference_footholds,  # seed positions
    hip_positions=hip_positions,
    heightmaps=heightmaps,  # from HeightMap sensor
    forward_vel=base_lin_vel,
    base_orientation=base_ori_euler,
    base_orientation_rate=base_ang_vel
)

# Get adapted footholds
adapted_footholds, constraints = vfa.get_footholds_adapted(reference_footholds)
```

## GIA Principles (Gait Independent Adaptation)

Based on the TAMOLS reference implementation (`ianpedroza/tamols-rl`), this implementation supports **Gait Independent Adaptation (GIA)** principles.

### What is GIA?

GIA (also known as GIAC - Gait Independent Angular momentum Control in the reference) is a control approach where:
1. **Foothold selection** is based on terrain and kinematics, not gait phase
2. **Dynamic stability** is maintained through proper angular momentum management
3. **Kinematic configuration** remains consistent independent of gait timing

### GIA in TAMOLS Reference

The TAMOLS reference (`go2-hrl/fetch/tamols/`) implements GIA through:

1. **Dynamics Constraints** (`constraints.py:add_dynamics_constraints`):
   - GIAC stability constraints: `m·det(p_ij, p_B - p_i, a_B) - p_ij·L_dot_B ≤ ε`
   - Friction cone constraints ensuring stability
   - Enforced continuously independent of gait phase

2. **Nominal Kinematic Cost** (`costs.py:add_nominal_kinematic_cost`):
   - Maintains desired hip height `h_des`
   - Penalizes: `||base + R_B·hip_offset - l_des - p_i||²`
   - Keeps legs in nominal configuration independent of gait

3. **Tracking Cost** (`costs.py:add_tracking_cost`):
   - Tracks reference velocity (X direction only in TAMOLS)
   - Ensures forward progress independent of gait timing
   - Weight: 2·T_k / tau_sampling_rate per sample

### GIA in Our Implementation

Since our foothold planner is simpler (no full trajectory optimization), we adapt GIA principles:

1. **Nominal Kinematic Cost** (`weight_nominal_kinematic`):
   - From TAMOLS: `costs.py:add_nominal_kinematic_cost`
   - Maintains desired hip height `h_des`
   - Penalizes: `||hip - (candidate - [0,0,-h_des])||²`
   - **Effect**: Keeps robot at consistent height independent of gait phase
   - **Weight**: 20.0 (matches TAMOLS default)

2. **Reference Tracking Cost** (`weight_reference_tracking`):
   - From TAMOLS: `costs.py:add_tracking_cost`
   - Tracks desired velocity direction (X-direction focus)
   - Penalizes footholds opposing motion direction
   - **Effect**: Ensures forward progress, prevents standing still
   - **Anti-Conservative**: Avoids local minima where robot stops
   - **Weight**: 2.0 (matches TAMOLS tracking weight)

### Differences from Full TAMOLS

Our implementation adapts TAMOLS for foothold selection (vs. full trajectory optimization):

**Simplified**:
- No spline-based trajectory optimization
- No full GIAC dynamics constraints (would require trajectory)
- No angular momentum derivative computation

**Retained**:
- ✓ Kinematic reachability constraints (`l_min/l_max`)
- ✓ Nominal kinematic cost (hip height maintenance)
- ✓ Reference tracking (velocity direction)
- ✓ Edge avoidance (gradient-based)
- ✓ Previous solution tracking
- ✓ Foothold-on-ground (implicit via heightmap query)

### Tuning for GIA

To emphasize GIA compliance:
- Set `weight_nominal_kinematic` high (30.0-40.0) to maintain hip height
- Set `weight_reference_tracking` moderate-high (3.0-5.0) to track velocity
- Ensure `h_des` matches your robot's nominal hip height
- Balance with terrain safety weights

**Example GIA-optimized configuration:**
```python
'tamols_params': {
    'weight_edge_avoidance': 5.0,
    'weight_roughness': 2.0,
    'weight_previous_solution': 0.01,
    'weight_kinematic': 10.0,
    'weight_nominal_kinematic': 30.0,    # High - maintains hip height (GIA)
    'weight_reference_tracking': 3.0,    # High - tracks velocity (GIA)
    'h_des': 0.25,                       # Tune for your robot
}
```

### Reference

For full TAMOLS implementation with GIAC dynamics constraints, see:
- https://github.com/ianpedroza/tamols-rl/tree/main/go2-hrl/fetch/tamols
- `constraints.py`: GIAC constraints and kinematic bounds
- `costs.py`: All cost functions including nominal kinematics and tracking
- `map_processing.py`: Heightmap processing and gradient computation

## Backward Compatibility

The implementation is fully backward compatible with existing strategies:
- `'blind'`: No adaptation, uses seed footholds directly
- `'height'`: Simple height adjustment from heightmap
- `'vfa'`: Uses VFA neural network (if installed)
- `'tamols'`: New TAMOLS-inspired strategy with GIA support

Existing code continues to work without modification. Only enable TAMOLS by changing the config.

## Testing

Run the test suite to verify TAMOLS functionality:

```bash
python test_tamols_foothold_adaptation.py
```

Tests cover:
1. Candidate generation
2. Edge avoidance on synthetic step terrain
3. Kinematic constraint enforcement
4. Backward compatibility with existing strategies

## Performance Considerations

- **Candidate count**: With default parameters (radius=0.15m, resolution=0.03m), ~81 candidates per leg
- **Heightmap queries**: ~85 queries per leg per adaptation (1 per candidate + gradient/roughness sampling)
- **Computation time**: Should remain under 10ms per adaptation on modern hardware
- **Trigger frequency**: Only at swing apex (not every control cycle), minimizing overhead

## Mapping to TAMOLS Reference Implementation

| TAMOLS Component | This Implementation |
|------------------|---------------------|
| `map_processing.py` smoothing | Heightmap provides smoothed terrain data |
| `map_processing.py` gradient computation | `_compute_edge_cost()` using finite differences |
| `costs.py` edge avoidance | `weight_edge_avoidance` cost term |
| `costs.py` foothold-on-ground | Implicit (z set from heightmap query) |
| `costs.py` nominal kinematics | `weight_kinematic` cost term |
| `costs.py` previous solution | `weight_deviation` cost term |
| `constraints.py` hip-foot distance | `l_min`, `l_max` bounds in kinematic cost |

## Future Enhancements

Potential improvements for future development:
1. **Adaptive search radius**: Vary search area based on terrain complexity
2. **Multi-step lookahead**: Consider future footsteps in scoring
3. **Contact surface area**: Prefer larger flat regions for better contact
4. **Dynamic cost weights**: Adjust weights based on gait phase or velocity
5. **GPU acceleration**: Parallelize candidate evaluation for very large search spaces
6. **Learned terrain costs**: Integrate learned terrain difficulty predictor
7. **Configurable sampling deltas**: Make finite difference step sizes (currently hardcoded at 0.02m and 0.03m) configurable for different heightmap resolutions and terrain scales
8. **Adaptive fallback costs**: Make fallback cost values (when heightmap queries fail) configurable or derived from other cost parameters
