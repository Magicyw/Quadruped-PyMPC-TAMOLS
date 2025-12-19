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

The total cost for each candidate combines seven TAMOLS-inspired metrics:

#### Edge Avoidance
- **Purpose**: Avoid placing footholds near edges, steps, or high-gradient terrain
- **Implementation**: Uses finite differences to estimate local terrain gradient magnitude
- **Mapping to TAMOLS**: Corresponds to gradient-based edge detection in `tamols/map_processing.py`

#### Roughness Penalty
- **Purpose**: Prefer smooth, flat terrain over rough/irregular surfaces
- **Implementation**: Computes height variance over a small patch around the candidate
- **Mapping to TAMOLS**: Related to terrain smoothing and local planarity checks

#### Previous Solution Tracking
- **Purpose**: Minimize deviation from the seed foothold (reference trajectory)
- **Implementation**: Penalizes Euclidean distance from candidate to seed in XY
- **Mapping to TAMOLS**: Corresponds to tracking cost in `tamols/costs.py`

#### Kinematic Reachability
- **Purpose**: Ensure foothold is within leg's kinematic reach
- **Implementation**: Checks hip-to-foothold distance against `l_min` and `l_max` bounds
- **Mapping to TAMOLS**: Corresponds to kinematic constraints in `tamols/constraints.py`

#### Forward Progress (Anti-Conservative) **NEW**
- **Purpose**: Prevent overly conservative behavior and standing still
- **Implementation**: Penalizes footholds that don't contribute to forward motion relative to hip
- **Effect**: Encourages robot to keep making progress, avoiding local minima where robot stops moving
- **Weight Parameter**: `weight_forward_progress` (default: 3.0)

#### Velocity Alignment (GIA Principle) **NEW**
- **Purpose**: Support Gait Independent Adaptation (GIA) by aligning footholds with motion direction
- **Implementation**: Encourages foothold displacement aligned with robot's current velocity vector
- **Effect**: Footholds naturally follow the direction of motion, independent of gait timing
- **Weight Parameter**: `weight_velocity_alignment` (default: 2.0)

#### Step Consistency (GIA Principle) **NEW**
- **Purpose**: Maintain consistent step patterns across all legs for stable periodic gaits
- **Implementation**: Minimizes variance in step lengths across legs
- **Effect**: Promotes symmetric, balanced gait patterns that support GIA principles
- **Weight Parameter**: `weight_step_consistency` (default: 1.5)

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
        'weight_edge_avoidance': 5.0,       # penalize high gradients
        'weight_roughness': 2.0,            # penalize rough terrain
        'weight_deviation': 1.0,            # penalize deviation from seed
        'weight_kinematic': 10.0,           # penalize unreachable positions
        'weight_forward_progress': 3.0,     # penalize lack of forward motion (anti-conservative)
        'weight_velocity_alignment': 2.0,   # encourage velocity-aligned footholds (GIA)
        'weight_step_consistency': 1.5,     # maintain consistent step patterns (GIA)
        
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
- Increase `weight_deviation` to stay closer to nominal footholds
- Decrease `search_radius` to limit foothold variation

### For Rough Terrain
- Increase `weight_roughness` to prefer flat areas
- Increase `patch_size` for more robust roughness estimation

### For Kinematic Safety
- Ensure `l_min` and `l_max` match your robot's actual leg reach
- Increase `weight_kinematic` if candidates outside reach are still selected

### To Prevent Conservative Behavior (Anti-Standing Still)
- Increase `weight_forward_progress` (e.g., 5.0 or higher)
- This penalizes footholds that don't move forward
- Useful when robot tends to stop or take very small steps
- **Caution**: Too high values may cause instability on difficult terrain

### For GIA Compliance (Gait Independent Adaptation)
- Increase `weight_velocity_alignment` (e.g., 3.0-4.0) to strongly align with velocity
- Increase `weight_step_consistency` (e.g., 2.0-3.0) to maintain symmetric gaits
- These weights help maintain stable, periodic gaits independent of specific timing
- **Balance**: Higher GIA weights improve gait stability but reduce terrain adaptability

### Recommended Weight Combinations

**Conservative Mode** (safer, smoother):
```python
'weight_edge_avoidance': 8.0,
'weight_roughness': 3.0,
'weight_deviation': 2.0,
'weight_forward_progress': 1.0,  # Low - allows careful stepping
'weight_velocity_alignment': 1.0,
'weight_step_consistency': 1.0,
```

**Aggressive Mode** (faster, more progress):
```python
'weight_edge_avoidance': 4.0,
'weight_roughness': 1.5,
'weight_deviation': 0.5,
'weight_forward_progress': 5.0,  # High - encourages forward motion
'weight_velocity_alignment': 3.0,
'weight_step_consistency': 2.0,
```

**GIA-Optimized Mode** (stable gaits, balanced):
```python
'weight_edge_avoidance': 5.0,
'weight_roughness': 2.0,
'weight_deviation': 1.0,
'weight_forward_progress': 3.0,
'weight_velocity_alignment': 4.0,  # High - strong velocity alignment
'weight_step_consistency': 3.0,    # High - symmetric gait
```

### Performance Tuning
- Decrease `search_radius` or increase `search_resolution` to reduce candidate count
- Current defaults: ~81 candidates per leg, reasonable for real-time control

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

The new cost functions support **Gait Independent Adaptation (GIA)** principles, which aim to select footholds that maintain stable, periodic gaits independent of specific gait timing:

### What is GIA?

GIA is a locomotion principle where foothold selection is based primarily on:
1. **Terrain characteristics** (safety, stability)
2. **Kinematic feasibility** (reachability)
3. **Motion direction** (velocity alignment)
4. **Gait symmetry** (consistent step patterns)

Rather than being tied to specific gait phases or timing.

### Why GIA Matters

- **Robustness**: Gaits remain stable even with timing variations or disturbances
- **Adaptability**: Robot can adjust gait parameters (frequency, duty cycle) without replanning footholds
- **Efficiency**: Symmetric, consistent steps reduce energy consumption
- **Predictability**: Periodic patterns make trajectory planning more reliable

### GIA in TAMOLS

The TAMOLS implementation supports GIA through:

1. **Velocity Alignment Cost** (`weight_velocity_alignment`):
   - Encourages footholds that align with the robot's current velocity direction
   - Ensures foot placements naturally follow the direction of motion
   - Decouples foothold selection from gait phase

2. **Step Consistency Cost** (`weight_step_consistency`):
   - Maintains similar step lengths across all legs
   - Promotes symmetric gait patterns
   - Reduces left-right and front-rear asymmetries

3. **Forward Progress Cost** (`weight_forward_progress`):
   - Prevents the robot from stalling or taking excessively small steps
   - Maintains forward momentum independent of gait type
   - Complements GIA by ensuring continuous progress

### Tuning for GIA

To emphasize GIA compliance:
- Set `weight_velocity_alignment` relatively high (3.0-4.0)
- Set `weight_step_consistency` to moderate-high (2.0-3.0)
- Balance with terrain safety weights (`edge_avoidance`, `roughness`)
- Monitor gait symmetry and adjust if asymmetries develop

**Example GIA-optimized configuration:**
```python
'tamols_params': {
    'weight_edge_avoidance': 5.0,
    'weight_roughness': 2.0,
    'weight_deviation': 1.0,
    'weight_kinematic': 10.0,
    'weight_forward_progress': 3.0,
    'weight_velocity_alignment': 4.0,  # Emphasis on velocity alignment
    'weight_step_consistency': 3.0,     # Emphasis on gait symmetry
}
```

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
