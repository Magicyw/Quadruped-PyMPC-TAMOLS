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

The total cost for each candidate combines four TAMOLS-inspired metrics:

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
        'weight_edge_avoidance': 5.0,  # penalize high gradients
        'weight_roughness': 2.0,       # penalize rough terrain
        'weight_deviation': 1.0,       # penalize deviation from seed
        'weight_kinematic': 10.0,      # penalize unreachable positions
        
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

## Backward Compatibility

The implementation is fully backward compatible with existing strategies:
- `'blind'`: No adaptation, uses seed footholds directly
- `'height'`: Simple height adjustment from heightmap
- `'vfa'`: Uses VFA neural network (if installed)
- `'tamols'`: New TAMOLS-inspired strategy

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
