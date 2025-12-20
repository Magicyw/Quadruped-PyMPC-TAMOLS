# Paper-Based Foothold Optimization (TAMOLS Strategy)

This document explains the implementation of online foothold optimization from the paper "Perceptive Locomotion in Rough Terrain – Online Foothold Optimization".

## Overview

The `tamols` adaptation strategy implements the paper's foothold optimization algorithm, which:
- Batch-searches over grid cells around predicted footholds
- Evaluates candidates using 6 weighted objectives (descending importance)
- Enforces 3 hard constraints
- Selects the locally optimal foothold for each leg independently

## Enabling the Strategy

In `quadruped_pympc/config.py`, set:

```python
simulation_params = {
    'visual_foothold_adaptation': 'tamols',  # Options: 'blind', 'height', 'vfa', 'tamols'
    ...
}
```

## Paper Objectives (Equations 2-9)

The objectives are weighted in descending order of importance:

### 1. Default Leg Configuration (Eq. 2-4)
**Weight:** `weight_default_config` (default: 1.0)

Penalizes deviation of candidate foothold from a default foot location derived from:
- Thigh position (hip position)
- Nominal leg extension direction
- Blending of world z-axis and local terrain plane normal
- *Note: Velocity feedback term (Eq. 3-4) requires thigh velocity, which is not currently available in the interface. This is a future enhancement.*

### 2. Foothold Score (Eq. 5)
**Weight:** `weight_foothold_score` (default: 0.8)

Terrain quality score combining:
- **Edges:** High terrain gradients (detected via finite differences)
- **Slopes:** Local terrain inclination
- **Roughness:** Height variance in local patch

All terms are normalized into [0,1] where 0=best terrain, 1=worst.

### 3. Push-Over Regularizer (Eq. 6)
**Weight:** `weight_pushover` (default: 0.6)

For **swinging legs only**, minimizes the maximum foothold-score along the line segment between:
- Previous optimal foothold
- Current candidate

This penalizes trajectories that pass over high obstacles. For stance legs, this cost is zero.

### 4. Support Area (Eq. 7)
**Weight:** `weight_support_area` (default: 0.4)

Encourages larger support polygon by maximizing distances to neighboring legs' measured end-effector locations. Larger support area = more stable stance = lower cost.

### 5. Previous Foothold Continuity (Eq. 8)
**Weight:** `weight_continuity` (default: 0.2)

Penalizes deviation from the previous optimal foothold, promoting smooth foothold sequences.

### 6. Leg Over-Extension (Eq. 9)
**Weight:** `weight_leg_extension` (default: 0.1)

Regularizes to reduce required leg extension. Penalizes candidates that require the leg to extend far from its nominal length.

## Paper Constraints (Equations 10-11)

Hard constraints that filter out infeasible candidates:

### 1. Max Step Height (Eq. 10)
**Threshold:** `h_max` (default: 0.15 m)

Computes obstacle height along the line between:
- Current/previous stance foothold → candidate
- Seed foothold → candidate

Rejects candidates if any obstacle along the line exceeds `h_max`.

### 2. Leg Collision (Eq. 11)
**Threshold:** `d_min` (default: 0.10 m)

Rejects candidates that are too close to neighboring legs' current end-effector locations (distance < `d_min`).

### 3. Leg Over-Extension
The paper notes hard constraints are conservative. We implement this as a **soft objective** only (Objective 6), allowing the controller to handle infeasible cases gracefully.

## Configuration Parameters

All parameters are in `simulation_params['tamols_params']`:

```python
'tamols_params': {
    # Candidate generation
    'search_radius': 0.35,           # [m] Search radius around seed
    'search_resolution': 0.04,       # [m] Grid spacing
    'use_local_window': True,        # Use efficient local window
    'gradient_delta': 0.04,          # [m] Finite difference offset
    
    # Objective weights (descending importance)
    'weight_default_config': 1.0,    # Objective 1
    'weight_foothold_score': 0.8,    # Objective 2
    'weight_pushover': 0.6,          # Objective 3
    'weight_support_area': 0.4,      # Objective 4
    'weight_continuity': 0.2,        # Objective 5
    'weight_leg_extension': 0.1,     # Objective 6
    
    # Constraint thresholds
    'h_max': 0.15,                   # [m] Max step height
    'd_min': 0.10,                   # [m] Min leg separation
    
    # Line sampling (for constraints & objectives)
    'pushover_line_samples': 10,     # Samples for Eq. 6
    'obstacle_line_samples': 10,     # Samples for Eq. 10
    
    # Nominal kinematics
    'l_nom': hip_height,             # [m] Nominal leg extension
    'kappa': 0.5,                    # Terrain normal blending factor
    
    # MPC constraints
    'constraint_box_dx': 0.05,       # [m] ±x around foothold
    'constraint_box_dy': 0.05,       # [m] ±y around foothold
}
```

## Integration with Project

The optimizer uses:
- **Current measured foot positions:** From `feet_pos` (LegsAttr) for objectives 4, 5 and constraint 2
- **Hip positions:** Passed as `hip_positions` (corresponds to paper's thigh positions)
- **Contact schedule:** From `self.current_contact` (1=stance, 0=swing) to enable/disable push-over regularizer
- **HeightMap API:** Uses `heightmap.data` and `get_height()` from `gym_quadruped.sensors.heightmap`

### Fallback Behavior
- If `feet_positions=None`: Uses `hip_positions` as fallback
- If `contact_state=None`: Assumes all legs are swinging (all can be optimized)
- If `heightmap.data is None`: Returns `False` (no adaptation possible)

## Tuning Guidance

### Increase Objective Weight When:
1. **Default config:** Robot's footholds drift too far from nominal stance
2. **Foothold score:** Robot doesn't avoid edges/rough terrain enough
3. **Push-over:** Swinging legs collide with obstacles during trajectory
4. **Support area:** Robot becomes unstable (small support polygon)
5. **Continuity:** Foothold jumps too much between steps
6. **Leg extension:** Legs over-extend frequently

### Adjust Constraints When:
- **h_max:** Reduce if robot struggles with steps; increase for more agility
- **d_min:** Increase if legs collide; decrease for more compact stance

### Search Parameters:
- **search_radius:** Larger = more exploration, slower
- **search_resolution:** Finer = more candidates, slower but more optimal
- **use_local_window:** Set `False` to scan full heightmap (debugging only)

## Testing

Run tests with:
```bash
pytest tests/test_foothold_optimization.py -v
```

Note: Most tests are placeholders documenting expected behavior. Full test implementation requires mocking `gym_quadruped` dependencies.

## References

Paper: "Perceptive Locomotion in Rough Terrain – Online Foothold Optimization"
- Equations 2-4: Default leg configuration
- Equation 5: Foothold score (terrain quality)
- Equation 6: Push-over regularizer
- Equation 7: Support area
- Equation 8: Previous foothold continuity
- Equation 9: Leg over-extension objective
- Equation 10: Max step height constraint
- Equation 11: Leg collision constraint

## Implementation Notes

- **Candidate generation:** Circular grid pattern around seed (not square)
- **Terrain quality:** Edge/roughness computed via finite differences over heightmap
- **Line sampling:** Linear interpolation between start/end for constraint checks
- **Objective normalization:** Each objective term is scaled to comparable magnitude
- **Previous optimal tracking:** Stored per-leg in `self.previous_optimal_footholds`
- **Contact-aware:** Push-over regularizer only applies to swinging legs

## Known Limitations

1. **Velocity feedback term (Eq. 3-4):** The paper's velocity feedback term in the default leg configuration objective requires thigh velocity, which is not currently available in the interface. The current implementation uses static thigh position only. Future enhancement would integrate thigh velocity for improved tracking of moving targets during dynamic maneuvers.

2. **Terrain normal blending:** The implementation uses a simplified downward leg extension direction. The paper blends world z-axis with local terrain plane normal using parameter κ (kappa). This could be enhanced to compute and blend terrain normals.

3. **Contact schedule timing:** The implementation uses current contact state. The paper discusses predicting thigh positions at lift-off and touch-down using contact schedule timing. This could be integrated when precise schedule timing is available.

## Future Enhancements

- Integrate thigh velocity for velocity feedback term (Eq. 3-4)
- Implement terrain normal estimation and blending (κ parameter)
- Add contact schedule timing predictions for lift-off/touch-down positions
- Optimize candidate generation with spatial indexing for large heightmaps
- Add visualization of selected footholds and rejected candidates for debugging

## Troubleshooting

**Issue:** No candidates found, falls back to seed
- Check `search_radius` isn't too small
- Verify heightmap covers expected area
- Check if all candidates violate constraints (reduce `h_max` or `d_min`)

**Issue:** Optimizer is slow
- Reduce `search_radius`
- Increase `search_resolution` (coarser grid)
- Ensure `use_local_window=True`

**Issue:** Selected footholds are poor quality
- Increase `weight_foothold_score`
- Verify heightmap resolution is adequate
- Check if constraints are too restrictive
