# Stepping Stones Terrain (梅花桩地图)

## Overview

This is a custom terrain designed to test the TAMOLS foothold planner on challenging stepping stone terrain, inspired by traditional Chinese martial arts training equipment (梅花桩 - plum blossom poles).

## Terrain Structure

The terrain consists of five sequential sections:

```
[Uphill] → [Flat 1] → [Stepping Stones] → [Flat 2] → [Downhill]
   ↗          —           • • •            —             ↘
```

### 1. Uphill Slope (上坡)
- **Length**: 3.0 m (default)
- **Angle**: 15° (default)
- **Purpose**: Tests uphill locomotion and prepares for stepping stones

### 2. First Flat Section (平地)
- **Length**: 1.0 m (default)
- **Purpose**: Transition zone before stepping stones

### 3. Stepping Stones Section (梅花桩)
- **Length**: 4.0 m (default)
- **Stone radius**: 0.15 m (default)
- **Stone height**: 0.05 m above previous section (default)
- **Stone spacing**: 0.4 m between centers (default)
- **Stones per row**: 3 (default)
- **Pattern**: Alternating offset rows (plum blossom pattern)
- **Purpose**: Main challenge - requires precise foothold placement

### 4. Second Flat Section (平地)
- **Length**: 1.0 m (default)
- **Purpose**: Recovery zone after stepping stones

### 5. Downhill Slope (下坡)
- **Length**: 3.0 m (default)
- **Angle**: 15° (default)
- **Purpose**: Tests downhill locomotion

## Usage

### Quick Start

1. **Generate the terrain XML file**:
```bash
cd simulation
python stepping_stones_scene.py
```

This creates `stepping_stones_terrain.xml` in the simulation directory.

2. **Use with gym_quadruped** (if using custom scene loading):
```python
from simulation.stepping_stones_scene import create_stepping_stones_scene

# Generate custom terrain
xml_content = create_stepping_stones_scene(
    uphill_length=3.0,
    uphill_angle=15.0,
    flat1_length=1.0,
    stepping_stones_length=4.0,
    stone_radius=0.15,
    stone_height=0.05,
    stone_spacing=0.4,
    stones_per_row=3,
    flat2_length=1.0,
    downhill_length=3.0,
    downhill_angle=15.0,
)

# Save to file
with open("my_terrain.xml", "w") as f:
    f.write(xml_content)
```

3. **Test with TAMOLS planner**:
```python
# In config.py, enable TAMOLS
cfg.simulation_params['visual_foothold_adaptation'] = 'tamols'

# Tune parameters for stepping stones
cfg.simulation_params['tamols_params']['search_radius'] = 0.20  # Larger search for stones
cfg.simulation_params['tamols_params']['weight_edge_avoidance'] = 8.0  # Higher edge penalty
```

### Customization

You can customize the terrain by modifying parameters:

```python
from simulation.stepping_stones_scene import save_stepping_stones_scene

# Create easier terrain (larger stones, closer spacing)
save_stepping_stones_scene(
    filename="easy_stones.xml",
    stone_radius=0.20,      # Larger stones
    stone_spacing=0.35,     # Closer spacing
    stone_height=0.03,      # Lower height
    uphill_angle=10.0,      # Gentler slopes
    downhill_angle=10.0,
)

# Create harder terrain (smaller stones, wider spacing)
save_stepping_stones_scene(
    filename="hard_stones.xml",
    stone_radius=0.12,      # Smaller stones
    stone_spacing=0.50,     # Wider spacing
    stone_height=0.08,      # Higher height
    uphill_angle=20.0,      # Steeper slopes
    downhill_angle=20.0,
)
```

## Design Rationale

### Stepping Stone Pattern

The stepping stones use an alternating offset pattern inspired by plum blossom poles:

```
Row 1:    •     •     •
Row 2:      •     •     •
Row 3:    •     •     •
Row 4:      •     •     •
```

This pattern:
- Mimics traditional 梅花桩 (plum blossom pole) training
- Forces dynamic foothold selection
- Tests lateral and longitudinal stepping
- Challenges balance and coordination

### Default Parameters

The default parameters are tuned for:
- **Robot**: Go1/Go2 quadruped (leg reach ~0.45m)
- **Difficulty**: Medium (suitable for testing TAMOLS planner)
- **Safety**: Stone radius large enough for stable footholds
- **Challenge**: Spacing requires active foothold planning

## Testing TAMOLS on This Terrain

This terrain is ideal for evaluating TAMOLS foothold adaptation:

### Expected Behaviors

1. **Uphill Section**: 
   - TAMOLS should maintain steady gait
   - Edge avoidance should detect slope transition

2. **Stepping Stones Section**:
   - TAMOLS should select stone centers as footholds
   - Edge avoidance should reject gaps between stones
   - Kinematic constraints should prevent overreaching
   - Roughness penalty should favor stone tops over edges

3. **Downhill Section**:
   - TAMOLS should adapt to negative slope
   - Deviation penalty should maintain reasonable trajectory

### Recommended TAMOLS Parameters

For optimal performance on stepping stones:

```python
'tamols_params': {
    'search_radius': 0.20,           # Increase to reach stones
    'search_resolution': 0.025,      # Finer search for stone centers
    'weight_edge_avoidance': 8.0,    # High penalty for gaps
    'weight_roughness': 3.0,         # Favor flat stone tops
    'weight_deviation': 0.5,         # Allow deviation to reach stones
    'weight_kinematic': 10.0,        # Enforce reach limits
}
```

### Expected Improvements vs Baseline

Compared to baseline strategies:

| Strategy | Expected Performance |
|----------|---------------------|
| `'blind'` | Likely to fall into gaps or miss stones |
| `'height'` | May step on stones but won't avoid edges |
| `'tamols'` | Should reliably find and step on stone centers |

## Visualization

When running with MuJoCo viewer:
- **Gray grid texture**: Slopes and flat sections
- **Gray cylinders**: Stepping stones (slightly elevated)
- **Robot footholds**: Should land on stone centers with TAMOLS

## Troubleshooting

### Robot falls through gaps
- Increase `stone_radius` or decrease `stone_spacing`
- Increase TAMOLS `search_radius`
- Increase `weight_edge_avoidance`

### Robot cannot reach stones
- Decrease `stone_spacing`
- Check `l_max` kinematic constraint
- Increase `search_radius`

### Robot steps on stone edges
- Increase `weight_edge_avoidance`
- Decrease `search_resolution` for finer search
- Increase `weight_roughness`

## Technical Details

### MuJoCo Implementation
- Terrain is built from primitive geoms (boxes and cylinders)
- All geoms have configurable friction
- Ground plane at z=-1.0 for safety
- Checker texture for visual feedback

### Collision Properties
- Friction coefficient: 1.0 (default)
- Rolling friction: 0.005
- Torsional friction: 0.0001

### Coordinate System
- X-axis: Forward direction (uphill → downhill)
- Y-axis: Lateral direction (left-right)
- Z-axis: Vertical (up)
- Origin: Start of uphill section

## Future Enhancements

Potential improvements:
1. **Random stone patterns**: Procedurally generate different layouts
2. **Variable stone heights**: Mix different elevation levels
3. **Unstable stones**: Stones that tilt when stepped on
4. **Moving stones**: Dynamic difficulty
5. **Different stone shapes**: Hexagonal, square, irregular
6. **Narrow beams**: Connect stones with thin beams

## References

- Traditional 梅花桩 (plum blossom poles): Ancient Chinese martial arts training equipment
- TAMOLS foothold planner: [docs/TAMOLS_FOOTHOLD_ADAPTATION.md](TAMOLS_FOOTHOLD_ADAPTATION.md)
