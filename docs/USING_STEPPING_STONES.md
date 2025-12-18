# Using Stepping Stones Terrain with Your Robot (机器人使用梅花桩地图指南)

## Quick Start (快速开始)

### 1. Generate Terrain Files (生成地图文件)

First, generate the stepping stones terrain XML files:

```bash
cd simulation
python example_stepping_stones.py
```

This creates 5 terrain files:
- `stepping_stones_easy.xml` - 初级难度
- `stepping_stones_medium.xml` - 中级难度  
- `stepping_stones_hard.xml` - 高级难度
- `stepping_stones_sparse.xml` - 稀疏布局
- `stepping_stones_terrain.xml` - 默认地图

### 2. Configure Your Simulation (配置仿真)

Edit `quadruped_pympc/config.py`:

```python
simulation_params = {
    # ... other parameters ...
    
    # Select stepping stones terrain (选择梅花桩地图)
    'scene': 'stepping_stones_medium',  # or 'stepping_stones_easy', 'stepping_stones_hard', etc.
    
    # Enable TAMOLS for terrain-aware foothold planning (启用TAMOLS地形感知)
    'visual_foothold_adaptation': 'tamols',
    
    # Adjust TAMOLS parameters for stepping stones (调整TAMOLS参数)
    'tamols_params': {
        'search_radius': 0.20,              # Larger search for stones
        'search_resolution': 0.025,         # Fine-grained search
        'weight_edge_avoidance': 8.0,       # High penalty for gaps
        'weight_roughness': 3.0,            # Favor flat stone tops
        'weight_deviation': 0.5,            # Allow deviation to reach stones
        'weight_kinematic': 10.0,           # Enforce leg reach limits
    },
}
```

### 3. Run Simulation (运行仿真)

```bash
python simulation/simulation.py
```

## Available Scenes (可用的地图)

### Built-in Scenes (内置地图)
- `'flat'` - Flat terrain (平地)
- `'random_boxes'` - Random obstacles (随机障碍物)
- `'random_pyramids'` - Pyramid obstacles (金字塔障碍物)
- `'perlin'` - Perlin noise terrain (柏林噪声地形)

### Stepping Stones Scenes (梅花桩地图)
- `'stepping_stones'` or `'stepping_stones_medium'` - **Recommended starting point** (推荐起点)
  - Stone radius: 0.15m
  - Stone spacing: 0.40m
  - Slope angles: 15°
  
- `'stepping_stones_easy'` - Easy difficulty (初级难度)
  - Stone radius: 0.20m (larger stones / 更大的石头)
  - Stone spacing: 0.35m (closer / 更近)
  - Slope angles: 10°
  
- `'stepping_stones_hard'` - Hard difficulty (高级难度)
  - Stone radius: 0.12m (smaller stones / 更小的石头)
  - Stone spacing: 0.50m (wider / 更宽)
  - Slope angles: 20°
  
- `'stepping_stones_sparse'` - Sparse layout (稀疏布局)
  - Only 2 stones per row (每排只有2个石头)
  - Stone spacing: 0.55m

## TAMOLS Parameters for Each Terrain (不同地形的TAMOLS参数)

### Easy Terrain (初级难度)
```python
'tamols_params': {
    'search_radius': 0.18,
    'search_resolution': 0.03,
    'weight_edge_avoidance': 5.0,
    'weight_roughness': 2.0,
    'weight_deviation': 1.0,
    'weight_kinematic': 10.0,
}
```

### Medium Terrain (中级难度) - **Recommended** (推荐)
```python
'tamols_params': {
    'search_radius': 0.20,
    'search_resolution': 0.025,
    'weight_edge_avoidance': 8.0,
    'weight_roughness': 3.0,
    'weight_deviation': 0.5,
    'weight_kinematic': 10.0,
}
```

### Hard Terrain (高级难度)
```python
'tamols_params': {
    'search_radius': 0.25,
    'search_resolution': 0.02,
    'weight_edge_avoidance': 10.0,
    'weight_roughness': 4.0,
    'weight_deviation': 0.3,
    'weight_kinematic': 12.0,
}
```

### Sparse Terrain (稀疏布局)
```python
'tamols_params': {
    'search_radius': 0.30,
    'search_resolution': 0.025,
    'weight_edge_avoidance': 12.0,
    'weight_roughness': 5.0,
    'weight_deviation': 0.2,
    'weight_kinematic': 15.0,
}
```

## Custom Terrain Parameters (自定义地形参数)

You can also generate custom terrains on-the-fly by providing parameters in config.py:

```python
simulation_params = {
    'scene': 'stepping_stones',  # Will use custom params below
    
    # Custom terrain parameters (自定义地形参数)
    'stepping_stones_params': {
        'uphill_length': 2.5,        # Length of uphill (上坡长度)
        'uphill_angle': 12.0,        # Uphill angle in degrees (上坡角度)
        'flat1_length': 0.8,         # First flat section (第一段平地)
        'stepping_stones_length': 5.0,  # Stones section length (梅花桩区域长度)
        'stone_radius': 0.18,        # Stone radius (石头半径)
        'stone_height': 0.04,        # Stone height above ground (石头高度)
        'stone_spacing': 0.38,       # Spacing between stones (石头间距)
        'stones_per_row': 3,         # Stones per row (每排石头数)
        'flat2_length': 0.8,         # Second flat section (第二段平地)
        'downhill_length': 2.5,      # Length of downhill (下坡长度)
        'downhill_angle': 12.0,      # Downhill angle (下坡角度)
    },
}
```

## Complete Example Configuration (完整配置示例)

Here's a complete example to get your robot moving on stepping stones terrain:

```python
# In quadruped_pympc/config.py

simulation_params = {
    # Basic simulation settings
    'swing_generator': 'scipy',
    'swing_position_gain_fb': 500,
    'swing_velocity_gain_fb': 10,
    'step_height': 0.2 * hip_height,
    
    # Stepping stones terrain (梅花桩地图)
    'scene': 'stepping_stones_medium',
    'stepping_stones_params': None,  # Use pre-generated file
    
    # Enable TAMOLS foothold adaptation (启用TAMOLS)
    'visual_foothold_adaptation': 'tamols',
    
    # TAMOLS parameters optimized for stepping stones (优化的TAMOLS参数)
    'tamols_params': {
        'search_radius': 0.20,
        'search_resolution': 0.025,
        'weight_edge_avoidance': 8.0,
        'weight_roughness': 3.0,
        'weight_deviation': 0.5,
        'weight_kinematic': 10.0,
        'l_min': {'go1': 0.15, 'go2': 0.15, 'aliengo': 0.18},
        'l_max': {'go1': 0.45, 'go2': 0.45, 'aliengo': 0.55},
        'constraint_box_dx': 0.05,
        'constraint_box_dy': 0.05,
    },
    
    # Other simulation settings
    'dt': 0.002,
    'gait': 'trot',
    'mode': 'human',  # Use keyboard to control
    'ref_z': hip_height,
    'mpc_frequency': 100,
}
```

## Troubleshooting (故障排除)

### Problem: Robot falls through gaps (机器人掉进缝隙)
**Solution:**
1. Start with `'stepping_stones_easy'` terrain
2. Increase TAMOLS `search_radius` to 0.25m
3. Increase `weight_edge_avoidance` to 10.0

### Problem: Robot cannot reach stones (机器人够不到石头)
**Solution:**
1. Check `l_max` kinematic constraint matches your robot
2. Increase `search_radius`
3. Use `'stepping_stones_easy'` with closer spacing

### Problem: Robot steps on stone edges (机器人踩在石头边缘)
**Solution:**
1. Decrease `search_resolution` to 0.02m for finer search
2. Increase `weight_edge_avoidance` to 10.0
3. Increase `weight_roughness` to 4.0

### Problem: Terrain files not found (找不到地形文件)
**Solution:**
```bash
cd simulation
python example_stepping_stones.py
```
This will regenerate all terrain files.

### Problem: gym_quadruped doesn't load custom scene (无法加载自定义地图)
**Note:** If `gym_quadruped` doesn't support custom XML paths directly, you may need to:
1. Copy the terrain XML to gym_quadruped's scene directory
2. Or modify gym_quadruped to accept custom paths
3. Or use the built-in scenes and manually place obstacles

## Expected Behavior (预期行为)

When the robot successfully uses stepping stones terrain with TAMOLS:

1. **Uphill Section (上坡)**: Robot maintains steady trot gait, adapts to slope
2. **Stepping Stones (梅花桩)**: 
   - Robot selects stone centers as footholds
   - Avoids gaps between stones
   - Maintains balance through precise foothold placement
3. **Downhill Section (下坡)**: Robot adapts to negative slope, maintains stability

## Performance Tips (性能提示)

- **Start slow**: Begin with easy terrain and human control mode
- **Monitor heightmaps**: Check that heightmaps are updating correctly at swing apex
- **Tune gradually**: Adjust one TAMOLS parameter at a time
- **Check kinematic limits**: Ensure `l_min`/`l_max` match your robot's actual reach

## Further Reading (延伸阅读)

- [TAMOLS Foothold Adaptation](TAMOLS_FOOTHOLD_ADAPTATION.md) - Detailed TAMOLS documentation
- [Stepping Stones Terrain](STEPPING_STONES_TERRAIN.md) - Terrain design and customization
- `simulation/example_stepping_stones.py` - Example script with all terrain variants

## Video Tutorial (视频教程)

Coming soon: Step-by-step video showing robot navigation through stepping stones terrain.

---

**Questions?** Check the documentation files or examine `simulation/simulation.py` for implementation details.
