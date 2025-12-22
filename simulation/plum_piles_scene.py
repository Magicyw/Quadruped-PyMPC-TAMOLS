"""Generate MuJoCo XML scene for plum piles (dense stepping stones) terrain.

This module creates a custom terrain with:
1. Ramps and platforms at ~0.536m height
2. Dense regular grid of cylindrical plum piles (radius=0.08m, height=0.268m)
3. Grid: x from 4.1 to 7.1 step 0.2, y from -0.4 to 0.4 step 0.2

Usage:
    from simulation.plum_piles_scene import create_plum_piles_scene, save_plum_piles_scene
    xml = create_plum_piles_scene()
    save_plum_piles_scene('plum_piles_terrain.xml')
"""

import numpy as np


def create_plum_piles_scene(
    flat_start_length=3.0,
    ramp_length=1.5,
    ramp_angle=15.0,
    platform_height=0.536,
    platform_length=1.0,
    plum_piles_x_start=4.1,
    plum_piles_x_end=7.1,
    plum_piles_y_min=-0.4,
    plum_piles_y_max=0.4,
    pile_x_step=0.2,
    pile_y_step=0.2,
    pile_radius=0.08,
    pile_height=0.268,
    ground_friction=1.0,
):
    """Generate MuJoCo XML for plum piles terrain.
    
    Creates a challenging terrain with ramps leading to a platform, then a dense
    grid of cylindrical "plum piles" (stepping stones) that the robot must navigate.
    
    Args:
        flat_start_length: Length of flat starting area where robot spawns (m)
        ramp_length: Length of ramp to platform (m)
        ramp_angle: Angle of ramp (degrees)
        platform_height: Height of platform section (m), approximately 0.536
        platform_length: Length of platform before piles (m)
        plum_piles_x_start: Start of pile grid in x (m), default 4.1
        plum_piles_x_end: End of pile grid in x (m), default 7.1
        plum_piles_y_min: Minimum y for pile grid (m), default -0.4
        plum_piles_y_max: Maximum y for pile grid (m), default 0.4
        pile_x_step: Spacing between pile centers along x (m), default 0.2
        pile_y_step: Spacing between pile centers along y (m), default 0.2
        pile_radius: Pile radius (m), default 0.08
        pile_height: Height of pile top above ground (m), default 0.268
        ground_friction: Friction coefficient
        
    Returns:
        str: MuJoCo XML string for the scene
    """
    ramp_rad = np.deg2rad(ramp_angle)
    
    # Calculate positions along x-axis
    x_start = 0.0
    x_flat_end = x_start + flat_start_length
    x_ramp_end = x_flat_end + ramp_length * np.cos(ramp_rad)
    x_platform_end = x_ramp_end + platform_length
    
    # Start building XML
    xml_parts = []
    xml_parts.append('<mujoco model="plum_piles_terrain">')
    xml_parts.append("  <asset>")
    xml_parts.append('    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>')
    xml_parts.append('    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>')
    xml_parts.append('    <material name="pile" rgba=".7 .5 .3 1"/>')  # Brown color for piles
    xml_parts.append("  </asset>")
    xml_parts.append("")
    xml_parts.append("  <worldbody>")
    xml_parts.append('    <light pos="0 0 3" dir="0 0 -1" directional="true"/>')
    xml_parts.append("")
    
    # 1. Flat starting area (where robot spawns)
    flat_x_center = x_start + flat_start_length / 2
    xml_parts.append('    <!-- Flat starting area (robot spawn point) -->')
    xml_parts.append(
        f'    <geom name="flat_start" type="box" '
        f'pos="{flat_x_center} 0 0" '
        f'size="{flat_start_length/2} 2.0 0.1" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")
    
    # 2. Ramp to platform
    ramp_x_center = x_flat_end + ramp_length * np.cos(ramp_rad) / 2
    ramp_z_center = ramp_length * np.sin(ramp_rad) / 2
    xml_parts.append('    <!-- Ramp to platform -->')
    xml_parts.append(
        f'    <geom name="ramp_up" type="box" '
        f'pos="{ramp_x_center} 0 {ramp_z_center}" '
        f'size="{ramp_length/2} 2.0 0.1" '
        f'euler="0 {ramp_angle} 0" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")
    
    # 3. Platform section before piles
    platform_x_center = x_ramp_end + platform_length / 2
    xml_parts.append('    <!-- Platform section -->')
    xml_parts.append(
        f'    <geom name="platform" type="box" '
        f'pos="{platform_x_center} 0 {platform_height}" '
        f'size="{platform_length/2} 2.0 0.1" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")
    
    # 4. Dense grid of plum piles (stepping stones)
    xml_parts.append('    <!-- Plum piles (dense stepping stones grid) -->')
    pile_count = 0
    x_positions = np.arange(plum_piles_x_start, plum_piles_x_end + pile_x_step/2, pile_x_step)
    y_positions = np.arange(plum_piles_y_min, plum_piles_y_max + pile_y_step/2, pile_y_step)
    
    for x_pos in x_positions:
        for y_pos in y_positions:
            # Place cylinder with its top at pile_height
            # MuJoCo cylinder size is (radius, half-height)
            pile_z_center = pile_height / 2
            xml_parts.append(
                f'    <geom name="pile_{pile_count}" type="cylinder" '
                f'pos="{x_pos} {y_pos} {pile_z_center}" '
                f'size="{pile_radius} {pile_height/2}" '
                f'material="pile" friction="{ground_friction} 0.005 0.0001"/>'
            )
            pile_count += 1
    
    xml_parts.append(f"    <!-- Total plum piles: {pile_count} -->")
    xml_parts.append("")
    
    # 5. Add a large ground plane underneath for safety
    xml_parts.append('    <!-- Safety ground plane -->')
    xml_parts.append(
        f'    <geom name="ground" type="plane" '
        f'pos="0 0 -1.0" size="50 50 0.1" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")
    
    xml_parts.append("  </worldbody>")
    xml_parts.append("</mujoco>")
    
    return "\n".join(xml_parts)


def save_plum_piles_scene(filename="plum_piles_terrain.xml", **kwargs):
    """Save plum piles scene to XML file.
    
    Args:
        filename: Output XML filename
        **kwargs: Parameters passed to create_plum_piles_scene()
    """
    xml_content = create_plum_piles_scene(**kwargs)
    with open(filename, "w") as f:
        f.write(xml_content)
    print(f"Plum piles scene saved to: {filename}")
    
    # Count piles for user info
    x_start = kwargs.get('plum_piles_x_start', 4.1)
    x_end = kwargs.get('plum_piles_x_end', 7.1)
    y_min = kwargs.get('plum_piles_y_min', -0.4)
    y_max = kwargs.get('plum_piles_y_max', 0.4)
    x_step = kwargs.get('pile_x_step', 0.2)
    y_step = kwargs.get('pile_y_step', 0.2)
    
    num_x = int((x_end - x_start) / x_step) + 1
    num_y = int((y_max - y_min) / y_step) + 1
    total_piles = num_x * num_y
    
    print(f"  Grid: {num_x} x {num_y} = {total_piles} piles")
    print(f"  X range: [{x_start}, {x_end}] step {x_step}")
    print(f"  Y range: [{y_min}, {y_max}] step {y_step}")
    print(f"  Pile radius: {kwargs.get('pile_radius', 0.08)}m, height: {kwargs.get('pile_height', 0.268)}m")
    
    return filename


if __name__ == "__main__":
    print("Generating plum piles terrain...")
    print("=" * 70)
    
    # Generate with default parameters from problem statement
    save_plum_piles_scene(
        "plum_piles_terrain.xml",
        plum_piles_x_start=4.1,
        plum_piles_x_end=7.1,
        plum_piles_y_min=-0.4,
        plum_piles_y_max=0.4,
        pile_x_step=0.2,
        pile_y_step=0.2,
        pile_radius=0.08,
        pile_height=0.268,
    )
    
    print("\n" + "=" * 70)
    print("✓ Plum piles terrain generated successfully!")
    print("\nTo use this terrain:")
    print("1. Copy the XML to gym_quadruped's scene directory, or")
    print("2. Set scene path in config.py: simulation_params['scene'] = 'path/to/plum_piles_terrain.xml'")
    print("3. Enable high-level planner: simulation_params['high_level_planner']['enabled'] = True")
    print("4. Enable plum piles mode: simulation_params['high_level_planner']['plum_piles']['enabled'] = True")
    print("5. Set gait to crawl: simulation_params['gait'] = 'crawl'")
