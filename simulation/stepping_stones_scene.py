"""
Custom scene generator for stepping stones (plum blossom poles) terrain.

Creates a terrain with:
1. Uphill slope
2. Short flat section
3. Stepping stones (plum blossom poles)
4. Short flat section
5. Downhill slope

Usage:
    from simulation.stepping_stones_scene import create_stepping_stones_scene
    scene_xml = create_stepping_stones_scene()
"""

import numpy as np


def create_stepping_stones_scene(
    uphill_length=3.0,
    uphill_angle=15.0,  # degrees
    flat1_length=1.0,
    stepping_stones_length=4.0,
    stone_radius=0.15,
    stone_height=0.05,
    stone_spacing=0.4,
    stones_per_row=3,
    flat2_length=1.0,
    downhill_length=3.0,
    downhill_angle=15.0,  # degrees
    ground_friction=1.0,
):
    """Generate MuJoCo XML for stepping stones terrain.

    Args:
        uphill_length: Length of uphill slope (m)
        uphill_angle: Angle of uphill slope (degrees)
        flat1_length: Length of first flat section (m)
        stepping_stones_length: Length of stepping stones section (m)
        stone_radius: Radius of each stepping stone (m)
        stone_height: Height of stepping stones above ground (m)
        stone_spacing: Spacing between stepping stone centers (m)
        stones_per_row: Number of stones in each row (transverse direction)
        flat2_length: Length of second flat section (m)
        downhill_length: Length of downhill slope (m)
        downhill_angle: Angle of downhill slope (degrees)
        ground_friction: Friction coefficient for ground

    Returns:
        str: MuJoCo XML string for the scene
    """
    # Convert angles to radians
    uphill_rad = np.deg2rad(uphill_angle)
    downhill_rad = np.deg2rad(downhill_angle)

    # Calculate heights
    uphill_height = uphill_length * np.tan(uphill_rad)
    downhill_height = downhill_length * np.tan(downhill_rad)

    # Calculate positions along x-axis
    x_start = 0.0
    x_uphill_end = x_start + uphill_length * np.cos(uphill_rad)
    x_flat1_end = x_uphill_end + flat1_length
    x_stones_end = x_flat1_end + stepping_stones_length
    x_flat2_end = x_stones_end + flat2_length
    x_downhill_end = x_flat2_end + downhill_length * np.cos(downhill_rad)

    # Heights at different sections
    z_ground = 0.0
    z_uphill_top = z_ground + uphill_height
    z_stones = z_uphill_top + stone_height

    # Start building XML
    xml_parts = []
    xml_parts.append('<mujoco model="stepping_stones_terrain">')
    xml_parts.append("  <asset>")
    xml_parts.append('    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>')
    xml_parts.append('    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>')
    xml_parts.append('    <material name="stone" rgba=".6 .6 .7 1"/>')
    xml_parts.append("  </asset>")
    xml_parts.append("")
    xml_parts.append("  <worldbody>")
    xml_parts.append('    <light pos="0 0 3" dir="0 0 -1" directional="true"/>')
    xml_parts.append("")

    # 1. Uphill slope
    uphill_x_center = x_start + uphill_length * np.cos(uphill_rad) / 2
    uphill_z_center = z_ground + uphill_length * np.sin(uphill_rad) / 2
    xml_parts.append('    <!-- Uphill slope -->')
    xml_parts.append(
        f'    <geom name="uphill" type="box" '
        f'pos="{uphill_x_center} 0 {uphill_z_center}" '
        f'size="{uphill_length/2} 2.0 0.1" '
        f'euler="0 {uphill_angle} 0" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")

    # 2. First flat section
    flat1_x_center = x_uphill_end + flat1_length / 2
    flat1_z_center = z_uphill_top
    xml_parts.append('    <!-- First flat section -->')
    xml_parts.append(
        f'    <geom name="flat1" type="box" '
        f'pos="{flat1_x_center} 0 {flat1_z_center}" '
        f'size="{flat1_length/2} 2.0 0.1" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")

    # 3. Stepping stones (plum blossom poles pattern)
    xml_parts.append('    <!-- Stepping stones section -->')
    stone_count = 0
    x_current = x_flat1_end + stone_spacing
    row_offset = 0  # Alternate row offset for plum blossom pattern

    while x_current < x_stones_end:
        # Create stones in current row
        if stones_per_row == 1:
            # Single stone in center
            y_positions = [0.0]
        elif stones_per_row == 2:
            # Two stones
            y_positions = [-stone_spacing / 2, stone_spacing / 2]
        else:
            # Multiple stones - create plum blossom pattern
            y_positions = []
            for i in range(stones_per_row):
                y_offset = (i - (stones_per_row - 1) / 2) * stone_spacing
                # Add alternating row offset
                y_offset += row_offset
                y_positions.append(y_offset)

        for y_pos in y_positions:
            xml_parts.append(
                f'    <geom name="stone_{stone_count}" type="cylinder" '
                f'pos="{x_current} {y_pos} {z_stones}" '
                f'size="{stone_radius} {stone_height/2}" '
                f'material="stone" friction="{ground_friction} 0.005 0.0001"/>'
            )
            stone_count += 1

        # Move to next row
        x_current += stone_spacing
        # Alternate offset for plum blossom pattern
        row_offset = stone_spacing / 4 if row_offset == 0 else 0

    xml_parts.append("")

    # 4. Second flat section
    flat2_x_center = x_stones_end + flat2_length / 2
    flat2_z_center = z_uphill_top
    xml_parts.append('    <!-- Second flat section -->')
    xml_parts.append(
        f'    <geom name="flat2" type="box" '
        f'pos="{flat2_x_center} 0 {flat2_z_center}" '
        f'size="{flat2_length/2} 2.0 0.1" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")

    # 5. Downhill slope
    downhill_x_center = x_flat2_end + downhill_length * np.cos(downhill_rad) / 2
    downhill_z_center = z_uphill_top - downhill_length * np.sin(downhill_rad) / 2
    xml_parts.append('    <!-- Downhill slope -->')
    xml_parts.append(
        f'    <geom name="downhill" type="box" '
        f'pos="{downhill_x_center} 0 {downhill_z_center}" '
        f'size="{downhill_length/2} 2.0 0.1" '
        f'euler="0 -{downhill_angle} 0" '
        f'material="grid" friction="{ground_friction} 0.005 0.0001"/>'
    )
    xml_parts.append("")

    # Add a large ground plane underneath everything for safety
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


def save_stepping_stones_scene(filename="stepping_stones_terrain.xml", **kwargs):
    """Save stepping stones scene to XML file.

    Args:
        filename: Output XML filename
        **kwargs: Parameters passed to create_stepping_stones_scene()
    """
    xml_content = create_stepping_stones_scene(**kwargs)
    with open(filename, "w") as f:
        f.write(xml_content)
    print(f"Stepping stones scene saved to: {filename}")
    return filename


if __name__ == "__main__":
    # Generate and save the default stepping stones scene
    print("Generating stepping stones terrain...")
    xml = create_stepping_stones_scene(
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
    save_stepping_stones_scene("stepping_stones_terrain.xml")

    # Print preview
    print("\nGenerated XML preview:")
    print("=" * 60)
    print(xml)
    print("=" * 60)
