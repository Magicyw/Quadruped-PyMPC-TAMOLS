#!/usr/bin/env python3
"""
Example script demonstrating how to use the stepping stones terrain.

This script shows:
1. How to generate custom stepping stones terrain
2. How to configure TAMOLS for stepping stones
3. How to visualize the terrain parameters

Usage:
    python example_stepping_stones.py
"""

from stepping_stones_scene import save_stepping_stones_scene


def generate_standard_terrains():
    """Generate a set of standard stepping stones terrains with different difficulty levels."""
    
    print("Generating stepping stones terrains...")
    print("=" * 70)
    
    # 1. Easy terrain (larger stones, closer spacing)
    print("\n1. Easy Terrain (初级难度)")
    print("-" * 70)
    easy_params = {
        'uphill_length': 2.5,
        'uphill_angle': 10.0,
        'flat1_length': 1.0,
        'stepping_stones_length': 3.5,
        'stone_radius': 0.20,  # Larger stones
        'stone_height': 0.03,  # Lower height
        'stone_spacing': 0.35,  # Closer spacing
        'stones_per_row': 3,
        'flat2_length': 1.0,
        'downhill_length': 2.5,
        'downhill_angle': 10.0,
    }
    print(f"  Stone radius: {easy_params['stone_radius']}m")
    print(f"  Stone spacing: {easy_params['stone_spacing']}m")
    print(f"  Slope angles: {easy_params['uphill_angle']}°")
    save_stepping_stones_scene("stepping_stones_easy.xml", **easy_params)
    print("  ✓ Saved to: stepping_stones_easy.xml")
    
    # 2. Medium terrain (default parameters)
    print("\n2. Medium Terrain (中级难度)")
    print("-" * 70)
    medium_params = {
        'uphill_length': 3.0,
        'uphill_angle': 15.0,
        'flat1_length': 1.0,
        'stepping_stones_length': 4.0,
        'stone_radius': 0.15,  # Medium stones
        'stone_height': 0.05,  # Medium height
        'stone_spacing': 0.40,  # Medium spacing
        'stones_per_row': 3,
        'flat2_length': 1.0,
        'downhill_length': 3.0,
        'downhill_angle': 15.0,
    }
    print(f"  Stone radius: {medium_params['stone_radius']}m")
    print(f"  Stone spacing: {medium_params['stone_spacing']}m")
    print(f"  Slope angles: {medium_params['uphill_angle']}°")
    save_stepping_stones_scene("stepping_stones_medium.xml", **medium_params)
    print("  ✓ Saved to: stepping_stones_medium.xml")
    
    # 3. Hard terrain (smaller stones, wider spacing)
    print("\n3. Hard Terrain (高级难度)")
    print("-" * 70)
    hard_params = {
        'uphill_length': 3.5,
        'uphill_angle': 20.0,
        'flat1_length': 0.8,
        'stepping_stones_length': 4.5,
        'stone_radius': 0.12,  # Smaller stones
        'stone_height': 0.08,  # Higher height
        'stone_spacing': 0.50,  # Wider spacing
        'stones_per_row': 3,
        'flat2_length': 0.8,
        'downhill_length': 3.5,
        'downhill_angle': 20.0,
    }
    print(f"  Stone radius: {hard_params['stone_radius']}m")
    print(f"  Stone spacing: {hard_params['stone_spacing']}m")
    print(f"  Slope angles: {hard_params['uphill_angle']}°")
    save_stepping_stones_scene("stepping_stones_hard.xml", **hard_params)
    print("  ✓ Saved to: stepping_stones_hard.xml")
    
    # 4. Sparse terrain (fewer stones, more challenge)
    print("\n4. Sparse Terrain (稀疏布局)")
    print("-" * 70)
    sparse_params = {
        'uphill_length': 3.0,
        'uphill_angle': 15.0,
        'flat1_length': 1.0,
        'stepping_stones_length': 5.0,
        'stone_radius': 0.15,
        'stone_height': 0.05,
        'stone_spacing': 0.55,  # Much wider spacing
        'stones_per_row': 2,  # Fewer stones per row
        'flat2_length': 1.0,
        'downhill_length': 3.0,
        'downhill_angle': 15.0,
    }
    print(f"  Stone radius: {sparse_params['stone_radius']}m")
    print(f"  Stone spacing: {sparse_params['stone_spacing']}m")
    print(f"  Stones per row: {sparse_params['stones_per_row']}")
    save_stepping_stones_scene("stepping_stones_sparse.xml", **sparse_params)
    print("  ✓ Saved to: stepping_stones_sparse.xml")
    
    print("\n" + "=" * 70)
    print("✓ All terrains generated successfully!")
    print("\nTo use a terrain in simulation, you'll need to:")
    print("1. Load the XML file with gym_quadruped or MuJoCo")
    print("2. Configure TAMOLS parameters in config.py")
    print("3. Run simulation.py with visual_foothold_adaptation='tamols'")


def print_tamols_recommendations():
    """Print recommended TAMOLS parameters for each terrain difficulty."""
    
    print("\n" + "=" * 70)
    print("Recommended TAMOLS Parameters for Each Terrain")
    print("=" * 70)
    
    print("\n1. Easy Terrain - Conservative Parameters")
    print("-" * 70)
    print("'tamols_params': {")
    print("    'search_radius': 0.18,")
    print("    'search_resolution': 0.03,")
    print("    'weight_edge_avoidance': 5.0,")
    print("    'weight_roughness': 2.0,")
    print("    'weight_deviation': 1.0,")
    print("    'weight_kinematic': 10.0,")
    print("}")
    
    print("\n2. Medium Terrain - Balanced Parameters")
    print("-" * 70)
    print("'tamols_params': {")
    print("    'search_radius': 0.20,")
    print("    'search_resolution': 0.025,")
    print("    'weight_edge_avoidance': 8.0,")
    print("    'weight_roughness': 3.0,")
    print("    'weight_deviation': 0.5,")
    print("    'weight_kinematic': 10.0,")
    print("}")
    
    print("\n3. Hard Terrain - Aggressive Parameters")
    print("-" * 70)
    print("'tamols_params': {")
    print("    'search_radius': 0.25,")
    print("    'search_resolution': 0.02,")
    print("    'weight_edge_avoidance': 10.0,")
    print("    'weight_roughness': 4.0,")
    print("    'weight_deviation': 0.3,")
    print("    'weight_kinematic': 12.0,")
    print("}")
    
    print("\n4. Sparse Terrain - Extended Reach Parameters")
    print("-" * 70)
    print("'tamols_params': {")
    print("    'search_radius': 0.30,")
    print("    'search_resolution': 0.025,")
    print("    'weight_edge_avoidance': 12.0,")
    print("    'weight_roughness': 5.0,")
    print("    'weight_deviation': 0.2,")
    print("    'weight_kinematic': 15.0,")
    print("}")


def visualize_terrain_info():
    """Print visual representation of terrain layout."""
    
    print("\n" + "=" * 70)
    print("Terrain Layout Visualization")
    print("=" * 70)
    
    print("""
    Side View (not to scale):
    
         Flat1   Stones    Flat2
           ___   • • •     ___
          /   |  • • •    |   \\
         /    |  • • •    |    \\
      Up|     |  • • •    |     |Down
       /      |  • • •    |      \\
      /       |_ • • • ___|       \\
    
    ↗ Uphill  → Flat → Stones → Flat → Downhill ↘
    
    Top View (石头布局):
    
    Forward Direction (X) →
    
    Row 1:    •        •        •
    Row 2:       •        •        •    (Offset)
    Row 3:    •        •        •
    Row 4:       •        •        •    (Offset)
    Row 5:    •        •        •
    
    ← Left | Lateral (Y) | Right →
    
    This alternating pattern creates the "plum blossom" formation
    that challenges the robot's foothold planning capabilities.
    """)


if __name__ == "__main__":
    print("=" * 70)
    print("Stepping Stones Terrain Generator")
    print("梅花桩地图生成器")
    print("=" * 70)
    
    # Generate standard terrains
    generate_standard_terrains()
    
    # Print TAMOLS recommendations
    print_tamols_recommendations()
    
    # Show terrain visualization
    visualize_terrain_info()
    
    print("\n" + "=" * 70)
    print("For more information, see:")
    print("  docs/STEPPING_STONES_TERRAIN.md")
    print("  docs/TAMOLS_FOOTHOLD_ADAPTATION.md")
    print("=" * 70)
