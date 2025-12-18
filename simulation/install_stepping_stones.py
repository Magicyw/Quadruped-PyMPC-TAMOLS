#!/usr/bin/env python3
"""
Installer script for stepping stones terrain files.

This script copies the stepping stones terrain XML files to gym_quadruped's
scene directory so they can be used with QuadrupedEnv.

Usage:
    python install_stepping_stones.py
"""

import sys
from pathlib import Path


def main():
    """Main installer function."""
    print("=" * 70)
    print("Stepping Stones Terrain Installer")
    print("梅花桩地图安装工具")
    print("=" * 70)
    print()
    
    # Check if terrain files exist
    sim_dir = Path(__file__).parent
    terrain_files = [
        'stepping_stones_terrain.xml',
        'stepping_stones_easy.xml',
        'stepping_stones_medium.xml',
        'stepping_stones_hard.xml',
        'stepping_stones_sparse.xml',
    ]
    
    missing_files = [f for f in terrain_files if not (sim_dir / f).exists()]
    
    if missing_files:
        print("⚠ WARNING: Some terrain files are missing:")
        for f in missing_files:
            print(f"  - {f}")
        print()
        print("Generating terrain files now...")
        print("-" * 70)
        
        # Try to generate them
        try:
            from example_stepping_stones import generate_standard_terrains
            generate_standard_terrains()
            print()
        except Exception as e:
            print(f"✗ Failed to generate terrain files: {e}")
            print()
            print("Please run manually:")
            print("  python simulation/example_stepping_stones.py")
            return 1
    
    # Install to gym_quadruped
    print()
    print("Installing to gym_quadruped...")
    print("-" * 70)
    
    try:
        from scene_loader import install_stepping_stones_to_gym_quadruped
        success = install_stepping_stones_to_gym_quadruped()
        
        if success:
            print()
            print("=" * 70)
            print("✓ Installation Complete!")
            print("=" * 70)
            print()
            print("You can now use stepping stones terrains in your simulation!")
            print()
            print("Example configuration in config.py:")
            print("-" * 70)
            print("simulation_params = {")
            print("    'scene': 'stepping_stones_medium',")
            print("    'visual_foothold_adaptation': 'tamols',")
            print("    'tamols_params': {")
            print("        'search_radius': 0.20,")
            print("        'weight_edge_avoidance': 8.0,")
            print("    },")
            print("}")
            print("-" * 70)
            print()
            print("Available scenes:")
            print("  - stepping_stones_easy")
            print("  - stepping_stones_medium (recommended)")
            print("  - stepping_stones_hard")
            print("  - stepping_stones_sparse")
            print()
            return 0
        else:
            print()
            print("=" * 70)
            print("✗ Installation Failed")
            print("=" * 70)
            print()
            print("Please check the error messages above.")
            return 1
            
    except ImportError as e:
        print(f"✗ Import error: {e}")
        print()
        print("Make sure you are in the simulation directory:")
        print("  cd simulation")
        print("  python install_stepping_stones.py")
        return 1
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
