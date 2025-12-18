"""
Helper module for loading custom MuJoCo scenes in simulation.

This module provides utilities to load stepping stones terrains and other
custom scene XML files for use with gym_quadruped.
"""

from pathlib import Path


def get_scene_path(scene_name, stepping_stones_params=None):
    """Get the path to a scene file or handle built-in scene names.
    
    Args:
        scene_name: Scene identifier. Can be:
            - Built-in gym_quadruped scene: 'flat', 'random_boxes', 'random_pyramids', 'perlin'
            - Stepping stones preset: 'stepping_stones_easy', 'stepping_stones_medium',
                                     'stepping_stones_hard', 'stepping_stones_sparse',
                                     'stepping_stones' (default)
            - Custom path: Full path to XML file
        stepping_stones_params: Optional dict of parameters to generate custom stepping stones
                              terrain on-the-fly (if None, uses pre-generated files)
    
    Returns:
        str or None: Path to scene XML file, or None for built-in scenes
    """
    # Built-in gym_quadruped scenes - return None to use default behavior
    builtin_scenes = ['flat', 'random_boxes', 'random_pyramids', 'perlin']
    if scene_name in builtin_scenes:
        return None
    
    # Get simulation directory path
    sim_dir = Path(__file__).parent
    
    # Handle stepping stones scenes
    stepping_stones_variants = {
        'stepping_stones': 'stepping_stones_terrain.xml',
        'stepping_stones_easy': 'stepping_stones_easy.xml',
        'stepping_stones_medium': 'stepping_stones_medium.xml',
        'stepping_stones_hard': 'stepping_stones_hard.xml',
        'stepping_stones_sparse': 'stepping_stones_sparse.xml',
    }
    
    if scene_name in stepping_stones_variants:
        # Check if custom parameters are provided
        if stepping_stones_params is not None:
            # Generate custom terrain on-the-fly
            from stepping_stones_scene import create_stepping_stones_scene
            
            xml_content = create_stepping_stones_scene(**stepping_stones_params)
            
            # Save to temporary file in simulation directory
            temp_xml_path = sim_dir / "stepping_stones_custom_temp.xml"
            with open(temp_xml_path, 'w') as f:
                f.write(xml_content)
            
            print(f"Generated custom stepping stones terrain at: {temp_xml_path}")
            return str(temp_xml_path)
        else:
            # Use pre-generated XML file
            xml_filename = stepping_stones_variants[scene_name]
            xml_path = sim_dir / xml_filename
            
            if not xml_path.exists():
                raise FileNotFoundError(
                    f"Stepping stones terrain file not found: {xml_path}\n"
                    f"Please run 'python simulation/example_stepping_stones.py' to generate terrain files."
                )
            
            return str(xml_path)
    
    # Handle custom path (absolute or relative)
    custom_path = Path(scene_name)
    if custom_path.is_absolute():
        if not custom_path.exists():
            raise FileNotFoundError(f"Custom scene file not found: {custom_path}")
        return str(custom_path)
    else:
        # Try relative to simulation directory
        rel_path = sim_dir / custom_path
        if rel_path.exists():
            return str(rel_path)
        else:
            raise FileNotFoundError(
                f"Custom scene file not found: {custom_path}\n"
                f"Tried absolute path and relative to simulation directory."
            )


def load_scene_for_quadruped_env(scene_name, stepping_stones_params=None):
    """Prepare scene configuration for QuadrupedEnv.
    
    Args:
        scene_name: Scene identifier (see get_scene_path for options)
        stepping_stones_params: Optional custom parameters for stepping stones
    
    Returns:
        tuple: (scene_arg, custom_xml_path)
            - scene_arg: Value to pass to QuadrupedEnv(scene=...)
            - custom_xml_path: Path to custom XML file or None
    """
    scene_path = get_scene_path(scene_name, stepping_stones_params)
    
    if scene_path is None:
        # Built-in scene - pass name directly
        return scene_name, None
    else:
        # Custom scene - check if gym_quadruped supports custom XML
        # For now, we'll return the path and let the caller handle it
        return scene_path, scene_path


def print_available_scenes():
    """Print all available scene options."""
    print("=" * 70)
    print("Available Scenes for Quadruped Simulation")
    print("=" * 70)
    
    print("\n1. Built-in Scenes (gym_quadruped):")
    print("   - 'flat': Flat terrain")
    print("   - 'random_boxes': Random box obstacles")
    print("   - 'random_pyramids': Random pyramid obstacles")
    print("   - 'perlin': Perlin noise terrain")
    
    print("\n2. Stepping Stones Terrains (梅花桩地图):")
    print("   - 'stepping_stones': Default stepping stones (medium difficulty)")
    print("   - 'stepping_stones_easy': Easy difficulty (larger stones, closer spacing)")
    print("   - 'stepping_stones_medium': Medium difficulty")
    print("   - 'stepping_stones_hard': Hard difficulty (smaller stones, wider spacing)")
    print("   - 'stepping_stones_sparse': Sparse layout (2 stones per row)")
    
    print("\n3. Custom Scenes:")
    print("   - Provide full path to custom MuJoCo XML file")
    
    print("\n" + "=" * 70)
    print("Usage in config.py:")
    print("=" * 70)
    print("simulation_params = {")
    print("    'scene': 'stepping_stones_medium',  # Use medium difficulty stepping stones")
    print("    'visual_foothold_adaptation': 'tamols',  # Enable TAMOLS for terrain")
    print("    'tamols_params': {")
    print("        'search_radius': 0.20,  # Increase for stepping stones")
    print("        'weight_edge_avoidance': 8.0,  # Higher for edges between stones")
    print("    },")
    print("}")
    print("=" * 70)


if __name__ == "__main__":
    # Test scene loading
    print_available_scenes()
    
    print("\n\nTesting scene path resolution:")
    print("-" * 70)
    
    # Test built-in scene
    try:
        scene, path = load_scene_for_quadruped_env('flat')
        print(f"✓ 'flat' -> scene={scene}, path={path}")
    except Exception as e:
        print(f"✗ 'flat' failed: {e}")
    
    # Test stepping stones
    try:
        scene, path = load_scene_for_quadruped_env('stepping_stones_medium')
        print(f"✓ 'stepping_stones_medium' -> scene={scene}, path={path}")
    except Exception as e:
        print(f"✗ 'stepping_stones_medium' failed: {e}")
    
    print("-" * 70)
