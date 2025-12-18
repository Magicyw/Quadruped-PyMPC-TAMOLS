"""
Helper module for loading custom MuJoCo scenes in simulation.

This module provides utilities to load stepping stones terrains and other
custom scene XML files for use with gym_quadruped.

IMPORTANT: gym_quadruped expects scene names to refer to XML files in its 
internal scene/xml directory. For custom terrains like stepping stones, 
you need to either:
1. Copy the XML files to gym_quadruped's scene/xml directory, or
2. Use the install_stepping_stones_to_gym_quadruped() function
"""

import shutil
from pathlib import Path


def get_gym_quadruped_scene_dir():
    """Get the scene directory path for gym_quadruped.
    
    Returns:
        Path or None: Path to gym_quadruped's scene/xml directory, or None if not found
    """
    try:
        import gym_quadruped
        gym_quadruped_dir = Path(gym_quadruped.__file__).parent
        scene_dir = gym_quadruped_dir / 'scene' / 'xml'
        if scene_dir.exists():
            return scene_dir
        return None
    except ImportError:
        return None


def install_stepping_stones_to_gym_quadruped():
    """Copy stepping stones terrain files to gym_quadruped's scene directory.
    
    This function copies all stepping stones XML files from the simulation
    directory to gym_quadruped's scene/xml directory so they can be used
    with QuadrupedEnv.
    
    Returns:
        bool: True if successful, False otherwise
    """
    scene_dir = get_gym_quadruped_scene_dir()
    if scene_dir is None:
        print("ERROR: gym_quadruped not found or scene directory doesn't exist")
        return False
    
    sim_dir = Path(__file__).parent
    
    stepping_stones_files = [
        'stepping_stones_terrain.xml',
        'stepping_stones_easy.xml',
        'stepping_stones_medium.xml',
        'stepping_stones_hard.xml',
        'stepping_stones_sparse.xml',
    ]
    
    copied_count = 0
    for filename in stepping_stones_files:
        src = sim_dir / filename
        dst = scene_dir / filename
        
        if src.exists():
            try:
                shutil.copy2(src, dst)
                print(f"✓ Copied {filename} to {scene_dir}")
                copied_count += 1
            except Exception as e:
                print(f"✗ Failed to copy {filename}: {e}")
        else:
            print(f"⚠ Source file not found: {src}")
    
    if copied_count > 0:
        print(f"\n✓ Successfully installed {copied_count} stepping stones terrain files")
        print(f"  Location: {scene_dir}")
        print("\nYou can now use them in config.py with:")
        print("  simulation_params['scene'] = 'stepping_stones_medium'")
        return True
    else:
        print("\n✗ No terrain files were copied")
        print("  Please run 'python simulation/example_stepping_stones.py' first")
        return False


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
    
    IMPORTANT: gym_quadruped.QuadrupedEnv expects scene names to refer to XML files
    in its internal scene/xml directory, not arbitrary file paths.
    
    For stepping stones terrains, you must first install them to gym_quadruped:
        from simulation.scene_loader import install_stepping_stones_to_gym_quadruped
        install_stepping_stones_to_gym_quadruped()
    
    Or run the installer script:
        python simulation/install_stepping_stones.py
    
    Args:
        scene_name: Scene identifier (see get_scene_path for options)
        stepping_stones_params: Optional custom parameters for stepping stones
    
    Returns:
        tuple: (scene_arg, custom_xml_path)
            - scene_arg: Value to pass to QuadrupedEnv(scene=...)
            - custom_xml_path: Path to custom XML file or None
    
    Raises:
        RuntimeError: If stepping stones scene is requested but not installed in gym_quadruped
    """
    # Built-in gym_quadruped scenes - pass name directly
    builtin_scenes = ['flat', 'random_boxes', 'random_pyramids', 'perlin']
    if scene_name in builtin_scenes:
        return scene_name, None
    
    # Stepping stones scenes - need to be installed in gym_quadruped
    stepping_stones_variants = [
        'stepping_stones', 'stepping_stones_easy', 'stepping_stones_medium',
        'stepping_stones_hard', 'stepping_stones_sparse'
    ]
    
    if scene_name in stepping_stones_variants:
        # Check if the terrain file exists in gym_quadruped's directory
        scene_dir = get_gym_quadruped_scene_dir()
        if scene_dir is None:
            raise RuntimeError(
                "gym_quadruped is not installed or its scene directory cannot be found.\n"
                "Please install gym_quadruped first."
            )
        
        # Map scene name to XML filename
        xml_filename_map = {
            'stepping_stones': 'stepping_stones_terrain.xml',
            'stepping_stones_easy': 'stepping_stones_easy.xml',
            'stepping_stones_medium': 'stepping_stones_medium.xml',
            'stepping_stones_hard': 'stepping_stones_hard.xml',
            'stepping_stones_sparse': 'stepping_stones_sparse.xml',
        }
        
        xml_filename = xml_filename_map.get(scene_name)
        if xml_filename:
            xml_path = scene_dir / xml_filename
            
            if not xml_path.exists():
                raise FileNotFoundError(
                    f"\n{'='*70}\n"
                    f"ERROR: Stepping stones terrain not installed in gym_quadruped\n"
                    f"{'='*70}\n\n"
                    f"The terrain file '{xml_filename}' was not found in gym_quadruped's scene directory.\n"
                    f"Expected location: {xml_path}\n\n"
                    f"To fix this, run ONE of the following:\n\n"
                    f"1. Run the installer script (recommended):\n"
                    f"   cd simulation\n"
                    f"   python install_stepping_stones.py\n\n"
                    f"2. Run in Python:\n"
                    f"   from simulation.scene_loader import install_stepping_stones_to_gym_quadruped\n"
                    f"   install_stepping_stones_to_gym_quadruped()\n\n"
                    f"3. Manually copy files:\n"
                    f"   cp simulation/stepping_stones_*.xml {scene_dir}/\n\n"
                    f"Note: You need to generate the terrain files first if not already done:\n"
                    f"   python simulation/example_stepping_stones.py\n"
                    f"{'='*70}\n"
                )
            
            # Return just the filename (without .xml extension) as gym_quadruped expects
            scene_basename = xml_filename.replace('.xml', '')
            return scene_basename, str(xml_path)
    
    # For other custom scenes, try to get the path
    scene_path = get_scene_path(scene_name, stepping_stones_params)
    
    if scene_path is None:
        # Shouldn't reach here, but just in case
        return scene_name, None
    else:
        # Custom path - gym_quadruped may not support this
        # Return the name and path, but warn that it might not work
        print("WARNING: gym_quadruped may not support custom scene paths.")
        print(f"If this fails, copy your scene XML to: {get_gym_quadruped_scene_dir()}")
        return scene_name, scene_path


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
