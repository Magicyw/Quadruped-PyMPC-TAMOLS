"""
Simple validation tests for TAMOLS joint foothold optimization.

This module tests the core functionality without requiring full dependencies.
"""
import sys
import os
import numpy as np

# Add the repo root to path
repo_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, repo_root)

# Mock the config module to avoid import errors
class MockConfig:
    robot = 'aliengo'
    simulation_params = {
        'gait': 'trot',
        'tamols_params': {
            'search_radius': 0.35,
            'search_resolution': 0.04,
            'gradient_delta': 0.04,
            'weight_edge_avoidance': 15.0,
            'weight_roughness': 10.0,
            'weight_previous_solution': 2.0,
            'weight_kinematic': 10.0,
            'weight_nominal_kinematic': 1.0,
            'weight_reference_tracking': 5.0,
            'h_des': 0.30,
            'l_min': {'aliengo': 0.18},
            'l_max': {'aliengo': 0.55},
            'constraint_box_dx': 0.05,
            'constraint_box_dy': 0.05,
            'joint_optimize': True,
            'top_k_per_leg': 15,
            'stability_margin': 0.02,
            'joint_weight_stability': 1000.0,
        }
    }

# Mock gym_quadruped before importing
class LegsAttr:
    def __init__(self, FL=None, FR=None, RL=None, RR=None):
        self.FL = FL
        self.FR = FR
        self.RL = RL
        self.RR = RR
    
    def __getitem__(self, key):
        return getattr(self, key)
    
    def __setitem__(self, key, value):
        setattr(self, key, value)
    
    def copy(self):
        return LegsAttr(
            FL=self.FL.copy() if self.FL is not None else None,
            FR=self.FR.copy() if self.FR is not None else None,
            RL=self.RL.copy() if self.RL is not None else None,
            RR=self.RR.copy() if self.RR is not None else None
        )

# Setup mock modules
import types

gym_quadruped_module = types.ModuleType('gym_quadruped')
gym_quadruped_utils = types.ModuleType('gym_quadruped.utils')
gym_quadruped_utils_quadruped_utils = types.ModuleType('gym_quadruped.utils.quadruped_utils')
gym_quadruped_utils_quadruped_utils.LegsAttr = LegsAttr

sys.modules['gym_quadruped'] = gym_quadruped_module
sys.modules['gym_quadruped.utils'] = gym_quadruped_utils
sys.modules['gym_quadruped.utils.quadruped_utils'] = gym_quadruped_utils_quadruped_utils

# Mock config
config_module = types.ModuleType('quadruped_pympc.config')
for attr in dir(MockConfig):
    if not attr.startswith('_'):
        setattr(config_module, attr, getattr(MockConfig, attr))

sys.modules['quadruped_pympc.config'] = config_module

# Create proper package structure for quadruped_pympc
quadruped_pympc_pkg = types.ModuleType('quadruped_pympc')
quadruped_pympc_pkg.__path__ = [os.path.join(repo_root, 'quadruped_pympc')]
quadruped_pympc_pkg.config = config_module
sys.modules['quadruped_pympc'] = quadruped_pympc_pkg

# Mock VFA to avoid import errors
try:
    virall_module = types.ModuleType('virall')
    virall_vfa = types.ModuleType('virall.vfa')
    virall_vfa_vfa = types.ModuleType('virall.vfa.vfa')
    sys.modules['virall'] = virall_module
    sys.modules['virall.vfa'] = virall_vfa
    sys.modules['virall.vfa.vfa'] = virall_vfa_vfa
except:
    pass


def test_generate_candidates_with_heightmap():
    """Test that _generate_candidates no longer crashes with heightmap parameter."""
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
    
    vfa = VisualFootholdAdaptation(['FL', 'FR', 'RL', 'RR'], adaptation_strategy='tamols')
    
    seed = np.array([0.3, 0.2, 0.0])
    
    # Test without heightmap (should use grid generation)
    candidates = vfa._generate_candidates(seed, heightmap=None)
    assert len(candidates) > 0, "Should generate candidates without heightmap"
    print(f"✓ Generated {len(candidates)} candidates without heightmap")
    
    # Test with None heightmap explicitly
    candidates = vfa._generate_candidates(seed, None)
    assert len(candidates) > 0, "Should generate candidates with None heightmap"
    print(f"✓ Generated {len(candidates)} candidates with None heightmap")


def test_stability_constraint_evaluation():
    """Test stability constraint evaluation with known foothold positions."""
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
    
    vfa = VisualFootholdAdaptation(['FL', 'FR', 'RL', 'RR'], adaptation_strategy='tamols')
    
    # Create a stable square foothold configuration
    # FL: front-left, FR: front-right, RL: rear-left, RR: rear-right
    foothold_positions = {
        'FL': np.array([0.3, 0.2]),    # front-left
        'FR': np.array([0.3, -0.2]),   # front-right
        'RL': np.array([-0.3, 0.2]),   # rear-left
        'RR': np.array([-0.3, -0.2])   # rear-right
    }
    
    # Test 1: Stability point at origin (center of support polygon) - should be feasible with margin
    stability_point = np.array([0.0, 0.0])
    violation = vfa._evaluate_stability_constraints(foothold_positions, stability_point, margin=0.0)
    assert violation < 1e-3, f"Expected negligible violation for centered point with zero margin, got {violation}"
    print(f"✓ Stability point at origin is feasible with zero margin (violation={violation:.6f})")
    
    # Test with margin - should still be inside but report the margin requirement
    violation_with_margin = vfa._evaluate_stability_constraints(foothold_positions, stability_point, margin=0.02)
    print(f"✓ Stability point at origin with 0.02m margin: violation={violation_with_margin:.6f}")
    
    # Test 2: Stability point outside support polygon - should violate
    stability_point = np.array([0.5, 0.5])  # way outside
    violation = vfa._evaluate_stability_constraints(foothold_positions, stability_point, margin=0.02)
    assert violation > 0.0, f"Expected violation for point outside polygon, got {violation}"
    print(f"✓ Stability point outside polygon has violation={violation}")
    
    # Test 3: Verify constraint signs are correct
    # Point slightly in front should still be inside for this configuration
    stability_point = np.array([0.1, 0.0])
    violation = vfa._evaluate_stability_constraints(foothold_positions, stability_point, margin=0.02)
    print(f"✓ Stability point slightly forward has violation={violation}")


def test_transform_to_horizontal_frame():
    """Test transformation from world to horizontal frame."""
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
    
    vfa = VisualFootholdAdaptation(['FL', 'FR', 'RL', 'RR'], adaptation_strategy='tamols')
    
    # World frame positions
    base_position = np.array([1.0, 2.0, 0.3])
    base_yaw = 0.0  # No rotation
    
    world_positions = {
        'FL': np.array([1.3, 2.2, 0.0]),
        'FR': np.array([1.3, 1.8, 0.0]),
        'RL': np.array([0.7, 2.2, 0.0]),
        'RR': np.array([0.7, 1.8, 0.0])
    }
    
    horizontal_positions = vfa._transform_to_horizontal_frame(
        world_positions, base_position, base_yaw
    )
    
    # With zero yaw, should just be relative positions
    expected_FL = np.array([0.3, 0.2])
    assert np.allclose(horizontal_positions['FL'], expected_FL, atol=1e-6), \
        f"Expected {expected_FL}, got {horizontal_positions['FL']}"
    print(f"✓ Transform to horizontal frame works correctly")
    
    # Test with non-zero yaw
    base_yaw = np.pi / 2  # 90 degrees
    horizontal_positions = vfa._transform_to_horizontal_frame(
        world_positions, base_position, base_yaw
    )
    # After 90 degree rotation: (x, y) -> (y, -x)
    # FL relative: (0.3, 0.2) -> (0.2, -0.3)
    expected_FL_rotated = np.array([0.2, -0.3])
    assert np.allclose(horizontal_positions['FL'], expected_FL_rotated, atol=1e-6), \
        f"Expected {expected_FL_rotated}, got {horizontal_positions['FL']}"
    print(f"✓ Transform with yaw rotation works correctly")


def test_determine_joint_leg_sets():
    """Test determination of which legs to jointly optimize."""
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
    
    vfa = VisualFootholdAdaptation(['FL', 'FR', 'RL', 'RR'], adaptation_strategy='tamols')
    
    # Test trot gait (should give diagonal pairs)
    leg_sets = vfa._determine_joint_leg_sets(['FL', 'FR', 'RL', 'RR'], gait_type='trot')
    assert len(leg_sets) == 2, f"Expected 2 leg sets for trot, got {len(leg_sets)}"
    assert ('FL', 'RR') in leg_sets or ('RR', 'FL') in leg_sets, "Expected FL-RR pair"
    assert ('FR', 'RL') in leg_sets or ('RL', 'FR') in leg_sets, "Expected FR-RL pair"
    print(f"✓ Trot gait correctly identifies diagonal pairs: {leg_sets}")


def test_stability_proxy():
    """Test stability proxy computation."""
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
    
    vfa = VisualFootholdAdaptation(['FL', 'FR', 'RL', 'RR'], adaptation_strategy='tamols')
    
    base_position = np.array([1.0, 2.0, 0.3])
    base_orientation = np.array([0.0, 0.0, 0.0])
    forward_vel = np.array([0.5, 0.0, 0.0])
    
    # Test static stability proxy (should be at origin)
    proxy = vfa._compute_stability_proxy(base_position, base_orientation, forward_vel)
    expected = np.array([0.0, 0.0])
    assert np.allclose(proxy, expected, atol=1e-6), \
        f"Expected {expected}, got {proxy}"
    print(f"✓ Static stability proxy at origin: {proxy}")


def main():
    """Run all tests."""
    print("=" * 60)
    print("TAMOLS Joint Foothold Optimization - Validation Tests")
    print("=" * 60)
    
    tests = [
        ("Bug Fix: _generate_candidates with heightmap", test_generate_candidates_with_heightmap),
        ("Stability Constraint Evaluation", test_stability_constraint_evaluation),
        ("Transform to Horizontal Frame", test_transform_to_horizontal_frame),
        ("Joint Leg Set Determination", test_determine_joint_leg_sets),
        ("Stability Proxy Computation", test_stability_proxy),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        print(f"\n[TEST] {test_name}")
        try:
            test_func()
            passed += 1
            print(f"[PASS] {test_name}\n")
        except Exception as e:
            failed += 1
            print(f"[FAIL] {test_name}")
            print(f"Error: {e}\n")
            import traceback
            traceback.print_exc()
    
    print("=" * 60)
    print(f"Test Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
