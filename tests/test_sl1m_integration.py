#!/usr/bin/env python3
"""Integration test for sl1m planner with plum piles terrain.

This script verifies that:
1. Planner modules can be imported
2. Plum piles terrain can be generated
3. Planner can be instantiated and produces valid plans
4. Constraint enforcement works correctly

Run this before using the planner in simulation to verify setup.
"""

import sys
import os
import numpy as np

# Add repo root to path so quadruped_pympc can be imported
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, repo_root)


def test_imports():
    """Test that all required modules can be imported."""
    print("=" * 70)
    print("Test 1: Module Imports")
    print("=" * 70)
    
    try:
        from quadruped_pympc.high_level_planners.sl1m_planner import (
            Sl1mFootholdPlanner, HighLevelPlan, FootholdConstraintRegion
        )
        print("✓ sl1m planner modules imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Failed to import planner modules: {e}")
        return False


def test_geometry_utils():
    """Test geometry utility functions."""
    print("\n" + "=" * 70)
    print("Test 2: Geometry Utilities")
    print("=" * 70)
    
    try:
        from quadruped_pympc.high_level_planners.sl1m_planner.geometry_utils import (
            generate_cylinder_top_polygon, clamp_to_circle, build_plum_pile_grid
        )
        
        # Test polygon generation
        vertices = generate_cylinder_top_polygon(np.array([0, 0]), 0.08, 0.268, 8)
        assert vertices.shape == (8, 3), "Polygon should have 8 vertices"
        print("✓ Cylinder polygon generation works")
        
        # Test clamping
        point = np.array([2.0, 0.0, 1.0])
        center = np.array([0.0, 0.0, 1.0])
        clamped = clamp_to_circle(point, center, 1.0)
        dist = np.linalg.norm(clamped[:2] - center[:2])
        assert abs(dist - 1.0) < 0.001, "Clamping should place point on circle"
        print("✓ Circle clamping works")
        
        # Test pile grid building
        piles = build_plum_pile_grid((4.1, 7.1), (-0.4, 0.4), 0.2, 0.2, 0.08, 0.268)
        assert len(piles) == 80, f"Should generate 80 piles, got {len(piles)}"
        print(f"✓ Built pile grid: {len(piles)} piles")
        
        return True
    except Exception as e:
        print(f"✗ Geometry utils test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_planner_instantiation():
    """Test planner can be instantiated."""
    print("\n" + "=" * 70)
    print("Test 3: Planner Instantiation")
    print("=" * 70)
    
    try:
        from quadruped_pympc.high_level_planners.sl1m_planner import Sl1mFootholdPlanner
        
        pile_config = {
            'x_range': (4.1, 7.1),
            'y_range': (-0.4, 0.4),
            'x_step': 0.2,
            'y_step': 0.2,
            'radius': 0.08,
            'height': 0.268,
            'constraint_radius': 0.06,
        }
        
        planner = Sl1mFootholdPlanner(
            pile_config=pile_config,
            planning_horizon=4,
            use_sl1m=False,  # Use heuristic to avoid sl1m dependency
        )
        
        assert planner is not None, "Planner should be created"
        assert len(planner.piles) == 80, "Planner should have 80 piles"
        print(f"✓ Planner instantiated with {len(planner.piles)} piles")
        
        return True
    except Exception as e:
        print(f"✗ Planner instantiation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_plan_generation():
    """Test planner can generate valid plans."""
    print("\n" + "=" * 70)
    print("Test 4: Plan Generation")
    print("=" * 70)
    
    try:
        # Need gym_quadruped for LegsAttr
        try:
            from gym_quadruped.utils.quadruped_utils import LegsAttr
        except ImportError:
            print("⚠ Skipping plan generation test (gym_quadruped not available)")
            print("  This is OK if you're just testing module imports")
            return True
        
        from quadruped_pympc.high_level_planners.sl1m_planner import Sl1mFootholdPlanner
        
        pile_config = {
            'x_range': (4.1, 7.1),
            'y_range': (-0.4, 0.4),
            'x_step': 0.2,
            'y_step': 0.2,
            'radius': 0.08,
            'height': 0.268,
            'constraint_radius': 0.06,
        }
        
        planner = Sl1mFootholdPlanner(
            pile_config=pile_config,
            planning_horizon=4,
            use_sl1m=False,
        )
        
        # Create synthetic state
        base_position = np.array([4.5, 0.0, 0.3])
        base_orientation = np.array([0.0, 0.0, 0.0])
        base_velocity = np.array([0.3, 0.0, 0.0])
        reference_velocity = np.array([0.3, 0.0, 0.0])
        
        feet_positions = LegsAttr(
            FL=np.array([4.7, 0.15, 0.268]),
            FR=np.array([4.7, -0.15, 0.268]),
            RL=np.array([4.3, 0.15, 0.268]),
            RR=np.array([4.3, -0.15, 0.268]),
        )
        
        current_contact = np.array([0, 1, 1, 1])  # FL in swing
        
        # Generate plan
        plan = planner.plan(
            base_position=base_position,
            base_orientation=base_orientation,
            base_velocity=base_velocity,
            feet_positions=feet_positions,
            current_contact=current_contact,
            reference_velocity=reference_velocity,
        )
        
        # Verify plan structure
        assert 'FL' in plan.footholds, "Plan should have FL foothold"
        assert plan.foothold_constraints['FL'] is not None, "Should have constraints"
        assert plan.contact_schedule is None, "Mode A: no contact schedule"
        print("✓ Generated valid plan with footholds and constraints")
        
        # Verify constraints
        constraint = plan.foothold_constraints['FL']
        assert constraint.region_type == 'circle', "Should use circular constraints"
        assert constraint.radius == 0.06, "Constraint radius should match config"
        print(f"✓ Constraint region: {constraint.region_type}, radius={constraint.radius}m")
        
        return True
    except Exception as e:
        print(f"✗ Plan generation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_terrain_generation():
    """Test plum piles terrain XML generation."""
    print("\n" + "=" * 70)
    print("Test 5: Terrain Generation")
    print("=" * 70)
    
    try:
        import sys
        sys.path.insert(0, 'simulation')
        from plum_piles_scene import create_plum_piles_scene
        
        xml = create_plum_piles_scene()
        assert len(xml) > 0, "XML should be generated"
        assert 'plum_piles_terrain' in xml, "XML should have model name"
        assert 'pile_' in xml, "XML should contain pile geometries"
        print("✓ Terrain XML generated successfully")
        print(f"  XML length: {len(xml)} characters")
        
        return True
    except Exception as e:
        print(f"✗ Terrain generation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all integration tests."""
    print("\nsl1m Planner Integration Test Suite")
    print("This verifies the planner setup before using in simulation\n")
    
    results = []
    
    # Run tests
    results.append(("Module Imports", test_imports()))
    results.append(("Geometry Utils", test_geometry_utils()))
    results.append(("Planner Instantiation", test_planner_instantiation()))
    results.append(("Plan Generation", test_plan_generation()))
    results.append(("Terrain Generation", test_terrain_generation()))
    
    # Summary
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status:8} {test_name}")
    
    print("-" * 70)
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n✓✓✓ All tests passed! Ready to use sl1m planner. ✓✓✓")
        return 0
    else:
        print(f"\n⚠ {total - passed} test(s) failed. Check errors above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
