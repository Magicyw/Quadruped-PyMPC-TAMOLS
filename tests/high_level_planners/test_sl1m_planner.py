"""Smoke tests for Sl1mFootholdPlanner."""

import pytest
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc.high_level_planners.sl1m_planner import (
    Sl1mFootholdPlanner,
    HighLevelPlan,
)


class TestSl1mFootholdPlanner:
    """Smoke tests for Sl1mFootholdPlanner instantiation and basic usage."""
    
    @pytest.fixture
    def pile_config(self):
        """Provide sample pile configuration."""
        return {
            'x_range': (4.0, 6.0),
            'y_range': (-0.4, 0.4),
            'x_step': 0.2,
            'y_step': 0.2,
            'radius': 0.08,
            'height': 0.268,
            'constraint_radius': 0.06,
        }
    
    def test_planner_instantiation(self, pile_config):
        """Test that planner can be instantiated."""
        planner = Sl1mFootholdPlanner(
            pile_config=pile_config,
            planning_horizon=4,
            use_sl1m=False,  # Use heuristic to avoid sl1m dependency
        )
        
        assert planner is not None
        assert len(planner.piles) > 0, "Should have generated pile grid"
    
    def test_planner_basic_plan(self, pile_config):
        """Test that planner can generate a basic plan."""
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
        
        # Simulate FL in swing
        current_contact = np.array([0, 1, 1, 1])
        
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
        assert isinstance(plan, HighLevelPlan)
        assert 'FL' in plan.footholds
        assert 'FR' in plan.footholds
        assert 'RL' in plan.footholds
        assert 'RR' in plan.footholds
        
        # Verify constraint regions
        assert plan.foothold_constraints['FL'] is not None
        assert plan.foothold_constraints['FL'].region_type == 'circle'
        assert plan.foothold_constraints['FL'].radius == pile_config['constraint_radius']
        
        # Verify Mode A (no contact schedule)
        assert plan.contact_schedule is None, "Mode A should not provide contact schedule"
        
        # Verify metadata
        assert 'planner_type' in plan.metadata
        assert plan.metadata['planner_type'] == 'heuristic'
    
    def test_planner_foothold_within_reach(self, pile_config):
        """Test that generated footholds are within reasonable reach."""
        planner = Sl1mFootholdPlanner(
            pile_config=pile_config,
            planning_horizon=4,
            use_sl1m=False,
        )
        
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
        
        current_contact = np.array([0, 1, 1, 1])
        
        plan = planner.plan(
            base_position=base_position,
            base_orientation=base_orientation,
            base_velocity=base_velocity,
            feet_positions=feet_positions,
            current_contact=current_contact,
            reference_velocity=reference_velocity,
        )
        
        # Check that footholds are within reasonable distance of base
        max_reach = 1.0  # meters
        for leg in ['FL', 'FR', 'RL', 'RR']:
            foothold = plan.footholds[leg]
            distance = np.linalg.norm(foothold - base_position)
            assert distance < max_reach, f"Foothold for {leg} too far from base: {distance}m"
    
    def test_planner_reset(self, pile_config):
        """Test planner reset functionality."""
        planner = Sl1mFootholdPlanner(
            pile_config=pile_config,
            planning_horizon=4,
            use_sl1m=False,
        )
        
        # Add some cached data
        planner.cached_plan['FL'] = [np.array([1.0, 2.0, 3.0])]
        planner.current_swing_leg = 'FL'
        
        # Reset
        planner.reset()
        
        # Verify reset
        assert len(planner.cached_plan['FL']) == 0
        assert planner.current_swing_leg is None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
