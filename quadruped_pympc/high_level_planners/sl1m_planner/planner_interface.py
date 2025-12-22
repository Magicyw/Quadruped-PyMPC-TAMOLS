"""Interface protocol for high-level contact and foothold planners."""

from typing import Protocol, Dict, Optional
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from .data_types import HighLevelPlan


class HighLevelContactPlanner(Protocol):
    """Protocol defining the interface for high-level contact planners.
    
    High-level planners compute foothold locations and optionally contact
    schedules over a planning horizon. They can use optimization-based
    methods (e.g., sl1m) or heuristic approaches.
    """
    
    def plan(
        self,
        base_position: np.ndarray,
        base_orientation: np.ndarray,
        base_velocity: np.ndarray,
        feet_positions: LegsAttr,
        current_contact: np.ndarray,
        reference_velocity: np.ndarray,
    ) -> HighLevelPlan:
        """Compute high-level plan for footholds and optionally contact schedule.
        
        Args:
            base_position: (3,) base COM position in world frame [x, y, z].
            base_orientation: (3,) base orientation in euler angles [roll, pitch, yaw].
            base_velocity: (3,) base linear velocity in world frame [vx, vy, vz].
            feet_positions: Current feet positions in world frame.
            current_contact: (4,) binary contact state [FL, FR, RL, RR].
            reference_velocity: (3,) desired base velocity in world frame.
            
        Returns:
            HighLevelPlan containing footholds, constraints, and optional schedule.
        """
        ...
    
    def update_terrain(
        self, 
        terrain_data: Optional[Dict] = None,
    ) -> None:
        """Update planner's internal terrain representation.
        
        Args:
            terrain_data: Terrain information (e.g., heightmap, surface list).
        """
        ...
    
    def reset(self) -> None:
        """Reset planner state (e.g., cached plans, internal counters)."""
        ...
