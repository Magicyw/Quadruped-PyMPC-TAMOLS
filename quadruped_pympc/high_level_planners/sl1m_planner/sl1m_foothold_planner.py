"""sl1m-based foothold planner implementation for plum piles terrain.

This module implements a high-level foothold planner that:
1. Uses sl1m optimization if available (Mode A: foothold planning only)
2. Falls back to simple heuristics if sl1m is not installed
3. Plans a rolling horizon of 4 steps for crawl gait
4. Provides placeholders for Mode B (contact schedule generation)
"""

import copy
import warnings
from typing import Dict, List, Optional, Tuple
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from .data_types import HighLevelPlan, FootholdConstraintRegion
from .geometry_utils import (
    build_plum_pile_grid,
    point_in_circle,
    clamp_to_circle,
)

# Try to import sl1m - if not available, fall back to heuristic
SL1M_AVAILABLE = False
try:
    # sl1m may provide: Problem, generic_solver.solve_L1_combinatorial
    # This is a placeholder - actual import depends on sl1m package structure
    import sl1m
    from sl1m import Problem
    SL1M_AVAILABLE = True
except ImportError:
    warnings.warn(
        "sl1m package not found. Sl1mFootholdPlanner will use heuristic fallback. "
        "To use optimization-based planning, install sl1m: pip install git+https://github.com/loco-3d/sl1m.git",
        ImportWarning
    )


class Sl1mFootholdPlanner:
    """High-level foothold planner for plum piles terrain using sl1m or heuristics.
    
    Implements Mode A: Provides footholds for currently swinging leg based on
    a 4-step planning horizon. Contact schedule remains handled by existing
    gait generator (PGG in WBInterface).
    
    Mode B placeholders are provided but not implemented (contact schedule output).
    """
    
    def __init__(
        self,
        pile_config: Dict,
        planning_horizon: int = 4,
        robot_config: Optional[Dict] = None,
        use_sl1m: bool = True,
    ):
        """Initialize the sl1m foothold planner.
        
        Args:
            pile_config: Dict with plum pile terrain configuration:
                         - 'x_range': (x_min, x_max) in meters
                         - 'y_range': (y_min, y_max) in meters
                         - 'x_step': spacing between piles in x (meters)
                         - 'y_step': spacing between piles in y (meters)
                         - 'radius': pile radius (meters)
                         - 'height': pile top height above ground (meters)
                         - 'constraint_radius': allowed foothold radius on pile (meters)
            planning_horizon: Number of future steps to plan (default: 4).
            robot_config: Optional dict with robot parameters (hip positions, reach limits).
            use_sl1m: If True and sl1m is available, use optimization. Otherwise use heuristic.
        """
        self.pile_config = pile_config
        self.planning_horizon = planning_horizon
        self.robot_config = robot_config or {}
        self.use_sl1m = use_sl1m and SL1M_AVAILABLE
        
        # Build pile grid representation
        self.piles = build_plum_pile_grid(
            x_range=pile_config['x_range'],
            y_range=pile_config['y_range'],
            x_step=pile_config['x_step'],
            y_step=pile_config['y_step'],
            radius=pile_config['radius'],
            height=pile_config['height'],
        )
        
        # Constraint radius for foothold (typically smaller than pile radius)
        self.constraint_radius = pile_config.get('constraint_radius', pile_config['radius'] * 0.75)
        
        # Cache for planned footholds (rolling plan)
        # Structure: {'FL': [foothold_step1, foothold_step2, ...], ...}
        self.cached_plan: Dict[str, List[np.ndarray]] = {
            'FL': [], 'FR': [], 'RL': [], 'RR': []
        }
        
        # Track which leg is currently swinging (for Mode A)
        self.current_swing_leg: Optional[str] = None
        
        # Crawl gait leg sequence (one leg at a time, typical order)
        # Back-diagonal crawl: FL -> RR -> FR -> RL
        self.crawl_sequence = ['FL', 'RR', 'FR', 'RL']
        self.sequence_index = 0
        
        if not self.use_sl1m:
            warnings.warn(
                "Sl1mFootholdPlanner initialized in heuristic mode (sl1m not available or disabled).",
                UserWarning
            )
    
    def plan(
        self,
        base_position: np.ndarray,
        base_orientation: np.ndarray,
        base_velocity: np.ndarray,
        feet_positions: LegsAttr,
        current_contact: np.ndarray,
        reference_velocity: np.ndarray,
    ) -> HighLevelPlan:
        """Compute high-level foothold plan for the next swing leg.
        
        Returns the next foothold for the currently swinging leg (Mode A).
        Internally maintains a 4-step rolling plan.
        
        Args:
            base_position: (3,) base COM position [x, y, z].
            base_orientation: (3,) euler angles [roll, pitch, yaw].
            base_velocity: (3,) base linear velocity [vx, vy, vz].
            feet_positions: Current feet positions in world frame.
            current_contact: (4,) binary contact [FL, FR, RL, RR].
            reference_velocity: (3,) desired base velocity.
            
        Returns:
            HighLevelPlan with footholds and constraint regions.
        """
        # Detect swing leg from contact state
        swing_legs = [leg for i, leg in enumerate(['FL', 'FR', 'RL', 'RR']) 
                      if current_contact[i] == 0]
        
        if len(swing_legs) > 0:
            # In crawl gait, typically one leg swings at a time
            self.current_swing_leg = swing_legs[0]
        
        # If no cached plan or cache is stale, regenerate
        if not self._is_plan_valid():
            self._generate_rolling_plan(
                base_position, base_velocity, reference_velocity, feet_positions
            )
        
        # Extract next foothold for each leg from cached plan
        footholds = {}
        foothold_constraints = {}
        
        for leg in ['FL', 'FR', 'RL', 'RR']:
            if len(self.cached_plan[leg]) > 0:
                # Use first cached foothold
                footholds[leg] = self.cached_plan[leg][0].copy()
            else:
                # Fallback to current foot position
                footholds[leg] = np.array(feet_positions[leg])
            
            # Create constraint region around chosen foothold
            foothold_constraints[leg] = FootholdConstraintRegion(
                center=footholds[leg].copy(),
                radius=self.constraint_radius,
                region_type='circle'
            )
        
        # Mode A: no contact schedule (None means use existing gait generator)
        # Mode B placeholders
        contact_schedule = None
        phase_schedule = None
        
        metadata = {
            'planner_type': 'sl1m' if self.use_sl1m else 'heuristic',
            'planning_horizon': self.planning_horizon,
            'current_swing_leg': self.current_swing_leg,
        }
        
        return HighLevelPlan(
            footholds=footholds,
            foothold_constraints=foothold_constraints,
            contact_schedule=contact_schedule,
            phase_schedule=phase_schedule,
            metadata=metadata,
        )
    
    def _is_plan_valid(self) -> bool:
        """Check if cached plan is still valid (simple implementation)."""
        # For now, always regenerate. More sophisticated: check time since last plan,
        # or whether base has moved significantly.
        return False
    
    def _generate_rolling_plan(
        self,
        base_position: np.ndarray,
        base_velocity: np.ndarray,
        reference_velocity: np.ndarray,
        feet_positions: LegsAttr,
    ) -> None:
        """Generate a rolling plan of footholds for the next planning_horizon steps.
        
        Uses sl1m optimization if available, otherwise uses heuristic.
        """
        if self.use_sl1m:
            self._plan_with_sl1m(base_position, base_velocity, reference_velocity, feet_positions)
        else:
            self._plan_with_heuristic(base_position, base_velocity, reference_velocity, feet_positions)
    
    def _plan_with_sl1m(
        self,
        base_position: np.ndarray,
        base_velocity: np.ndarray,
        reference_velocity: np.ndarray,
        feet_positions: LegsAttr,
    ) -> None:
        """Plan footholds using sl1m optimization.
        
        NOTE: This is a placeholder for full sl1m integration. The sl1m API
        requires specific input formats and solver configuration that are
        pending integration. Currently falls back to heuristic planning.
        
        When fully integrated, this method will:
        1. Build sl1m Problem with surfaces (pile polygons as convex patches)
        2. Set up kinematic constraints (reachability from hips)
        3. Define objective (track reference velocity, minimize effort)
        4. Call sl1m solver (e.g., generic_solver.solve_L1_combinatorial)
        5. Extract foothold sequence from solution
        
        The fallback to heuristic ensures the planner is functional even
        without sl1m installed or while API integration is in progress.
        """
        # TODO: Implement full sl1m integration when sl1m API is finalized
        # sl1m integration requires:
        # - Surface definitions (convex polygons from pile tops)
        # - Initial foot positions
        # - Kinematic constraints (reachability from hips)
        # - Objective (track reference velocity, minimize effort)
        
        # Fall back to heuristic for now
        self._plan_with_heuristic(base_position, base_velocity, reference_velocity, feet_positions)
    
    def _plan_with_heuristic(
        self,
        base_position: np.ndarray,
        base_velocity: np.ndarray,
        reference_velocity: np.ndarray,
        feet_positions: LegsAttr,
    ) -> None:
        """Plan footholds using simple heuristic (nearest reachable pile ahead).
        
        For each leg, selects piles that are:
        1. Ahead of current base position (in direction of travel)
        2. Within kinematic reach
        3. Nearest to nominal foothold location
        """
        # Estimate future base positions assuming constant velocity
        dt_step = 0.5  # Rough step duration for crawl gait
        
        for step in range(self.planning_horizon):
            # Predict base position for this step
            future_base_pos = base_position + reference_velocity * dt_step * (step + 1)
            
            for leg in ['FL', 'FR', 'RL', 'RR']:
                # Compute nominal foothold (simple geometric heuristic)
                # Relative to base, legs are roughly at corners
                leg_offset = self._get_nominal_leg_offset(leg)
                nominal_foothold = future_base_pos + leg_offset
                
                # Find nearest reachable pile
                best_pile = self._find_nearest_reachable_pile(nominal_foothold)
                
                if best_pile is not None:
                    # Use pile center at pile height
                    foothold = np.array([
                        best_pile['center'][0],
                        best_pile['center'][1],
                        best_pile['height']
                    ])
                else:
                    # No pile found, use nominal (may not be safe)
                    foothold = nominal_foothold.copy()
                
                # Add to cache
                if len(self.cached_plan[leg]) <= step:
                    self.cached_plan[leg].append(foothold)
                else:
                    self.cached_plan[leg][step] = foothold
    
    def _get_nominal_leg_offset(self, leg: str) -> np.ndarray:
        """Get nominal offset from base to leg (simple approximation).
        
        Args:
            leg: Leg name ('FL', 'FR', 'RL', 'RR').
            
        Returns:
            offset: (3,) offset vector [dx, dy, dz].
        """
        # Simple rectangular hip configuration
        # Typical aliengo: ~0.2m forward/back, ~0.1m lateral
        hip_forward = 0.2
        hip_lateral = 0.15
        
        offsets = {
            'FL': np.array([hip_forward, hip_lateral, 0.0]),
            'FR': np.array([hip_forward, -hip_lateral, 0.0]),
            'RL': np.array([-hip_forward, hip_lateral, 0.0]),
            'RR': np.array([-hip_forward, -hip_lateral, 0.0]),
        }
        return offsets.get(leg, np.zeros(3))
    
    def _find_nearest_reachable_pile(
        self,
        nominal_foothold: np.ndarray,
        max_reach: float = 0.6
    ) -> Optional[Dict]:
        """Find the nearest pile within reach of nominal foothold.
        
        Args:
            nominal_foothold: (3,) desired foothold position.
            max_reach: Maximum distance to search for piles (meters).
            
        Returns:
            pile: Dict describing the nearest reachable pile, or None.
        """
        best_pile = None
        best_distance = float('inf')
        
        for pile in self.piles:
            pile_center_3d = np.array([pile['center'][0], pile['center'][1], pile['height']])
            distance = np.linalg.norm(pile_center_3d - nominal_foothold)
            
            if distance < best_distance and distance <= max_reach:
                best_distance = distance
                best_pile = pile
        
        return best_pile
    
    def update_terrain(self, terrain_data: Optional[Dict] = None) -> None:
        """Update internal terrain representation.
        
        Args:
            terrain_data: Optional terrain update (e.g., new pile locations).
        """
        # For static plum pile grid, no update needed
        # Future: could handle dynamic obstacles or terrain changes
        pass
    
    def reset(self) -> None:
        """Reset planner state."""
        self.cached_plan = {'FL': [], 'FR': [], 'RL': [], 'RR': []}
        self.current_swing_leg = None
        self.sequence_index = 0
