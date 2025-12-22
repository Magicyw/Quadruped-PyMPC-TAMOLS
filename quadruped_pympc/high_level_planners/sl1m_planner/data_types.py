"""Data types for high-level contact and foothold planning."""

from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple
import numpy as np


@dataclass
class FootholdConstraintRegion:
    """Defines a constraint region for a foothold (e.g., pile top boundary).
    
    Attributes:
        center: (x, y, z) center of the constraint region in world frame.
        radius: Radius for circular constraint (meters). None if using box.
        box_half_size: (dx, dy) half-size for box constraint (meters). None if using circle.
        region_type: 'circle' or 'box'.
    """
    center: np.ndarray  # shape (3,)
    radius: Optional[float] = None
    box_half_size: Optional[Tuple[float, float]] = None
    region_type: str = 'circle'  # 'circle' or 'box'
    
    def __post_init__(self):
        """Validate constraint region configuration."""
        if self.region_type == 'circle' and self.radius is None:
            raise ValueError("Circle region requires radius")
        if self.region_type == 'box' and self.box_half_size is None:
            raise ValueError("Box region requires box_half_size")


@dataclass
class HighLevelPlan:
    """High-level plan containing footholds and optional contact schedule.
    
    This dataclass supports Mode A (foothold-only planning) now, with
    placeholders for Mode B (full contact schedule planning) later.
    
    Attributes:
        footholds: Dict mapping leg names ('FL', 'FR', 'RL', 'RR') to 
                   foothold positions (x, y, z) in world frame.
        foothold_constraints: Dict mapping leg names to FootholdConstraintRegion.
                             Defines safe regions where foothold optimization/adaptation
                             must remain.
        contact_schedule: Optional future contact sequence. 
                         Format: List of arrays, one per leg. Each array is binary
                         (1=contact, 0=swing) over the planning horizon.
                         None for Mode A (existing gait generator is used).
        phase_schedule: Optional dict with phase information for Mode B.
                       None for Mode A.
        metadata: Dict for any additional planner-specific info (e.g., 
                 cost values, solver status, pile IDs chosen).
    """
    footholds: Dict[str, np.ndarray]  # {'FL': array([x,y,z]), ...}
    foothold_constraints: Dict[str, Optional[FootholdConstraintRegion]]
    contact_schedule: Optional[List[np.ndarray]] = None  # Mode B placeholder
    phase_schedule: Optional[Dict[str, np.ndarray]] = None  # Mode B placeholder
    metadata: Dict = field(default_factory=dict)
