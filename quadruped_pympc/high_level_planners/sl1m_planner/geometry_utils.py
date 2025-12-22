"""Geometry utilities for foothold planning on stepping stones."""

import numpy as np
from typing import Tuple, List


def generate_cylinder_top_polygon(
    center: np.ndarray,
    radius: float,
    height: float,
    num_vertices: int = 8
) -> np.ndarray:
    """Generate polygon approximation of cylinder top surface.
    
    Creates a regular polygon inscribed in the cylinder's top circle.
    This approximates the cylindrical stepping stone top as a convex polygon
    for use in optimization-based foothold planning.
    
    Args:
        center: (x, y) center of cylinder base in world frame (meters).
        radius: Cylinder radius (meters).
        height: Height of cylinder top above ground (meters).
        num_vertices: Number of polygon vertices (default: 8 for octagon).
        
    Returns:
        vertices: (num_vertices, 3) array of polygon vertices in world frame.
                 Each row is [x, y, z].
    """
    angles = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
    vertices = np.zeros((num_vertices, 3))
    vertices[:, 0] = center[0] + radius * np.cos(angles)
    vertices[:, 1] = center[1] + radius * np.sin(angles)
    vertices[:, 2] = height
    return vertices


def point_in_circle(
    point: np.ndarray,
    center: np.ndarray,
    radius: float
) -> bool:
    """Check if a 2D point is inside a circle.
    
    Args:
        point: (2,) or (3,) point coordinates. If 3D, only XY are checked.
        center: (2,) or (3,) circle center. If 3D, only XY are checked.
        radius: Circle radius (meters).
        
    Returns:
        True if point is inside or on the circle boundary.
    """
    point_xy = point[:2]
    center_xy = center[:2]
    distance = np.linalg.norm(point_xy - center_xy)
    return distance <= radius


def point_in_box(
    point: np.ndarray,
    center: np.ndarray,
    half_size: Tuple[float, float]
) -> bool:
    """Check if a 2D point is inside an axis-aligned box.
    
    Args:
        point: (2,) or (3,) point coordinates. If 3D, only XY are checked.
        center: (2,) or (3,) box center. If 3D, only XY are checked.
        half_size: (dx, dy) half-size of box along x and y axes (meters).
        
    Returns:
        True if point is inside or on the box boundary.
    """
    point_xy = point[:2]
    center_xy = center[:2]
    dx, dy = half_size
    return (
        abs(point_xy[0] - center_xy[0]) <= dx and
        abs(point_xy[1] - center_xy[1]) <= dy
    )


def clamp_to_circle(
    point: np.ndarray,
    center: np.ndarray,
    radius: float
) -> np.ndarray:
    """Clamp a point to lie within a circle (XY plane).
    
    If the point is outside the circle, it is projected to the nearest
    point on the circle boundary.
    
    Args:
        point: (3,) point in world frame [x, y, z].
        center: (3,) circle center in world frame.
        radius: Circle radius (meters).
        
    Returns:
        clamped_point: (3,) point clamped to circle, preserving Z coordinate.
    """
    point_xy = point[:2]
    center_xy = center[:2]
    diff = point_xy - center_xy
    distance = np.linalg.norm(diff)
    
    if distance <= radius:
        # Already inside
        return point.copy()
    
    # Project to boundary
    direction = diff / distance
    clamped_xy = center_xy + direction * radius
    return np.array([clamped_xy[0], clamped_xy[1], point[2]])


def clamp_to_box(
    point: np.ndarray,
    center: np.ndarray,
    half_size: Tuple[float, float]
) -> np.ndarray:
    """Clamp a point to lie within an axis-aligned box (XY plane).
    
    If the point is outside the box, it is projected to the nearest
    point on the box boundary.
    
    Args:
        point: (3,) point in world frame [x, y, z].
        center: (3,) box center in world frame.
        half_size: (dx, dy) half-size of box along x and y axes (meters).
        
    Returns:
        clamped_point: (3,) point clamped to box, preserving Z coordinate.
    """
    point_xy = point[:2]
    center_xy = center[:2]
    dx, dy = half_size
    
    clamped_xy = np.array([
        np.clip(point_xy[0], center_xy[0] - dx, center_xy[0] + dx),
        np.clip(point_xy[1], center_xy[1] - dy, center_xy[1] + dy),
    ])
    
    return np.array([clamped_xy[0], clamped_xy[1], point[2]])


def build_plum_pile_grid(
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    x_step: float,
    y_step: float,
    radius: float,
    height: float
) -> List[dict]:
    """Build a regular grid of plum pile (stepping stone) surfaces.
    
    Creates a list of pile surface representations for use in foothold planning.
    
    Args:
        x_range: (x_min, x_max) range for pile grid in world frame (meters).
        y_range: (y_min, y_max) range for pile grid in world frame (meters).
        x_step: Spacing between pile centers along x-axis (meters).
        y_step: Spacing between pile centers along y-axis (meters).
        radius: Pile radius (meters).
        height: Height of pile top above ground (meters).
        
    Returns:
        piles: List of dicts, each containing:
               - 'center': (x, y) center of pile base
               - 'radius': pile radius
               - 'height': pile top height
               - 'vertices': polygon approximation of top (8-gon)
    """
    piles = []
    x_min, x_max = x_range
    y_min, y_max = y_range
    
    x_positions = np.arange(x_min, x_max + x_step/2, x_step)
    y_positions = np.arange(y_min, y_max + y_step/2, y_step)
    
    for x in x_positions:
        for y in y_positions:
            center = np.array([x, y])
            vertices = generate_cylinder_top_polygon(center, radius, height, num_vertices=8)
            
            piles.append({
                'center': center,
                'radius': radius,
                'height': height,
                'vertices': vertices,
            })
    
    return piles
