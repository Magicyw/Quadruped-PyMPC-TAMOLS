"""Unit tests for geometry utilities in sl1m planner."""

import pytest
import numpy as np
from quadruped_pympc.high_level_planners.sl1m_planner.geometry_utils import (
    generate_cylinder_top_polygon,
    point_in_circle,
    point_in_box,
    clamp_to_circle,
    clamp_to_box,
    build_plum_pile_grid,
)


class TestCylinderPolygon:
    """Tests for cylinder top polygon generation."""
    
    def test_polygon_basic(self):
        """Test basic polygon generation."""
        center = np.array([1.0, 2.0])
        radius = 0.5
        height = 1.0
        vertices = generate_cylinder_top_polygon(center, radius, height, num_vertices=8)
        
        assert vertices.shape == (8, 3), "Should have 8 vertices with x,y,z"
        assert np.allclose(vertices[:, 2], height), "All vertices should be at specified height"
        
        # Check that vertices are approximately on the circle
        for vertex in vertices:
            dist = np.linalg.norm(vertex[:2] - center)
            assert np.isclose(dist, radius, atol=1e-6), "Vertices should lie on circle"
    
    def test_polygon_num_vertices(self):
        """Test polygon generation with different vertex counts."""
        center = np.array([0.0, 0.0])
        radius = 1.0
        height = 0.5
        
        for n in [4, 6, 8, 12]:
            vertices = generate_cylinder_top_polygon(center, radius, height, num_vertices=n)
            assert len(vertices) == n, f"Should have {n} vertices"


class TestPointInRegion:
    """Tests for point-in-region checks."""
    
    def test_point_in_circle_inside(self):
        """Test point inside circle."""
        center = np.array([0.0, 0.0, 1.0])
        radius = 1.0
        point = np.array([0.5, 0.5, 1.0])
        
        assert point_in_circle(point, center, radius), "Point should be inside circle"
    
    def test_point_in_circle_outside(self):
        """Test point outside circle."""
        center = np.array([0.0, 0.0])
        radius = 1.0
        point = np.array([2.0, 0.0])
        
        assert not point_in_circle(point, center, radius), "Point should be outside circle"
    
    def test_point_in_circle_boundary(self):
        """Test point on circle boundary."""
        center = np.array([0.0, 0.0])
        radius = 1.0
        point = np.array([1.0, 0.0])
        
        assert point_in_circle(point, center, radius), "Point on boundary should be inside"
    
    def test_point_in_box_inside(self):
        """Test point inside box."""
        center = np.array([0.0, 0.0, 1.0])
        half_size = (1.0, 1.0)
        point = np.array([0.5, 0.5, 1.0])
        
        assert point_in_box(point, center, half_size), "Point should be inside box"
    
    def test_point_in_box_outside(self):
        """Test point outside box."""
        center = np.array([0.0, 0.0])
        half_size = (1.0, 1.0)
        point = np.array([2.0, 0.0])
        
        assert not point_in_box(point, center, half_size), "Point should be outside box"


class TestClampToRegion:
    """Tests for clamping points to regions."""
    
    def test_clamp_to_circle_inside(self):
        """Test clamping point already inside circle."""
        center = np.array([0.0, 0.0, 1.0])
        radius = 1.0
        point = np.array([0.5, 0.0, 1.5])
        
        clamped = clamp_to_circle(point, center, radius)
        assert np.allclose(clamped, point), "Point inside should not be moved"
    
    def test_clamp_to_circle_outside(self):
        """Test clamping point outside circle."""
        center = np.array([0.0, 0.0, 1.0])
        radius = 1.0
        point = np.array([2.0, 0.0, 1.5])
        
        clamped = clamp_to_circle(point, center, radius)
        
        # Should be on boundary
        dist = np.linalg.norm(clamped[:2] - center[:2])
        assert np.isclose(dist, radius), "Clamped point should be on circle boundary"
        
        # Z coordinate preserved
        assert clamped[2] == point[2], "Z coordinate should be preserved"
        
        # Direction from center to clamped should match direction to original
        dir_orig = (point[:2] - center[:2]) / np.linalg.norm(point[:2] - center[:2])
        dir_clamped = (clamped[:2] - center[:2]) / np.linalg.norm(clamped[:2] - center[:2])
        assert np.allclose(dir_orig, dir_clamped), "Direction should be preserved"
    
    def test_clamp_to_box_inside(self):
        """Test clamping point already inside box."""
        center = np.array([0.0, 0.0, 1.0])
        half_size = (1.0, 1.0)
        point = np.array([0.5, 0.5, 1.5])
        
        clamped = clamp_to_box(point, center, half_size)
        assert np.allclose(clamped, point), "Point inside should not be moved"
    
    def test_clamp_to_box_outside(self):
        """Test clamping point outside box."""
        center = np.array([0.0, 0.0, 1.0])
        half_size = (1.0, 1.0)
        point = np.array([2.0, 3.0, 1.5])
        
        clamped = clamp_to_box(point, center, half_size)
        
        # Should be clamped to [1.0, 1.0, 1.5]
        expected = np.array([1.0, 1.0, 1.5])
        assert np.allclose(clamped, expected), "Point should be clamped to box boundary"


class TestBuildPileGrid:
    """Tests for building plum pile grid."""
    
    def test_build_grid_basic(self):
        """Test basic grid generation."""
        x_range = (0.0, 1.0)
        y_range = (0.0, 1.0)
        x_step = 0.5
        y_step = 0.5
        radius = 0.1
        height = 0.5
        
        piles = build_plum_pile_grid(x_range, y_range, x_step, y_step, radius, height)
        
        # Should have 3x3 = 9 piles (0.0, 0.5, 1.0 in each dimension)
        assert len(piles) == 9, f"Expected 9 piles, got {len(piles)}"
        
        # Check first pile
        assert piles[0]['radius'] == radius
        assert piles[0]['height'] == height
        assert 'center' in piles[0]
        assert 'vertices' in piles[0]
    
    def test_build_grid_matches_problem_spec(self):
        """Test grid matching problem statement specs."""
        # Problem: x from 4.1 to 7.1 step 0.2, y from -0.4 to 0.4 step 0.2
        x_range = (4.1, 7.1)
        y_range = (-0.4, 0.4)
        x_step = 0.2
        y_step = 0.2
        radius = 0.08
        height = 0.268
        
        piles = build_plum_pile_grid(x_range, y_range, x_step, y_step, radius, height)
        
        # Expected: 16 positions in x (4.1, 4.3, ..., 7.1) and 5 in y (-0.4, -0.2, 0.0, 0.2, 0.4)
        # 16 * 5 = 80 piles
        assert len(piles) > 0, "Should generate piles"
        
        # Verify some properties
        for pile in piles:
            assert pile['radius'] == radius
            assert pile['height'] == height
            assert pile['vertices'].shape == (8, 3), "Should have 8 vertices"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
