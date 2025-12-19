import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg

try:
    from virall.vfa.vfa import VFA
except ImportError:
    print("VFA not installed, not open source yet")


class VisualFootholdAdaptation:
    def __init__(self, legs_order, adaptation_strategy='height'):
        self.footholds_adaptation = LegsAttr(
            FL=np.array([0, 0, 0]), FR=np.array([0, 0, 0]), RL=np.array([0, 0, 0]), RR=np.array([0, 0, 0])
        )
        self.footholds_constraints = LegsAttr(FL=None, FR=None, RL=None, RR=None)
        self.initialized = False

        self.adaptation_strategy = adaptation_strategy

        if self.adaptation_strategy == 'vfa':
            self.vfa_evaluators = LegsAttr(FL=None, FR=None, RL=None, RR=None)
            for leg_id, leg_name in enumerate(legs_order):
                self.vfa_evaluators[leg_name] = VFA(leg=leg_name)

        # Load TAMOLS parameters if using TAMOLS strategy
        if self.adaptation_strategy == 'tamols':
            self.tamols_params = cfg.simulation_params.get('tamols_params', {})
            self.robot_name = cfg.robot

    def update_footholds_adaptation(self, update_footholds_adaptation):
        self.footholds_adaptation = update_footholds_adaptation
        self.initialized = True

    def reset(self):
        self.initialized = False

    def get_footholds_adapted(self, reference_footholds):
        # If the adaptation is not initialized, return the reference footholds
        if self.initialized == False:
            self.footholds_adaptation = reference_footholds
            return reference_footholds, self.footholds_constraints
        else:
            return self.footholds_adaptation, self.footholds_constraints

    def get_heightmap_coordinates_foothold_id(self, heightmaps, foothold_id, leg_name):
        r = round(foothold_id.item() / heightmaps[leg_name].n)
        c = round(foothold_id.item() % heightmaps[leg_name].n)

        if r >= heightmaps[leg_name].n:
            r = heightmaps[leg_name].n - 1
        if c >= heightmaps[leg_name].n:
            c = heightmaps[leg_name].n - 1

        return r, c

    def compute_adaptation(
        self,
        legs_order,
        reference_footholds,
        hip_positions,
        heightmaps,
        forward_vel,
        base_orientation,
        base_orientation_rate,
    ):
        for leg_id, leg_name in enumerate(legs_order):
            if heightmaps[leg_name].data is None:
                return False

        # Store forward velocity and base orientation for TAMOLS
        if self.adaptation_strategy == 'tamols':
            self.forward_vel = forward_vel
            self.base_orientation = base_orientation

        if self.adaptation_strategy == 'height':
            for leg_id, leg_name in enumerate(legs_order):
                height_adjustment = heightmaps[leg_name].get_height(reference_footholds[leg_name])
                if height_adjustment is not None:
                    reference_footholds[leg_name][2] = height_adjustment

        elif self.adaptation_strategy == 'vfa':
            gait_phases = 0.0  # for now
            for leg_id, leg_name in enumerate(legs_order):
                # Transform the heightmap in hip frame

                heightmap = heightmaps[leg_name].data
                hip_position = hip_positions[leg_name]

                heightmap_hip_frame = heightmap[:, :, 0, 2] - hip_position[2]

                convex_data, _, safe_map, info = self.vfa_evaluators[leg_name](
                    heightmap_data=heightmap_hip_frame,
                    base_linear_velocity=forward_vel,
                    base_orientation=base_orientation,
                    base_orientation_rate=base_orientation_rate,
                    gait_phase=gait_phases,
                    return_info=True,
                )

                best_foothold_id = convex_data[0][0]
                best_convex_area_vertices_id = [convex_data[1][0].x1, convex_data[1][0].y2]

                r, c = self.get_heightmap_coordinates_foothold_id(heightmaps, best_foothold_id, leg_name)

                reference_footholds[leg_name][0:2] = heightmap[r, c, 0, :][0:2]

                height_adjustment = heightmaps[leg_name].get_height(reference_footholds[leg_name])
                if height_adjustment is not None:
                    reference_footholds[leg_name][2] = height_adjustment

                r_vertex1, c_vertex1 = self.get_heightmap_coordinates_foothold_id(
                    heightmaps, np.array([best_convex_area_vertices_id[0]]), leg_name
                )
                r_vertex2, c_vertex2 = self.get_heightmap_coordinates_foothold_id(
                    heightmaps, np.array([best_convex_area_vertices_id[1]]), leg_name
                )
                vertex1_world_frame = heightmap[r_vertex1, c_vertex1, 0, :]
                vertex2_world_frame = heightmap[r_vertex2, c_vertex2, 0, :]

                self.footholds_constraints[leg_name] = [vertex1_world_frame, vertex2_world_frame]

                # print("Safe map: ", safe_map)

        elif self.adaptation_strategy == 'tamols':
            # TAMOLS-inspired foothold adaptation strategy
            # Performs local search around seed footholds using terrain-aware cost metrics
            for leg_id, leg_name in enumerate(legs_order):
                seed_foothold = reference_footholds[leg_name].copy()
                hip_position = hip_positions[leg_name]
                heightmap = heightmaps[leg_name]

                # Generate candidate footholds around the seed
                candidates = self._generate_candidates(seed_foothold)

                # Evaluate each candidate using TAMOLS-inspired cost metrics
                best_candidate = None
                best_score = float('inf')

                for candidate_xy in candidates:
                    # Query height from heightmap
                    candidate = np.array([candidate_xy[0], candidate_xy[1], 0.0])
                    height = heightmap.get_height(candidate)
                    if height is None:
                        continue  # Skip if heightmap query fails

                    candidate[2] = height

                    # Compute TAMOLS-inspired cost
                    score = self._compute_tamols_score(
                        candidate, seed_foothold, hip_position, heightmap, leg_name, reference_footholds
                    )

                    if score < best_score:
                        best_score = score
                        best_candidate = candidate

                # Use best candidate if found, otherwise keep seed
                if best_candidate is not None:
                    reference_footholds[leg_name] = best_candidate

                    # Create foothold constraints (simple box around chosen foothold)
                    dx = self.tamols_params.get('constraint_box_dx', 0.05)
                    dy = self.tamols_params.get('constraint_box_dy', 0.05)
                    vertex1 = best_candidate.copy()
                    vertex1[0] -= dx
                    vertex1[1] -= dy
                    vertex2 = best_candidate.copy()
                    vertex2[0] += dx
                    vertex2[1] += dy
                    self.footholds_constraints[leg_name] = [vertex1, vertex2]
                else:
                    # Fallback to height adjustment only
                    height_adjustment = heightmap.get_height(seed_foothold)
                    if height_adjustment is not None:
                        reference_footholds[leg_name][2] = height_adjustment

        self.update_footholds_adaptation(reference_footholds)

        return True

    def _generate_candidates(self, seed_foothold):
        """Generate candidate footholds in a grid around the seed position.

        Args:
            seed_foothold: np.ndarray [x, y, z] seed foothold position in world frame

        Returns:
            List of candidate [x, y] positions
        """
        radius = self.tamols_params.get('search_radius', 0.15)
        resolution = self.tamols_params.get('search_resolution', 0.03)

        candidates = []
        # Create grid around seed
        steps = int(radius / resolution)
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                dx = i * resolution
                dy = j * resolution
                # Only include candidates within circular radius
                if dx**2 + dy**2 <= radius**2:
                    candidates.append([seed_foothold[0] + dx, seed_foothold[1] + dy])

        return candidates

    def _compute_tamols_score(self, candidate, seed, hip_position, heightmap, leg_name, all_footholds):
        """Compute TAMOLS-inspired cost for a candidate foothold.

        Combines multiple cost terms inspired by TAMOLS reference (ianpedroza/tamols-rl):
        - Edge avoidance: penalizes high terrain gradients (from tamols/costs.py:add_edge_avoidance_cost)
        - Roughness: penalizes irregular terrain  
        - Previous solution tracking: penalizes deviation from seed (from tamols/costs.py:add_previous_solution_cost)
        - Kinematic reachability: enforces distance bounds (from tamols/constraints.py:add_kinematic_constraints)
        - Nominal kinematics: maintains desired hip height and leg configuration (from tamols/costs.py:add_nominal_kinematic_cost)
        - Reference tracking: tracks desired velocity direction (from tamols/costs.py:add_tracking_cost)

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            seed: np.ndarray [x, y, z] seed/reference foothold in world frame
            hip_position: np.ndarray [x, y, z] hip position in world frame
            heightmap: HeightMap object for querying terrain
            leg_name: str name of the leg (FL, FR, RL, RR)
            all_footholds: LegsAttr object with all leg footholds

        Returns:
            float: total cost (lower is better)
        """
        # Load cost weights
        w_edge = self.tamols_params.get('weight_edge_avoidance', 5.0)
        w_rough = self.tamols_params.get('weight_roughness', 2.0)
        w_prev_sol = self.tamols_params.get('weight_previous_solution', 0.01)
        w_kin = self.tamols_params.get('weight_kinematic', 10.0)
        w_nominal_kin = self.tamols_params.get('weight_nominal_kinematic', 20.0)
        w_tracking = self.tamols_params.get('weight_reference_tracking', 2.0)

        # 1. Kinematic reachability cost
        # From TAMOLS: constraints.py:add_kinematic_constraints (lines 130-154)
        # Enforces l_min <= ||hip - foot|| <= l_max
        hip_to_foot = candidate - hip_position
        distance = np.linalg.norm(hip_to_foot)
        l_min = self.tamols_params.get('l_min', {}).get(self.robot_name, 0.15)
        l_max = self.tamols_params.get('l_max', {}).get(self.robot_name, 0.45)

        # Penalize candidates outside kinematic bounds
        if distance < l_min:
            kinematic_cost = w_kin * (l_min - distance) ** 2
        elif distance > l_max:
            kinematic_cost = w_kin * (distance - l_max) ** 2
        else:
            kinematic_cost = 0.0

        # 2. Edge avoidance cost (gradient magnitude)
        # From TAMOLS: costs.py:add_edge_avoidance_cost (lines 164-185)
        # Uses gradient magnitudes from heightmap
        edge_cost = self._compute_edge_cost(candidate, heightmap) * w_edge

        # 3. Roughness cost (local height variance)
        roughness_cost = self._compute_roughness_cost(candidate, heightmap) * w_rough

        # 4. Previous solution tracking
        # From TAMOLS: costs.py:add_previous_solution_cost (lines 187-215)
        # Penalizes deviation from previous/seed footholds
        deviation_squared = np.sum((candidate - seed) ** 2)
        previous_solution_cost = w_prev_sol * deviation_squared

        # 5. Nominal kinematic cost
        # From TAMOLS: costs.py:add_nominal_kinematic_cost (lines 101-130)
        # Encourages footholds that maintain desired hip height (h_des)
        nominal_kinematic_cost = self._compute_nominal_kinematic_cost(
            candidate, hip_position
        ) * w_nominal_kin

        # 6. Reference tracking cost
        # From TAMOLS: costs.py:add_tracking_cost (lines 13-34)
        # Tracks reference velocity direction (only X direction in TAMOLS)
        reference_tracking_cost = self._compute_reference_tracking_cost(
            candidate, seed
        ) * w_tracking

        # Total cost combines all terms
        total_cost = (
            kinematic_cost + edge_cost + roughness_cost + previous_solution_cost +
            nominal_kinematic_cost + reference_tracking_cost
        )

        return total_cost

    def _compute_edge_cost(self, candidate, heightmap):
        """Estimate local terrain gradient magnitude (edge/slope risk).

        Uses finite differences over nearby heightmap queries to approximate gradient.

        Args:
            candidate: np.ndarray [x, y, z] position in world frame
            heightmap: HeightMap object

        Returns:
            float: gradient magnitude cost
        """
        delta = 0.02  # Small offset for finite difference

        # Sample heights in a cross pattern (±x, ±y)
        heights = []
        offsets = [np.array([delta, 0, 0]), np.array([-delta, 0, 0]), np.array([0, delta, 0]), np.array([0, -delta, 0])]

        for offset in offsets:
            query_pos = candidate + offset
            h = heightmap.get_height(query_pos)
            if h is not None:
                heights.append(h)

        if len(heights) < 4:
            # Not enough data, return high cost
            return 1.0

        # Estimate gradient using finite differences
        grad_x = abs(heights[0] - heights[1]) / (2 * delta)
        grad_y = abs(heights[2] - heights[3]) / (2 * delta)
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)

        return gradient_magnitude

    def _compute_roughness_cost(self, candidate, heightmap):
        """Estimate local terrain roughness (height variance).

        Samples multiple nearby points and computes height variance.

        Args:
            candidate: np.ndarray [x, y, z] position in world frame
            heightmap: HeightMap object

        Returns:
            float: roughness (variance) cost
        """
        delta = 0.03

        # Sample heights in a small patch
        heights = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                query_pos = candidate.copy()
                query_pos[0] += i * delta
                query_pos[1] += j * delta
                h = heightmap.get_height(query_pos)
                if h is not None:
                    heights.append(h)

        if len(heights) < 5:
            # Not enough data
            return 0.5

        # Compute variance of heights
        roughness = np.var(heights)

        return roughness

    def _compute_nominal_kinematic_cost(self, candidate, hip_position):
        """Cost for nominal kinematics - maintains desired hip height.

        From TAMOLS costs.py:add_nominal_kinematic_cost (lines 101-130).
        Encourages footholds that maintain the robot at its desired hip height
        with legs in nominal positions. This is key to GIA as it maintains
        consistent kinematic configuration independent of gait phase.

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            hip_position: np.ndarray [x, y, z] hip position in world frame

        Returns:
            float: nominal kinematic cost (lower is better)
        """
        # Desired leg vector in body frame: [0, 0, -h_des]
        # In TAMOLS, h_des is the desired hip height (default 0.25m for go2)
        h_des = self.tamols_params.get('h_des', 0.25)
        l_des = np.array([0.0, 0.0, -h_des])
        
        # In TAMOLS: cost = ||base + R_B * hip_offset - l_des - p_i||^2
        # Simplified version (no rotation): base at desired position relative to foot
        # base_minus_leg = hip_position - l_des
        # cost = ||(base_minus_leg) - candidate||^2
        
        # Desired base position if foot is at candidate
        desired_base_from_foot = candidate - l_des
        
        # Actual hip position
        # Cost penalizes deviation from nominal configuration
        diff = hip_position - desired_base_from_foot
        cost = np.dot(diff, diff)
        
        return cost

    def _compute_reference_tracking_cost(self, candidate, seed):
        """Cost to track reference velocity direction.

        From TAMOLS costs.py:add_tracking_cost (lines 13-34).
        In TAMOLS, this tracks desired velocity (only X direction).
        Adapted for foothold selection: encourages footholds that support
        forward motion in the reference direction, preventing conservative behavior.

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            seed: np.ndarray [x, y, z] seed/reference foothold in world frame

        Returns:
            float: reference tracking cost (lower is better)
        """
        if not hasattr(self, 'forward_vel') or self.forward_vel is None:
            # No velocity info, penalize backward displacement relative to seed
            displacement = candidate[0] - seed[0]  # X direction only, like TAMOLS
            if displacement < 0:
                # Backward step, high cost
                return displacement ** 2
            return 0.0
        
        # Get reference velocity
        ref_vel = self.forward_vel[:2]  # XY plane
        vel_magnitude = np.linalg.norm(ref_vel)
        
        if vel_magnitude < 0.01:
            # Nearly stationary, no tracking cost
            return 0.0
        
        # In TAMOLS: cost = (vel - ref_vel)^2, but we don't have foothold velocity
        # Instead, we check if foothold supports the desired motion direction
        # Foothold displacement should align with velocity
        displacement = candidate[:2] - seed[:2]
        
        # Project displacement onto velocity direction (like TAMOLS tracking in X only)
        # We focus on X direction for consistency with TAMOLS
        vel_x = ref_vel[0]
        displacement_x = displacement[0]
        
        # Cost: penalize if foothold doesn't support forward velocity
        # Similar to TAMOLS: (actual - desired)^2
        # If vel_x > 0 (forward), we want displacement_x > 0
        # Cost = (displacement_x - desired_displacement)^2
        
        # Simple version: penalize if displacement opposes velocity
        if vel_x > 0 and displacement_x < 0:
            # Moving forward but foothold goes backward
            cost = displacement_x ** 2
        elif vel_x < 0 and displacement_x > 0:
            # Moving backward but foothold goes forward
            cost = displacement_x ** 2
        else:
            # Displacement aligns with velocity, minimal cost
            cost = 0.0
        
        return cost
