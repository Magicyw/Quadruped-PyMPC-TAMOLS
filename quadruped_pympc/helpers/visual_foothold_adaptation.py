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

        Combines multiple cost terms inspired by TAMOLS:
        - Edge avoidance: penalizes high terrain gradients (edge risk)
        - Roughness: penalizes irregular terrain
        - Previous solution tracking: penalizes deviation from seed
        - Kinematic reachability: penalizes or rejects candidates outside leg reach
        - Forward progress: penalizes candidates that don't contribute to forward motion (anti-conservative)
        - Velocity alignment: encourages footholds aligned with velocity direction (GIA principle)
        - Step consistency: maintains consistent step patterns across legs (GIA principle)

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
        w_dev = self.tamols_params.get('weight_deviation', 1.0)
        w_kin = self.tamols_params.get('weight_kinematic', 10.0)
        w_forward = self.tamols_params.get('weight_forward_progress', 3.0)
        w_vel_align = self.tamols_params.get('weight_velocity_alignment', 2.0)
        w_step_consist = self.tamols_params.get('weight_step_consistency', 1.5)

        # 1. Kinematic reachability cost
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
        edge_cost = self._compute_edge_cost(candidate, heightmap) * w_edge

        # 3. Roughness cost (local height variance)
        roughness_cost = self._compute_roughness_cost(candidate, heightmap) * w_rough

        # 4. Previous solution tracking (deviation from seed)
        deviation = np.linalg.norm(candidate[:2] - seed[:2])
        deviation_cost = deviation * w_dev

        # 5. Forward progress cost (NEW - anti-conservative)
        # Penalize footholds that move backward relative to hip
        forward_progress_cost = self._compute_forward_progress_cost(candidate, hip_position) * w_forward

        # 6. Velocity alignment cost (NEW - GIA principle)
        # Encourage footholds aligned with robot's velocity direction
        velocity_alignment_cost = self._compute_velocity_alignment_cost(candidate, seed) * w_vel_align

        # 7. Step consistency cost (NEW - GIA principle)
        # Maintain consistent step lengths across legs for stable gait
        step_consistency_cost = self._compute_step_consistency_cost(
            candidate, hip_position, leg_name, all_footholds
        ) * w_step_consist

        # Total cost combines all terms
        total_cost = (
            kinematic_cost + edge_cost + roughness_cost + deviation_cost +
            forward_progress_cost + velocity_alignment_cost + step_consistency_cost
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

    def _compute_forward_progress_cost(self, candidate, hip_position):
        """Penalize footholds that don't contribute to forward motion.

        This prevents the robot from being overly conservative and standing still.
        Encourages footholds that are forward of the hip in the direction of motion.

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            hip_position: np.ndarray [x, y, z] hip position in world frame

        Returns:
            float: forward progress cost (lower is better)
        """
        # Vector from hip to candidate foothold
        hip_to_foot = candidate - hip_position

        # Get forward velocity direction (if available)
        if hasattr(self, 'forward_vel') and self.forward_vel is not None:
            vel_magnitude = np.linalg.norm(self.forward_vel[:2])
            if vel_magnitude > 0.01:  # Only apply if moving
                # Normalize velocity direction
                vel_direction = self.forward_vel[:2] / vel_magnitude
                
                # Project foothold displacement onto velocity direction
                forward_displacement = np.dot(hip_to_foot[:2], vel_direction)
                
                # Penalize backward steps (negative displacement)
                # Also penalize very small forward steps (overly conservative)
                min_forward_step = 0.05  # Minimum expected forward step [m]
                if forward_displacement < min_forward_step:
                    # Quadratic penalty for insufficient forward progress
                    cost = (min_forward_step - forward_displacement) ** 2
                else:
                    cost = 0.0
                
                return cost
        
        # Fallback: if no velocity info, assume forward is +X direction
        forward_displacement = hip_to_foot[0]
        min_forward_step = 0.03
        if forward_displacement < min_forward_step:
            cost = (min_forward_step - forward_displacement) ** 2
        else:
            cost = 0.0
        
        return cost

    def _compute_velocity_alignment_cost(self, candidate, seed):
        """Encourage footholds aligned with robot's velocity direction.

        This supports GIA (Gait Independent Adaptation) by selecting footholds
        that naturally align with the direction of motion, regardless of specific gait timing.

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            seed: np.ndarray [x, y, z] seed/reference foothold in world frame

        Returns:
            float: velocity alignment cost (lower is better)
        """
        if not hasattr(self, 'forward_vel') or self.forward_vel is None:
            return 0.0
        
        vel_magnitude = np.linalg.norm(self.forward_vel[:2])
        if vel_magnitude < 0.01:  # Robot nearly stationary
            return 0.0
        
        # Normalize velocity direction
        vel_direction = self.forward_vel[:2] / vel_magnitude
        
        # Displacement from seed to candidate
        displacement = candidate[:2] - seed[:2]
        displacement_magnitude = np.linalg.norm(displacement)
        
        if displacement_magnitude < 0.001:  # Candidate same as seed
            return 0.0
        
        # Normalize displacement
        displacement_direction = displacement / displacement_magnitude
        
        # Compute alignment: 1 = perfectly aligned, -1 = opposite direction
        alignment = np.dot(displacement_direction, vel_direction)
        
        # Cost is higher when misaligned (alignment < 1)
        # Use (1 - alignment) so perfect alignment gives 0 cost
        # Multiply by displacement magnitude to penalize larger misaligned steps more
        cost = (1.0 - alignment) * displacement_magnitude
        
        return cost

    def _compute_step_consistency_cost(self, candidate, hip_position, leg_name, all_footholds):
        """Maintain consistent step patterns across legs.

        This supports GIA by encouraging similar step lengths across all legs,
        which helps maintain stable periodic gaits independent of specific timing.

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            hip_position: np.ndarray [x, y, z] hip position in world frame
            leg_name: str name of current leg
            all_footholds: LegsAttr object with footholds for all legs

        Returns:
            float: step consistency cost (lower is better)
        """
        # Compute step length for candidate (hip to foothold distance in XY plane)
        hip_to_candidate = candidate[:2] - hip_position[:2]
        candidate_step_length = np.linalg.norm(hip_to_candidate)
        
        # Get step lengths of other legs (those that have been updated)
        other_step_lengths = []
        leg_names = ['FL', 'FR', 'RL', 'RR']
        
        for other_leg in leg_names:
            if other_leg != leg_name and all_footholds[other_leg] is not None:
                # We don't have hip positions for other legs here, so we'll use
                # the foothold positions as a proxy for step distance consistency
                other_foothold = all_footholds[other_leg]
                if other_foothold is not None:
                    # Distance between current candidate and other leg's foothold
                    distance_to_other = np.linalg.norm(candidate[:2] - other_foothold[:2])
                    other_step_lengths.append(distance_to_other)
        
        if len(other_step_lengths) == 0:
            # First leg being adapted, no consistency cost yet
            return 0.0
        
        # Compute variance of step lengths (including candidate)
        # Lower variance = more consistent = better for GIA
        all_step_lengths = other_step_lengths + [candidate_step_length]
        step_length_variance = np.var(all_step_lengths)
        
        return step_length_variance
