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
            for _leg_id, leg_name in enumerate(legs_order):
                self.vfa_evaluators[leg_name] = VFA(leg=leg_name)

        # Load TAMOLS parameters if using TAMOLS strategy
        if self.adaptation_strategy == 'tamols':
            self.tamols_params = cfg.simulation_params.get('tamols_params', {})
            self.robot_name = cfg.robot
            # Initialize previous optimal footholds for continuity objective (Eq. 8)
            self.previous_optimal_footholds = LegsAttr(
                FL=None, FR=None, RL=None, RR=None
            )

    def update_footholds_adaptation(self, update_footholds_adaptation):
        self.footholds_adaptation = update_footholds_adaptation
        self.initialized = True

    def reset(self):
        self.initialized = False

    def get_footholds_adapted(self, reference_footholds):
        # If the adaptation is not initialized, return the reference footholds
        if not self.initialized:
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
        feet_positions=None,
        contact_state=None,
    ):
        """Compute foothold adaptation using the selected strategy.
        
        Args:
            legs_order: List of leg names in order
            reference_footholds: LegsAttr with reference foothold positions
            hip_positions: LegsAttr with hip positions (thigh positions from paper)
            heightmaps: LegsAttr with HeightMap objects per leg
            forward_vel: Base forward velocity vector
            base_orientation: Base orientation as euler angles
            base_orientation_rate: Base angular velocity
            feet_positions: LegsAttr with current measured foot positions (for TAMOLS objectives 4,5 and constraints)
            contact_state: Array[4] indicating current contact state (1=stance, 0=swing) for TAMOLS
        """
        for _leg_id, leg_name in enumerate(legs_order):
            if heightmaps[leg_name].data is None:
                return False

        if self.adaptation_strategy == 'height':
            for _leg_id, leg_name in enumerate(legs_order):
                height_adjustment = heightmaps[leg_name].get_height(reference_footholds[leg_name])
                if height_adjustment is not None:
                    reference_footholds[leg_name][2] = height_adjustment

        elif self.adaptation_strategy == 'vfa':
            gait_phases = 0.0  # for now
            for _leg_id, leg_name in enumerate(legs_order):
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
            # Paper-based foothold optimization: "Perceptive Locomotion in Rough Terrain"
            # Store velocity for objectives
            self.forward_vel = forward_vel
            
            # Initialize feet_positions if not provided (fallback)
            if feet_positions is None:
                feet_positions = LegsAttr(
                    FL=hip_positions.FL.copy(), 
                    FR=hip_positions.FR.copy(),
                    RL=hip_positions.RL.copy(), 
                    RR=hip_positions.RR.copy()
                )
            
            # Initialize contact_state if not provided (assume all legs can be optimized)
            if contact_state is None:
                contact_state = np.array([0, 0, 0, 0])  # All swing by default

            for leg_id, leg_name in enumerate(legs_order):
                seed_foothold = reference_footholds[leg_name].copy()
                hip_position = hip_positions[leg_name]
                heightmap = heightmaps[leg_name]

                # Generate candidate footholds around the seed (paper: batch-search over grid cells)
                candidates = self._generate_candidates(seed_foothold, heightmap)

                # Evaluate each candidate using paper's objectives and constraints
                best_candidate = None
                best_score = float('inf')

                for candidate_xy in candidates:
                    # Query height from heightmap
                    candidate = np.array([candidate_xy[0], candidate_xy[1], 0.0])
                    height = heightmap.get_height(candidate)
                    if height is None:
                        continue  # Skip if heightmap query fails

                    candidate[2] = height
                    
                    # Apply hard constraints (paper Eq. 10, 11)
                    if not self._check_constraints(
                        candidate, seed_foothold, hip_position, heightmap,
                        feet_positions, leg_name, legs_order, contact_state[leg_id]
                    ):
                        continue  # Skip candidates that violate constraints

                    # Compute weighted objective score (paper Eq. 2-9)
                    score = self._compute_paper_objectives(
                        candidate, seed_foothold, hip_position, heightmap,
                        feet_positions, leg_name, legs_order, contact_state[leg_id]
                    )

                    if score < best_score:
                        best_score = score
                        best_candidate = candidate

                # Use best candidate if found, otherwise keep seed
                if best_candidate is not None:
                    reference_footholds[leg_name] = best_candidate
                    # Store for continuity objective (Eq. 8)
                    self.previous_optimal_footholds[leg_name] = best_candidate.copy()

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

    def _generate_candidates(self, seed_foothold, heightmap):
        """Generate candidate footholds in a grid around the seed position.
        
        Paper: Batch-search over grid cells in local window around predicted foothold.

        Args:
            seed_foothold: np.ndarray [x, y, z] seed foothold position in world frame
            heightmap: HeightMap object for querying terrain

        Returns:
            List of candidate [x, y] positions
        """
        # If heightmap has data, use local window approach (efficient)
        use_local_window = self.tamols_params.get('use_local_window', True)
        
        if heightmap is not None and heightmap.data is not None and not use_local_window:
            # Full heightmap scan (slower, for debugging/comparison)
            candidates = []
            rows, cols = heightmap.data.shape[:2]
            for i in range(rows):
                for j in range(cols):
                    # heightmap.data[i][j][0] is [x, y, z]
                    pos = heightmap.data[i][j][0]
                    candidates.append([pos[0], pos[1]])
            return candidates

        # Local window approach (default, efficient)
        radius = self.tamols_params.get('search_radius', 0.32)
        resolution = self.tamols_params.get('search_resolution', 0.04)

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

    def _compute_paper_objectives(self, candidate, seed, hip_position, heightmap,
                                   feet_positions, current_leg, legs_order, is_stance):
        """Compute weighted objectives from paper (Equations 2-9).
        
        Paper objectives in descending importance order:
        1. Default leg configuration (Eq. 2-4): penalize deviation from nominal foothold
        2. Foothold score (Eq. 5): terrain quality (edges, slopes, roughness)
        3. Push-over regularizer (Eq. 6): for swinging legs, penalize high terrain along trajectory
        4. Support area (Eq. 7): encourage larger support polygon
        5. Previous foothold continuity (Eq. 8): penalize deviation from previous optimal
        6. Leg over-extension (Eq. 9): regularize leg extension

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            seed: np.ndarray [x, y, z] seed/reference foothold in world frame
            hip_position: np.ndarray [x, y, z] hip (thigh) position in world frame
            heightmap: HeightMap object for querying terrain
            feet_positions: LegsAttr with current measured foot positions
            current_leg: str, name of current leg ('FL', 'FR', 'RL', 'RR')
            legs_order: list of leg names
            is_stance: bool, 1 if leg is in stance, 0 if swinging

        Returns:
            float: total weighted cost (lower is better)
        """
        # Load weights (descending importance as in paper)
        w_default_config = self.tamols_params.get('weight_default_config', 1.0)
        w_foothold_score = self.tamols_params.get('weight_foothold_score', 0.8)
        w_pushover = self.tamols_params.get('weight_pushover', 0.6)
        w_support_area = self.tamols_params.get('weight_support_area', 0.4)
        w_continuity = self.tamols_params.get('weight_continuity', 0.2)
        w_leg_extension = self.tamols_params.get('weight_leg_extension', 0.1)

        # Objective 1: Default leg configuration (Eq. 2-4)
        # Penalizes deviation from nominal foothold derived from thigh position
        cost_default = self._compute_default_config_cost(candidate, hip_position)

        # Objective 2: Foothold score (Eq. 5)
        # Terrain quality: edges, slopes, roughness (normalized to [0,1])
        cost_foothold_score = self._compute_foothold_score(candidate, heightmap)

        # Objective 3: Push-over regularizer (Eq. 6)
        # For swinging legs: max foothold-score along line from previous optimal to candidate
        # For grounded legs: zero
        cost_pushover = 0.0
        if not is_stance and self.previous_optimal_footholds[current_leg] is not None:
            cost_pushover = self._compute_pushover_cost(
                candidate, self.previous_optimal_footholds[current_leg], heightmap
            )

        # Objective 4: Support area (Eq. 7)
        # Encourage larger support polygon via distances to neighboring feet
        cost_support_area = self._compute_support_area_cost(
            candidate, feet_positions, current_leg, legs_order
        )

        # Objective 5: Previous foothold continuity (Eq. 8)
        # Penalize deviation from previous optimal foothold
        cost_continuity = 0.0
        if self.previous_optimal_footholds[current_leg] is not None:
            deviation = np.linalg.norm(candidate - self.previous_optimal_footholds[current_leg])
            cost_continuity = deviation ** 2

        # Objective 6: Leg over-extension (Eq. 9)
        # Regularize to reduce required leg extension
        cost_leg_extension = self._compute_leg_extension_cost(candidate, hip_position)

        # Total weighted cost
        total_cost = (
            w_default_config * cost_default +
            w_foothold_score * cost_foothold_score +
            w_pushover * cost_pushover +
            w_support_area * cost_support_area +
            w_continuity * cost_continuity +
            w_leg_extension * cost_leg_extension
        )

        return total_cost
    
    def _check_constraints(self, candidate, seed, hip_position, heightmap,
                          feet_positions, current_leg, legs_order, is_stance):
        """Check hard constraints from paper (Equations 10-11).
        
        Paper constraints:
        1. Max step height (Eq. 10): obstacle height along line < h_max
        2. Leg collision (Eq. 11): distance to neighboring feet >= d_min
        3. Leg over-extension: handled softly via objective (paper notes hard constraints are conservative)

        Args:
            candidate: np.ndarray [x, y, z] candidate foothold
            seed: np.ndarray [x, y, z] seed/reference foothold
            hip_position: np.ndarray [x, y, z] hip position
            heightmap: HeightMap object
            feet_positions: LegsAttr with current measured foot positions
            current_leg: str, current leg name
            legs_order: list of leg names
            is_stance: bool, 1 if stance, 0 if swing

        Returns:
            bool: True if candidate satisfies all constraints, False otherwise
        """
        # Constraint 1: Max step height (Eq. 10)
        # Check obstacle height along line from current stance foothold and from previous optimal
        h_max = self.tamols_params.get('h_max', 0.15)  # meters
        
        # Check line from current/previous stance foothold to candidate
        if self.previous_optimal_footholds[current_leg] is not None and not self._check_step_height_constraint(
            self.previous_optimal_footholds[current_leg], candidate, heightmap, h_max
        ):
            return False
        
        # Also check from seed foothold
        if not self._check_step_height_constraint(seed, candidate, heightmap, h_max):
            return False

        # Constraint 2: Leg collision (Eq. 11)
        # Reject candidates too close to neighboring end-effector locations
        d_min = self.tamols_params.get('d_min', 0.10)  # meters
        
        for leg_name in legs_order:
            if leg_name == current_leg:
                continue
            neighbor_foot = feet_positions[leg_name]
            distance = np.linalg.norm(candidate - neighbor_foot)
            if distance < d_min:
                return False

        # Constraint 3: Leg over-extension
        # Paper notes hard constraints are conservative; we handle via soft objective only
        # No hard rejection here
        
        return True

    def _compute_default_config_cost(self, candidate, hip_position):
        """Objective 1: Default leg configuration (Eq. 2-4).
        
        Penalizes deviation of candidate from default foot location derived from:
        - Thigh position (hip_position from paper)
        - Nominal leg extension direction (blend of world z-axis and local terrain normal)
        - Velocity feedback term using thigh velocity error (if available)
        
        Simplified implementation: penalize deviation from nominal foothold below hip.
        """
        l_nom = self.tamols_params.get('l_nom', 0.30)  # Nominal leg extension
        
        # Default foothold: directly below hip at nominal extension
        # Paper blends world z-axis with terrain normal; simplified here as downward
        default_foothold = hip_position.copy()
        default_foothold[2] -= l_nom
        
        # Penalize deviation from default
        deviation = np.linalg.norm(candidate - default_foothold)
        cost = deviation ** 2
        
        # TODO: Add velocity feedback term if thigh velocity is available
        # Paper Eq. 3-4: includes velocity error term
        
        return cost

    def _compute_foothold_score(self, candidate, heightmap):
        """Objective 2: Foothold score (Eq. 5).
        
        Terrain quality score combining edges, slopes, roughness.
        Normalize terms into [0,1] where 0=best, 1=worst.
        
        Higher score = worse terrain = avoid.
        """
        # Edge detection (gradient magnitude)
        edge_score = self._compute_edge_cost(candidate, heightmap)
        edge_score_normalized = min(edge_score / 1.0, 1.0)  # normalize assuming max gradient ~1.0
        
        # Roughness (height variance)
        roughness_score = self._compute_roughness_cost(candidate, heightmap)
        roughness_score_normalized = min(roughness_score / 0.1, 1.0)  # normalize assuming max variance ~0.1
        
        # Slope (local gradient magnitude, similar to edge)
        # Already captured by edge_score
        
        # Combine scores (paper Eq. 5: weighted sum, we use equal weights)
        foothold_score = (edge_score_normalized + roughness_score_normalized) / 2.0
        
        return foothold_score

    def _compute_pushover_cost(self, candidate, previous_optimal, heightmap):
        """Objective 3: Push-over regularizer (Eq. 6).
        
        For swinging legs, minimize maximum foothold-score along line segment
        between previous optimal foothold and candidate.
        Penalizes trajectories that pass over high obstacles.
        
        For grounded legs, this is zero (handled in caller).
        """
        num_samples = self.tamols_params.get('pushover_line_samples', 10)
        
        # Sample points along line from previous to candidate
        max_score = 0.0
        for i in range(num_samples):
            t = i / (num_samples - 1)  # interpolation parameter [0, 1]
            sample_point = (1 - t) * previous_optimal + t * candidate
            
            # Query height at sample point
            height = heightmap.get_height(sample_point)
            if height is not None:
                sample_point[2] = height
                # Compute foothold score at this point
                score = self._compute_foothold_score(sample_point, heightmap)
                max_score = max(max_score, score)
            else:
                # Can't query heightmap, assume high cost
                max_score = 1.0
                break
        
        return max_score

    def _compute_support_area_cost(self, candidate, feet_positions, current_leg, legs_order):
        """Objective 4: Support area (Eq. 7).
        
        Encourage larger support area via distances to neighboring legs' measured end-effector locations.
        Larger distances = larger support polygon = more stable = lower cost.
        
        Cost is inverse of mean distance to neighbors (normalized).
        """
        distances = []
        for leg_name in legs_order:
            if leg_name == current_leg:
                continue
            neighbor_foot = feet_positions[leg_name]
            distance = np.linalg.norm(candidate[:2] - neighbor_foot[:2])  # XY plane distance
            distances.append(distance)
        
        if len(distances) == 0:
            return 0.0
        
        mean_distance = np.mean(distances)
        
        # Inverse relationship: smaller distance = higher cost (penalize small support area)
        # Normalize assuming typical leg separation ~0.4m
        cost = max(0, 1.0 - mean_distance / 0.4)
        
        return cost

    def _compute_leg_extension_cost(self, candidate, hip_position):
        """Objective 6: Leg over-extension (Eq. 9).
        
        Regularize to reduce required leg extension.
        Penalizes candidates far from hip (over-extension).
        """
        l_nom = self.tamols_params.get('l_nom', 0.30)  # Nominal leg extension
        
        leg_vector = candidate - hip_position
        leg_extension = np.linalg.norm(leg_vector)
        
        # Penalize deviation from nominal extension
        deviation = abs(leg_extension - l_nom)
        cost = deviation ** 2
        
        return cost

    def _check_step_height_constraint(self, start, end, heightmap, h_max):
        """Constraint 1: Max step height (Eq. 10).
        
        Compute obstacle height along line between start and end.
        Constraint violated if any obstacle along line exceeds h_max.
        
        Args:
            start: np.ndarray [x, y, z] start position
            end: np.ndarray [x, y, z] end position
            heightmap: HeightMap object
            h_max: float, maximum allowable step height

        Returns:
            bool: True if constraint satisfied, False if violated
        """
        num_samples = self.tamols_params.get('obstacle_line_samples', 10)
        
        # Sample points along line from start to end
        for i in range(num_samples):
            t = i / (num_samples - 1)
            sample_point = (1 - t) * start + t * end
            
            # Query height at sample point
            height = heightmap.get_height(sample_point)
            if height is not None:
                # Check if obstacle height above start/end exceeds threshold
                obstacle_height = height - min(start[2], end[2])
                if obstacle_height > h_max:
                    return False
            else:
                # Can't query heightmap, conservatively reject
                return False
        
        return True

    def _compute_edge_cost(self, candidate, heightmap):
        """Estimate local terrain gradient magnitude (edge/slope risk).

        Uses finite differences over nearby heightmap queries to approximate gradient.

        Args:
            candidate: np.ndarray [x, y, z] position in world frame
            heightmap: HeightMap object

        Returns:
            float: gradient magnitude cost
        """
        delta = self.tamols_params.get('gradient_delta', 0.04)

        # Sample heights in a cross pattern (±x, ±y) around the candidate
        # P1: (x+d, y), P2: (x-d, y), P3: (x, y+d), P4: (x, y-d)
        heights = []
        offsets = [np.array([delta, 0, 0]), np.array([-delta, 0, 0]), np.array([0, delta, 0]), np.array([0, -delta, 0])]

        for offset in offsets:
            query_pos = candidate + offset
            h = heightmap.get_height(query_pos)
            if h is not None:
                heights.append(h)

        if len(heights) < 4:
            # Not enough data (e.g., outside map or hole), return high cost to avoid this area
            return 1.0

        # Estimate gradient using central finite differences: f'(x) approx (f(x+h) - f(x-h)) / 2h
        grad_x = abs(heights[0] - heights[1]) / (2 * delta)
        grad_y = abs(heights[2] - heights[3]) / (2 * delta)
        
        # Compute L2 norm of the gradient vector
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
        delta = self.tamols_params.get('gradient_delta', 0.04) # Sampling spacing (3cm), covering a ~6x6cm patch

        # Sample heights in a small 3x3 grid patch centered at candidate
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
            # Not enough data to compute reliable variance
            return 0.5

        # Compute variance of heights
        roughness = np.var(heights)

        return roughness
