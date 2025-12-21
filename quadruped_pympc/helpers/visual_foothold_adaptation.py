import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg

try:
    from virall.vfa.vfa import VFA
except ImportError:
    print("VFA not installed, not open source yet")


class VisualFootholdAdaptation:
    """Visual foothold adaptation for quadruped robots on rough terrain.
    
    Supports three adaptation strategies:
    1. 'height': Simple height adjustment from heightmap
    2. 'vfa': Visual Foothold Adaptation using convex region analysis
    3. 'tamols': TAMOLS-inspired terrain-aware foothold selection with stability constraints
    
    TAMOLS Strategy with Joint Optimization (NEW):
    -----------------------------------------------
    The TAMOLS strategy has been enhanced with joint (multi-leg) foothold selection
    that considers stability constraints aligned with the NMPC controller.
    
    Key improvements:
    - Instead of optimizing each leg independently, selects footholds jointly for leg groups
      (e.g., diagonal pairs FL-RR and FR-RL in trot gait)
    - Evaluates stability constraints using the same inequality structure as
      Acados_NMPC_Nominal.create_stability_constraints()
    - Ensures the selected footholds keep the stability point (COM projection or ZMP)
      inside the support polygon with a configurable safety margin
    
    Alignment with NMPC Stability Constraints:
    ------------------------------------------
    The joint optimization uses the same mathematical framework as the NMPC controller:
    1. Transforms foothold positions to horizontal (yaw-rotated) frame: h_R_w @ (foot - base)
    2. Computes stability proxy point in the same frame (default: COM projection at origin)
    3. Evaluates 6 inequality constraints defining support polygon edges and diagonals
    4. Penalizes foothold combinations that violate these constraints
    
    This ensures consistency between foothold adaptation and MPC internal constraints,
    preventing locally optimal but globally unstable foothold selections.
    
    Configuration:
    --------------
    Set in config.py under simulation_params['tamols_params']:
    - joint_optimize: Enable joint foothold optimization (default: True)
    - top_k_per_leg: Number of top candidates per leg for joint evaluation (default: 15)
    - stability_margin: Safety margin for stability constraints in meters (default: 0.02)
    - joint_weight_stability: Weight for stability violations in cost (default: 1000.0)
    
    See also:
    - quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal.Acados_NMPC_Nominal.create_stability_constraints()
    - docs/TAMOLS_FOOTHOLD_ADAPTATION.md
    """
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
        base_position=None,
    ):
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
            # Store forward velocity for TAMOLS reference tracking
            self.forward_vel = forward_vel
            
            # =========================================================================
            # TAMOLS-inspired foothold adaptation strategy
            # =========================================================================
            # Performs local search around seed footholds using terrain-aware cost metrics
            #
            # NEW: Joint foothold selection for stability-aware adaptation
            # ------------------------------------------------------------
            # This implementation aligns with NMPC stability constraints defined in
            # Acados_NMPC_Nominal.create_stability_constraints() by:
            #
            # 1. Transforming footholds to the same horizontal (yaw-rotated) frame used by NMPC
            # 2. Evaluating the same inequality constraints that enforce ZMP/COM inside support polygon
            # 3. Selecting footholds jointly for leg groups (e.g., diagonal pairs in trot)
            #    to ensure the resulting support geometry keeps the stability point feasible
            #
            # Benefits:
            # - Prevents locally good but globally poor foothold selections
            # - Improves balance especially in trot gait (diagonal support)
            # - Consistent with NMPC's internal stability constraints
            # =========================================================================

            # Try joint optimization if base_position is available
            joint_success = False
            if base_position is not None:
                joint_success = self._apply_joint_foothold_selection(
                    legs_order,
                    reference_footholds,
                    hip_positions,
                    heightmaps,
                    base_position,
                    base_orientation,
                    forward_vel,
                )

            # Fall back to single-leg optimization if joint optimization disabled or failed
            if not joint_success:
                for _leg_id, leg_name in enumerate(legs_order):
                    seed_foothold = reference_footholds[leg_name].copy()
                    hip_position = hip_positions[leg_name]
                    heightmap = heightmaps[leg_name]

                    # Generate candidate footholds around the seed
                    candidates = self._generate_candidates(seed_foothold, heightmap)

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
                            candidate, seed_foothold, hip_position, heightmap
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

    def _generate_candidates(self, seed_foothold, heightmap=None):
        """Generate candidate footholds in a grid around the seed position.

        Args:
            seed_foothold: np.ndarray [x, y, z] seed foothold position in world frame
            heightmap: HeightMap object (optional) for sampling from heightmap grid

        Returns:
            List of candidate [x, y] positions
        """
        if heightmap is not None and heightmap.data is not None:
            candidates = []
            rows, cols = heightmap.data.shape[:2]
            for i in range(rows):
                for j in range(cols):
                    # heightmap.data[i][j][0] is [x, y, z]
                    pos = heightmap.data[i][j][0]
                    candidates.append([pos[0], pos[1]])
            return candidates

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

    def _compute_tamols_score(self, candidate, seed, hip_position, heightmap):
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

    def _compute_stability_proxy(self, base_position, base_orientation, forward_vel=None):
        """Compute proxy stability point in horizontal frame.

        This matches the NMPC stability constraint implementation in
        create_stability_constraints() which uses either COM projection (static)
        or ZMP (dynamic) in the horizontal (yaw-rotated) frame.

        Args:
            base_position: np.ndarray [x, y, z] base position in world frame
            base_orientation: np.ndarray [roll, pitch, yaw] base orientation
            forward_vel: np.ndarray [vx, vy, vz] base velocity in world frame (optional)

        Returns:
            np.ndarray [x, y]: stability point in horizontal frame relative to base (0, 0)
        """
        # Default: use static stability (COM projection at base origin in horizontal frame)
        # This matches: if self.use_static_stability: x = 0.0, y = 0.0
        # For conservative adaptation, we use the static case
        # A future extension could add dynamic proxy based on velocity
        return np.array([0.0, 0.0])

    def _evaluate_stability_constraints(self, foothold_positions, stability_point, margin=0.02):
        """Evaluate NMPC-like stability constraints for a set of footholds.

        This implements the same inequality structure as create_stability_constraints()
        in centroidal_nmpc_nominal.py. The constraints enforce that the stability point
        (x, y) stays inside the support polygon/line formed by the footholds.

        The constraint inequalities are:
        - FL-FR edge: x <= (x_FR - x_FL) * (y - y_FL) / (y_FR - y_FL + eps) + x_FL
        - FR-RR edge: y >= (y_RR - y_FR) * (x - x_FR) / (x_RR - x_FR + eps) + y_FR
        - RR-RL edge: x >= (x_RL - x_RR) * (y - y_RR) / (y_RL - y_RR + eps) + x_RR
        - RL-FL edge: y <= (y_FL - y_RL) * (x - x_RL) / (x_FL - x_RL + eps) + y_RL
        - FL-RR diagonal: y >= (y_RR - y_FL) * (x - x_FL) / (x_RR - x_FL + eps) + y_FL
        - FR-RL diagonal: y >= (y_RL - y_FR) * (x - x_FR) / (x_RL - x_FR + eps) + y_FR

        Args:
            foothold_positions: dict with keys 'FL', 'FR', 'RL', 'RR' containing
                               np.ndarray [x, y] positions in horizontal frame relative to base
            stability_point: np.ndarray [x, y] stability point in horizontal frame
            margin: float, safety margin (positive = shrink polygon inward)

        Returns:
            float: total constraint violation (0 if all constraints satisfied, positive if violated)
        """
        eps = 0.001  # Small epsilon to avoid division by zero (same as NMPC)
        x, y = stability_point

        # Extract foothold positions
        # Using uppercase leg names to match NMPC convention (see create_stability_constraints)
        x_FL, y_FL = foothold_positions['FL']  # noqa: N806
        x_FR, y_FR = foothold_positions['FR']  # noqa: N806
        x_RL, y_RL = foothold_positions['RL']  # noqa: N806
        x_RR, y_RR = foothold_positions['RR']  # noqa: N806

        # Compute constraint values (same as NMPC create_stability_constraints)
        # For each constraint, negative/zero means satisfied, positive means violated
        # The margin shrinks the safe region inward

        # FL-FR edge: x <= ... or equivalently: x - ... <= 0
        constraint_FL_FR = x - (x_FR - x_FL) * (y - y_FL) / (y_FR - y_FL + eps) - x_FL  # noqa: N806
        # Upper bound is 0, adding margin shrinks safe region: x - ... <= -margin
        # So violation is when constraint_FL_FR > -margin, i.e., max(constraint_FL_FR + margin, 0)
        violation_FL_FR = max(constraint_FL_FR + margin, 0.0)  # noqa: N806

        # FR-RR edge: y >= ... or equivalently: ... - y <= 0
        constraint_FR_RR = (y_RR - y_FR) * (x - x_FR) / (x_RR - x_FR + eps) + y_FR - y  # noqa: N806
        # Lower bound is 0, adding margin shrinks safe region: ... - y <= -margin
        # So violation is when constraint_FR_RR > -margin, i.e., max(constraint_FR_RR + margin, 0)
        violation_FR_RR = max(constraint_FR_RR + margin, 0.0)  # noqa: N806

        # RR-RL edge: x >= ... or equivalently: ... - x <= 0
        constraint_RR_RL = (x_RL - x_RR) * (y - y_RR) / (y_RL - y_RR + eps) + x_RR - x  # noqa: N806
        # Lower bound is 0, adding margin shrinks safe region: ... - x <= -margin
        # So violation is when constraint_RR_RL > -margin, i.e., max(constraint_RR_RL + margin, 0)
        violation_RR_RL = max(constraint_RR_RL + margin, 0.0)  # noqa: N806

        # RL-FL edge: y <= ... or equivalently: y - ... <= 0
        constraint_RL_FL = y - (y_FL - y_RL) * (x - x_RL) / (x_FL - x_RL + eps) - y_RL  # noqa: N806
        # Upper bound is 0, so violation is max(constraint_RL_FL + margin, 0)
        violation_RL_FL = max(constraint_RL_FL + margin, 0.0)  # noqa: N806

        # FL-RR diagonal: y >= ... (for trot/diagonal support)
        constraint_FL_RR = (y_RR - y_FL) * (x - x_FL) / (x_RR - x_FL + eps) + y_FL - y  # noqa: N806
        violation_FL_RR = max(constraint_FL_RR + margin, 0.0)  # noqa: N806

        # FR-RL diagonal: y >= ... (for trot/diagonal support)
        constraint_FR_RL = (y_RL - y_FR) * (x - x_FR) / (x_RL - x_FR + eps) + y_FR - y  # noqa: N806
        violation_FR_RL = max(constraint_FR_RL + margin, 0.0)  # noqa: N806

        # Total violation (sum of all constraint violations)
        total_violation = (
            violation_FL_FR + violation_FR_RR + violation_RR_RL +
            violation_RL_FL + violation_FL_RR + violation_FR_RL
        )

        return total_violation

    def _transform_to_horizontal_frame(self, world_positions, base_position, base_yaw):
        """Transform foot positions from world frame to horizontal (yaw-rotated) frame.

        This matches the transformation in NMPC create_stability_constraints():
        h_R_w = rotation matrix for yaw
        foot_horizontal = h_R_w @ (foot_world - base_world)

        Args:
            world_positions: dict with keys 'FL', 'FR', 'RL', 'RR' containing
                            np.ndarray [x, y, z] positions in world frame
            base_position: np.ndarray [x, y, z] base position in world frame
            base_yaw: float, base yaw angle in radians

        Returns:
            dict with keys 'FL', 'FR', 'RL', 'RR' containing
            np.ndarray [x, y] positions in horizontal frame relative to base
        """
        # Rotation matrix for yaw (same as NMPC)
        cos_yaw = np.cos(base_yaw)
        sin_yaw = np.sin(base_yaw)
        h_R_w = np.array([[cos_yaw, sin_yaw],  # noqa: N806
                          [-sin_yaw, cos_yaw]])

        horizontal_positions = {}
        for leg_name in ['FL', 'FR', 'RL', 'RR']:
            # Transform to horizontal frame: h_R_w @ (foot_world - base_world)
            foot_world_xy = world_positions[leg_name][:2]
            base_world_xy = base_position[:2]
            foot_horizontal_xy = h_R_w @ (foot_world_xy - base_world_xy)
            horizontal_positions[leg_name] = foot_horizontal_xy

        return horizontal_positions

    def _determine_joint_leg_sets(self, legs_order, gait_type='trot'):
        """Determine which leg sets should be jointly optimized.

        For trot: diagonal pairs (FL-RR, FR-RL)
        For pace: lateral pairs (FL-FR, RL-RR)
        For crawl/full stance: all combinations or individual legs

        Args:
            legs_order: list of leg names
            gait_type: str, gait type (default 'trot')

        Returns:
            list of tuples, each tuple contains leg names to jointly optimize
        """
        # For now, implement trot (diagonal pairs) as the primary use case
        # Future extension: detect gait from config or parameters
        gait = cfg.simulation_params.get('gait', 'trot')

        if gait in ['trot', 'bound']:
            # Diagonal pairs
            return [('FL', 'RR'), ('FR', 'RL')]
        elif gait == 'pace':
            # Lateral pairs
            return [('FL', 'FR'), ('RL', 'RR')]
        elif gait in ['crawl', 'full_stance']:
            # For crawl, optimize all legs together (or could do tripods)
            return [('FL', 'FR', 'RL', 'RR')]
        else:
            # Default: diagonal pairs (trot-like)
            return [('FL', 'RR'), ('FR', 'RL')]

    def _joint_foothold_selection(
        self,
        legs_order,
        reference_footholds,
        hip_positions,
        heightmaps,
        base_position,
        base_orientation,
        forward_vel,
    ):
        """Perform joint (multi-leg) foothold selection using stability-aware cost.

        This implements the core improvement: instead of optimizing each leg independently,
        we select footholds jointly for groups of legs (e.g., diagonal pairs in trot)
        to ensure the resulting support polygon/line keeps the stability point feasible.

        Args:
            legs_order: list of leg names
            reference_footholds: LegsAttr with seed footholds
            hip_positions: LegsAttr with hip positions
            heightmaps: LegsAttr with heightmap objects
            base_position: np.ndarray [x, y, z]
            base_orientation: np.ndarray [roll, pitch, yaw]
            forward_vel: np.ndarray [vx, vy, vz]

        Returns:
            LegsAttr with selected footholds, or None if joint selection fails
        """
        joint_optimize = self.tamols_params.get('joint_optimize', True)
        if not joint_optimize:
            return None  # Fall back to single-leg optimization

        top_k = self.tamols_params.get('top_k_per_leg', 15)
        stability_margin = self.tamols_params.get('stability_margin', 0.02)
        stability_weight = self.tamols_params.get('joint_weight_stability', 1000.0)

        # Determine which leg sets to jointly optimize
        leg_sets = self._determine_joint_leg_sets(legs_order)

        # Generate top-K candidates for each leg
        leg_candidates = {}
        leg_candidate_scores = {}

        for leg_name in legs_order:
            seed_foothold = reference_footholds[leg_name].copy()
            hip_position = hip_positions[leg_name]
            heightmap = heightmaps[leg_name]

            # Generate candidates
            candidates = self._generate_candidates(seed_foothold, heightmap)

            # Evaluate each candidate using single-leg TAMOLS cost
            scored_candidates = []
            for candidate_xy in candidates:
                candidate = np.array([candidate_xy[0], candidate_xy[1], 0.0])
                height = heightmap.get_height(candidate)
                if height is None:
                    continue  # Skip if heightmap query fails

                candidate[2] = height

                # Check kinematic bounds
                hip_to_foot = candidate - hip_position
                distance = np.linalg.norm(hip_to_foot)
                l_min = self.tamols_params.get('l_min', {}).get(self.robot_name, 0.15)
                l_max = self.tamols_params.get('l_max', {}).get(self.robot_name, 0.45)
                if distance < l_min or distance > l_max:
                    continue  # Skip kinematically infeasible candidates

                # Compute single-leg TAMOLS score
                score = self._compute_tamols_score(
                    candidate, seed_foothold, hip_position, heightmap
                )
                scored_candidates.append((score, candidate))

            # Sort by score and keep top-K
            scored_candidates.sort(key=lambda x: x[0])
            top_candidates = scored_candidates[:top_k]

            if len(top_candidates) == 0:
                # No valid candidates for this leg, joint optimization cannot proceed
                return None

            leg_candidates[leg_name] = [c[1] for c in top_candidates]
            leg_candidate_scores[leg_name] = [c[0] for c in top_candidates]

        # Now evaluate combinations for each leg set
        best_footholds = reference_footholds.copy()
        base_yaw = base_orientation[2]

        for leg_set in leg_sets:
            # Evaluate all combinations of candidates for this leg set
            best_combination = None
            best_combination_cost = float('inf')

            # Generate all combinations
            if len(leg_set) == 2:
                # Pair: evaluate K^2 combinations
                leg_a, leg_b = leg_set
                for i, cand_a in enumerate(leg_candidates[leg_a]):
                    for j, cand_b in enumerate(leg_candidates[leg_b]):
                        # Build candidate foothold positions
                        candidate_footholds = {
                            'FL': best_footholds['FL'],
                            'FR': best_footholds['FR'],
                            'RL': best_footholds['RL'],
                            'RR': best_footholds['RR']
                        }
                        candidate_footholds[leg_a] = cand_a
                        candidate_footholds[leg_b] = cand_b

                        # Transform to horizontal frame
                        horizontal_positions = self._transform_to_horizontal_frame(
                            candidate_footholds, base_position, base_yaw
                        )

                        # Compute stability point proxy
                        stability_point = self._compute_stability_proxy(
                            base_position, base_orientation, forward_vel
                        )

                        # Evaluate stability constraints
                        stability_violation = self._evaluate_stability_constraints(
                            horizontal_positions, stability_point, stability_margin
                        )

                        # Combined cost: single-leg scores + stability penalty
                        single_leg_cost = leg_candidate_scores[leg_a][i] + leg_candidate_scores[leg_b][j]
                        total_cost = single_leg_cost + stability_weight * stability_violation

                        if total_cost < best_combination_cost:
                            best_combination_cost = total_cost
                            best_combination = {leg_a: cand_a, leg_b: cand_b}

            elif len(leg_set) == 4:
                # All four legs: evaluate K^4 combinations (can be expensive, limit K)
                limited_k = min(5, top_k)  # Limit to avoid combinatorial explosion
                from itertools import product
                for cand_combo in product(
                    leg_candidates['FL'][:limited_k],
                    leg_candidates['FR'][:limited_k],
                    leg_candidates['RL'][:limited_k],
                    leg_candidates['RR'][:limited_k]
                ):
                    candidate_footholds = {
                        'FL': cand_combo[0],
                        'FR': cand_combo[1],
                        'RL': cand_combo[2],
                        'RR': cand_combo[3]
                    }

                    # Transform to horizontal frame
                    horizontal_positions = self._transform_to_horizontal_frame(
                        candidate_footholds, base_position, base_yaw
                    )

                    # Compute stability point proxy
                    stability_point = self._compute_stability_proxy(
                        base_position, base_orientation, forward_vel
                    )

                    # Evaluate stability constraints
                    stability_violation = self._evaluate_stability_constraints(
                        horizontal_positions, stability_point, stability_margin
                    )

                    # Combined cost
                    single_leg_cost = sum(
                        leg_candidate_scores[leg_name][
                            leg_candidates[leg_name].index(cand_combo[idx])
                        ]
                        for idx, leg_name in enumerate(['FL', 'FR', 'RL', 'RR'])
                    )
                    total_cost = single_leg_cost + stability_weight * stability_violation

                    if total_cost < best_combination_cost:
                        best_combination_cost = total_cost
                        best_combination = {
                            'FL': cand_combo[0],
                            'FR': cand_combo[1],
                            'RL': cand_combo[2],
                            'RR': cand_combo[3]
                        }

            # Update best footholds with the selected combination
            if best_combination is not None:
                for leg_name, foothold in best_combination.items():
                    best_footholds[leg_name] = foothold

        return best_footholds

    def _apply_joint_foothold_selection(
        self,
        legs_order,
        reference_footholds,
        hip_positions,
        heightmaps,
        base_position,
        base_orientation,
        forward_vel,
    ):
        """Apply joint foothold selection and update reference footholds.

        This is a wrapper that calls _joint_foothold_selection and handles fallback.

        Args:
            legs_order: list of leg names
            reference_footholds: LegsAttr with seed footholds (will be modified in place)
            hip_positions: LegsAttr with hip positions
            heightmaps: LegsAttr with heightmap objects
            base_position: np.ndarray [x, y, z]
            base_orientation: np.ndarray [roll, pitch, yaw]
            forward_vel: np.ndarray [vx, vy, vz]

        Returns:
            bool: True if joint selection succeeded, False if fell back to single-leg
        """
        # Try joint selection
        joint_footholds = self._joint_foothold_selection(
            legs_order,
            reference_footholds,
            hip_positions,
            heightmaps,
            base_position,
            base_orientation,
            forward_vel,
        )

        if joint_footholds is not None:
            # Update reference footholds with joint selection
            for leg_name in legs_order:
                reference_footholds[leg_name] = joint_footholds[leg_name]

                # Create foothold constraints (simple box around chosen foothold)
                dx = self.tamols_params.get('constraint_box_dx', 0.05)
                dy = self.tamols_params.get('constraint_box_dy', 0.05)
                vertex1 = joint_footholds[leg_name].copy()
                vertex1[0] -= dx
                vertex1[1] -= dy
                vertex2 = joint_footholds[leg_name].copy()
                vertex2[0] += dx
                vertex2[1] += dy
                self.footholds_constraints[leg_name] = [vertex1, vertex2]

            return True

        return False
