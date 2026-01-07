import numpy as np
from scipy.spatial import cKDTree
from gym_quadruped.utils.quadruped_utils import LegsAttr
from quadruped_pympc import config as cfg
try:
    # 尝试相对导入 (适用于作为包运行)
    from .vfa.vfa import VFA
except (ImportError, ValueError):
    try:
        # 尝试绝对路径导入
        from quadruped_pympc.helpers.vfa.vfa import VFA
    except ImportError:
        try:
            # 尝试旧的导入方式
            from vfa.vfa import VFA
        except ImportError:
            print("VFA not installed, not open source yet")
            VFA = None


class FastHeightMap:
    """Wrapper for HeightMap to speed up get_height using KDTree."""
    def __init__(self, heightmap):
        self.data = heightmap.data
        # Flatten data for KDTree: (rows*cols, 2)
        # data shape is (rows, cols, 1, 3)
        self.points = self.data[:, :, 0, :2].reshape(-1, 2)
        self.heights = self.data[:, :, 0, 2].reshape(-1)
        self.tree = cKDTree(self.points)

    def get_height(self, target):
        # Query nearest neighbor
        # target is [x, y, z]
        dist, idx = self.tree.query(target[:2])
        return self.heights[idx] + 0.02


class VisualFootholdAdaptation:
    def __init__(self, legs_order, adaptation_strategy='height'):
        self.footholds_adaptation = LegsAttr(
            FL=np.array([0, 0, 0]), FR=np.array([0, 0, 0]), RL=np.array([0, 0, 0]), RR=np.array([0, 0, 0])
        )
        self.footholds_constraints = LegsAttr(FL=None, FR=None, RL=None, RR=None)
        self.initialized = False

        self.adaptation_strategy = adaptation_strategy

        if self.adaptation_strategy == 'vfa':
            if VFA is None:
                raise ImportError("VFA strategy requested but VFA module could not be imported.")
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
        n = heightmaps[leg_name].data.shape[0]
        r = round(foothold_id.item() / n)
        c = round(foothold_id.item() % n)

        if r >= n:
            r = n - 1
        if c >= n:
            c = n - 1

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
        gait_phases=None,
        base_position=None,
        current_contact=None,
        current_feet_pos=None,
    ):
        for leg_id, leg_name in enumerate(legs_order):
            if heightmaps[leg_name].data is None:
                return False

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
            self.forward_vel = forward_vel
            # Store base position for stability checking
            self.base_position = base_position
            # Store current contact for swing leg detection
            # Default to all swing [0,0,0,0] if not provided (0=swing, 1=stance)
            self.current_contact = current_contact if current_contact is not None else np.array([0, 0, 0, 0])
            self.current_feet_pos = current_feet_pos
            # TAMOLS-inspired foothold adaptation strategy
            # Performs local search around seed footholds using terrain-aware cost metrics
            # print(f"\n[TAMOLS] Adaptation Start")
            for leg_id, leg_name in enumerate(legs_order):
                seed_foothold = reference_footholds[leg_name].copy()
                hip_position = hip_positions[leg_name]
                heightmap = heightmaps[leg_name]

                # print(f"  Leg {leg_name} Seed: {seed_foothold}")

                if heightmap.data is None:
                    continue

                # Optimization: Use FastHeightMap with KDTree for O(log N) lookups instead of O(N)
                # This significantly speeds up the process when evaluating many candidates
                fast_heightmap = FastHeightMap(heightmap)

                # Generate candidate footholds around the seed
                candidates = self._generate_candidates(fast_heightmap, seed_foothold)
                # print(f"    Generated {len(candidates)} candidates")
                # print(f"Leg {leg_name}: Generated {len(candidates)} candidates around seed foothold.")
                # print("Candidates:", candidates)

                # Evaluate each candidate using TAMOLS-inspired cost metrics
                best_candidate = None
                best_score = float('inf')

                for i, candidate_xy in enumerate(candidates):
                    # Query height from heightmap
                    candidate = np.array([candidate_xy[0], candidate_xy[1], 0.0])
                    # Use fast_heightmap for speed
                    height = fast_heightmap.get_height(candidate)+0.005
                    if height is None:
                        continue  # Skip if heightmap query fails

                    candidate[2] = height

                    # Compute TAMOLS-inspired cost
                    # Pass fast_heightmap instead of original heightmap
                    score = self._compute_tamols_score(candidate, seed_foothold, hip_position, fast_heightmap,
                                                    leg_id, leg_name, legs_order)

                    if score < best_score:
                        # print(f"    -> New Best Candidate [{i}]: {candidate}, Score: {score:.4f}")
                        best_score = score
                        best_candidate = candidate

                # Use best candidate if found, otherwise keep seed
                if best_candidate is not None:
                    # print(f"  Leg {leg_name} Selected: {best_candidate} (Score: {best_score:.4f})")
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
                    # print(f"  Leg {leg_name} No valid candidate found. Fallback to seed.")
                    # Fallback to height adjustment only
                    height_adjustment = heightmap.get_height(seed_foothold)
                    if height_adjustment is not None:
                        reference_footholds[leg_name][2] = height_adjustment
        self.update_footholds_adaptation(reference_footholds)

        return True

    def _generate_candidates(self, heightmap, seed_foothold):
        """Generate candidate footholds in a grid around the seed position.
        Args:
            seed_foothold: np.ndarray [x, y, z] seed foothold position in world frame
        Returns:
            List of candidate [x, y] positions
        """
        if heightmap is not None and heightmap.data is not None:
        	# Vectorized implementation for speed
        	# heightmap.data shape is (rows, cols, 1, 3) -> reshape to (N, 3) -> take XY
            return heightmap.data[:, :, 0, :2].reshape(-1, 2).tolist()
            
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

    def _compute_tamols_score(self, candidate, seed, hip_position, heightmap,
                              leg_id, leg_name, legs_order):
        """Compute TAMOLS-inspired cost for a candidate foothold.
        
        NOTE: Lower score is better. The score represents a 'cost' or 'penalty'.
        
        Combines multiple cost terms inspired by TAMOLS:
        - Edge avoidance: penalizes high terrain gradients (edge risk)
        - Roughness: penalizes irregular terrain
        - Previous solution tracking: penalizes deviation from seed
        - Kinematic reachability: penalizes or rejects candidates outside leg reach
        - Nominal kinematics: maintains desired hip height and leg configuration (from tamols/costs.py:add_nominal_kinematic_cost)
        - Reference tracking: tracks desired velocity direction (from tamols/costs.py:add_tracking_cost)
        - Stability: penalizes candidates that reduce CoM-to-support distance (Scheme 2)
        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            seed: np.ndarray [x, y, z] seed/reference foothold in world frame
            hip_position: np.ndarray [x, y, z] hip position in world frame
            heightmap: HeightMap object for querying terrain
            leg_id: int, leg index (0=FL, 1=FR, 2=RL, 3=RR)
            leg_name: str, leg name ('FL', 'FR', 'RL', 'RR')
            legs_order: tuple of leg names
            reference_footholds: LegsAttr of reference footholds
        Returns:
            float: total cost (lower is better)
        """
        # --- Hard Constraints (Requirements: TR, LC, KF) ---
        
        # c) Kinematic Feasibility (KF): Check workspace at Touchdown and Liftoff
        if not self._check_kinematic_feasibility(candidate, hip_position):
            return float('inf')

        # # b) Leg Collision (LC): Check if leg collides with terrain
        if self._check_leg_collision(candidate, hip_position, heightmap):
            return float('inf')

        # Load cost weights
        w_edge = self.tamols_params.get('weight_edge_avoidance', 15.0)
        w_rough = self.tamols_params.get('weight_roughness', 10.0)
        w_dev = self.tamols_params.get('weight_deviation', 1.0)
        w_kin = self.tamols_params.get('weight_kinematic', 10.0)
        w_nominal_kin = self.tamols_params.get('weight_nominal_kinematic', 20.0)
        w_tracking = self.tamols_params.get('weight_reference_tracking', 2.0)

        w_stability = self.tamols_params.get('weight_stability', 10.0)

        # 2. Edge avoidance cost (gradient magnitude)
        edge_cost_val = self._compute_edge_cost(candidate, heightmap)
        
        # 3. Roughness cost (local height variance)
        roughness_cost_val = self._compute_roughness_cost(candidate, heightmap)

        # a) Terrain Roughness (TR): Threshold check
        # If the terrain is too rough or too steep, consider it unsafe and discard
        #max_edge_cost = self.tamols_params.get('max_edge_cost', 1) # approx 30 deg
        #max_roughness = self.tamols_params.get('max_roughness', 0.05) # 5cm std dev
        
        # if edge_cost_val > max_edge_cost or roughness_cost_val > max_roughness:
        #    return float('inf')

        # --- Soft Costs Calculation ---

        # 1. Kinematic reachability cost
        # NOTE: Hard constraints are already checked in _check_kinematic_feasibility at the start.
        # So distance is guaranteed to be within [l_min, l_max].
        # We rely on 'nominal_kinematic_cost' to guide the leg to the desired configuration.
        # 这个cost有很大问题，
        # hip_to_foot = candidate - hip_position
        # distance = np.linalg.norm(hip_to_foot)
        # l_min = self.tamols_params.get('l_min', {}).get(self.robot_name, 0.15)
        # l_max = self.tamols_params.get('l_max', {}).get(self.robot_name, 0.45)

        # Penalize candidates outside kinematic bounds
        # kinematic_cost = w_kin * (l_min - distance) ** 2
        kinematic_cost = 0.0  # Handled as hard constraint already

        # 2. Edge avoidance cost (gradient magnitude)
        edge_cost = edge_cost_val * w_edge
        roughness_cost = roughness_cost_val * w_rough

        # 4. Previous solution tracking (deviation from seed)
        # deviation = np.linalg.norm(candidate[:2] - seed[:2])
        # TODO: 后一种方法，需测试哪种好
        deviation = np.sum((candidate - seed) ** 2)
        deviation_cost = deviation * w_dev

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

        # 7. Stability cost (Scheme 2)
        # Penalizes candidates that reduce CoM-to-diagonal-support distance
        stability_cost = self._compute_stability_cost(
            candidate, leg_id, leg_name, legs_order
        ) * w_stability

        # Total cost combines all terms
        total_cost = (
            kinematic_cost + edge_cost + roughness_cost + deviation_cost +
            nominal_kinematic_cost + reference_tracking_cost + stability_cost
        )

        return total_cost

    def _check_kinematic_feasibility(self, candidate, hip_position):
        """Check if foothold is within leg workspace at Touchdown and Liftoff (KF)."""
        l_min = self.tamols_params.get('l_min', {}).get(self.robot_name, 0.15)
        l_max = self.tamols_params.get('l_max', {}).get(self.robot_name, 0.45)
        
        # 1. Check at Touch Down (current hip pos)
        dist_td = np.linalg.norm(candidate - hip_position)
        if not (l_min <= dist_td <= l_max):
            return False
            
        # 2. Check at Lift Off (future hip pos)
        # Estimate stance duration (e.g., 0.3s)
        stance_duration = 0.3 
        if hasattr(self, 'forward_vel') and self.forward_vel is not None:
            # Predict hip position at lift off: Hip_LO = Hip_TD + Vel * T_stance
            hip_position_lo = hip_position + self.forward_vel[:3] * stance_duration
            dist_lo = np.linalg.norm(candidate - hip_position_lo)
            if not (l_min <= dist_lo <= l_max):
                return False
                
        return True

    def _check_leg_collision(self, candidate, hip_position, heightmap):
        """Check if the leg (line segment from hip to foot) collides with terrain (LC)."""
        # Sample points along the leg
        num_samples = 5
        # Start a bit away from hip and end a bit before foot
        alphas = np.linspace(0.2, 0.8, num_samples) 
        
        for alpha in alphas:
            # Point on the leg line
            p_leg = (1 - alpha) * hip_position + alpha * candidate
            
            # Query terrain height at this XY
            # heightmap.get_height returns z_ground + 0.02
            h_ground_sensor = heightmap.get_height(p_leg)
            if h_ground_sensor is None: 
                continue
            h_ground = h_ground_sensor - 0.02
            
            # Check if leg point is below ground (with margin for leg thickness)
            # If p_leg.z < h_ground + margin, collision.
            if p_leg[2] < (h_ground + 0.02):
                return True # Collision detected
                
        return False

    def _compute_edge_cost(self, candidate, heightmap):
        """Estimate local terrain gradient magnitude (edge/slope risk).
        
        Principle:
            Uses Central Finite Difference method to approximate the gradient of the terrain surface.
            Gradient Magnitude = sqrt((dz/dx)^2 + (dz/dy)^2)
            High gradient indicates steep slopes or edges (stairs/curbs), which are risky.

        Args:
            candidate: np.ndarray [x, y, z] position in world frame
            heightmap: HeightMap object
        Returns:
            float: gradient magnitude cost
        """
        # Delta should be related to grid resolution or foot size. 
        # 0.04m (4cm) is a good default for standard quadruped feet (~4cm diameter) and grid maps (~2-4cm res).
        delta = self.tamols_params.get('gradient_delta', 0.04)
        slope_threshold = self.tamols_params.get('slope_threshold', 0.7)
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

        if gradient_magnitude <= slope_threshold:
            return 0.0
        else:
            # Penalize the excess gradient beyond the acceptable slope threshold
            return gradient_magnitude - slope_threshold

    def _compute_roughness_cost(self, candidate, heightmap):
        """Estimate local terrain roughness (height standard deviation).
        
        Principle:
            Measures the statistical dispersion of terrain height in a small patch around the candidate.
            Roughness = StdDev(heights)
            High std dev indicates uneven, bumpy, or noisy terrain (e.g., rocks, debris), 
            which provides poor contact stability.
            For stepping stones, this penalizes positions straddling the edge.

        Args:
            candidate: np.ndarray [x, y, z] position in world frame
            heightmap: HeightMap object
        Returns:
            float: roughness (standard deviation) cost
        """
        delta = self.tamols_params.get('gradient_delta', 0.04) # Sampling spacing (3cm), covering a ~6x6cm patch

        # Sample heights in a small 3x3 grid patch centered at candidate
        heights = []
        positions = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                query_pos = candidate.copy()
                query_pos[0] += i * delta
                query_pos[1] += j * delta
                h = heightmap.get_height(query_pos)
                if h is not None:
                    heights.append(h)
                    positions.append([i * delta, j * delta])
                    
        if len(heights) < 5:
            # Not enough data to compute reliable variance
            return 0.5

        heights = np.array(heights)
        positions = np.array(positions)
        if len(heights) >= 3:
            # Build design matrix [x, y, 1]
            design_matrix = np.column_stack([positions[:, 0], positions[:, 1], np.ones(len(heights))])
            try:
                # Solve least squares: design_matrix @ [a, b, c]^T = heights
                plane_params, _, _, _ = np.linalg.lstsq(design_matrix, heights, rcond=None)
                # Compute predicted heights on the fitted plane
                predicted_heights = design_matrix @ plane_params
                # Compute detrended variance (variance from the plane)
                detrended_variance = np.var(heights - predicted_heights)
                return detrended_variance
            except np.linalg.LinAlgError:
                # If plane fitting fails, fall back to raw variance
                return np.var(heights)
        else:
            # Not enough points for plane fitting, use raw variance
            return np.var(heights)

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

    def _compute_stability_cost(self, candidate, leg_id, leg_name, legs_order):
        """Compute stability cost based on CoM distance to diagonal support line (Scheme 2).
        For trot gait, evaluates stability with respect to diagonal pair support segment:
        - FL <-> RR (diagonal pair)
        - FR <-> RL (diagonal pair)
        The correct stability criterion for trot is that the CoM should be CLOSE to the 
        diagonal support line (small distance is good), not far from it. This minimizes 
        roll moment and maintains stable support polygon.
        Computes the distance from predicted CoM projection (xy) to the line segment between
        the candidate foothold and its diagonal pair's CURRENT/ACTUAL foothold position.
        Penalizes candidates that would place CoM FAR from the support line.
        Args:
            candidate: np.ndarray [x, y, z] candidate foothold in world frame
            leg_id: int, leg index (0=FL, 1=FR, 2=RL, 3=RR)
            leg_name: str, leg name ('FL', 'FR', 'RL', 'RR')
            legs_order: tuple of leg names
        Returns:
            float: stability cost (lower is better when CoM is close to support line)
        """
        # Check if we have required data for stability computation
        if not hasattr(self, 'base_position') or self.base_position is None:
            return 0.0
        if not hasattr(self, 'current_feet_pos') or self.current_feet_pos is None:
            return 0.0
        # Load stability parameters
        stability_margin = self.tamols_params.get('stability_margin', 0.06)

        # Determine diagonal pair for this leg (trot gait diagonal pairing)
        # FL (0) <-> RR (3), FR (1) <-> RL (2)
        diagonal_pairs = {
            'FL': 'RR',
            'FR': 'RL',
            'RL': 'FR',
            'RR': 'FL'
        }

        if leg_name not in diagonal_pairs:
            return 0.0

        diagonal_leg_name = diagonal_pairs[leg_name]

        # Check if we should apply stability check based on contact state
        # Only apply when the current leg is in swing (being adapted)
        if hasattr(self, 'current_contact') and self.current_contact is not None:
            # current_contact is array [FL, FR, RL, RR] with 1=stance, 0=swing
            if leg_id < len(self.current_contact) and self.current_contact[leg_id] == 1:
                # Leg is in stance, not being adapted, skip stability check
                return 0.0

        # Get diagonal leg's CURRENT/ACTUAL foothold position (where it's standing now)
        if not hasattr(self.current_feet_pos, diagonal_leg_name):
            return 0.0
        try:
            diagonal_foothold = getattr(self.current_feet_pos, diagonal_leg_name)
        except (AttributeError, TypeError):
            return 0.0

        # Predict future CoM position when this foothold will be used
        # Estimate swing time: approximately (1 - duty_factor) * period
        # Use a reasonable default if not available
        swing_time = self.tamols_params.get('estimated_swing_time', 0.25)  # ~0.25s typical

        # Predict CoM position: current + velocity * time
        # Use only xy components for 2D stability
        predicted_com_xy = self.base_position[:2] + self.forward_vel[:2] * swing_time

        # Compute distance from predicted CoM to line segment between candidate and diagonal foothold
        # Line segment: P1 = candidate[:2], P2 = diagonal_foothold[:2]
        # Point: C = predicted_com_xy
        p1 = candidate[:2]
        p2 = diagonal_foothold[:2]
        c = predicted_com_xy

        # Vector from P1 to P2
        v = p2 - p1
        # Vector from P1 to C
        w = c - p1

        # Compute projection parameter t
        v_dot_v = np.dot(v, v)
        if v_dot_v < 1e-8:
            # P1 and P2 are essentially the same point, use distance to P1
            distance = np.linalg.norm(c - p1)
        else:
            # Parameter t for projection onto line (not segment yet)
            t = np.dot(w, v) / v_dot_v
            # Clamp t to [0, 1] for line segment
            t = np.clip(t, 0.0, 1.0)
            # Closest point on segment
            closest_point = p1 + t * v
            # Distance from predicted CoM to closest point on support line
            distance = np.linalg.norm(c - closest_point)

        # CORRECT stability criterion: penalize when distance is TOO LARGE
        # We want CoM close to support line (small distance = stable)
        # Apply quadratic penalty that increases with distance
        # No penalty when distance <= stability_margin (acceptable zone)
        if distance > stability_margin:
            # CoM is too far from support line, penalize
            penalty = (distance - stability_margin) ** 2
            return penalty
        else:
            # CoM is close to support line (good for stability)
            return 0.0
