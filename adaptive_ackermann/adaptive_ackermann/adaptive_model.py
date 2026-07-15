"""Pure adaptive-model and signed Nav2 path geometry helpers."""

import math


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def isotonic_points(points):
    """Project (x, y) pairs onto the nearest monotonic sequence in y.

    Pool-adjacent-violators against the trend implied by the endpoints.
    Learned maps must stay monotonic to stay invertible; a refit that
    crosses itself steers the wrong way in the crossed region (ODAAC
    steal #2 — the reverse-center -0.108 poisoning class).
    Returns (projected_points, changed)."""
    pts = sorted((float(x), float(y)) for x, y in points)
    if len(pts) < 2:
        return pts, False
    sign = 1.0 if pts[-1][1] >= pts[0][1] else -1.0
    ys = [sign * y for _, y in pts]
    # pool adjacent violators (weights = pooled counts)
    blocks = [[y, 1] for y in ys]
    merged = []
    for block in blocks:
        merged.append(block)
        while len(merged) > 1 and merged[-2][0] > merged[-1][0]:
            b = merged.pop()
            a = merged.pop()
            total = a[1] + b[1]
            merged.append([(a[0] * a[1] + b[0] * b[1]) / total, total])
    projected = []
    for value, count in merged:
        projected.extend([value] * count)
    # Pooling makes violators EQUAL — but equal y values invert into
    # duplicate x values, and interpolation on duplicates divides by
    # zero (crashed the controller 2026-07-15, 40 s into a session).
    # Enforce STRICTLY monotonic output.
    for i in range(1, len(projected)):
        if projected[i] <= projected[i - 1]:
            projected[i] = projected[i - 1] + 1e-6
    changed = any(abs(a - b) > 1e-9 for a, b in zip(ys, projected))
    return ([(x, sign * y) for (x, _), y in zip(pts, projected)], changed)


def compose_preview_curvature(rpp_curvature, current_path_curvature,
                              future_path_curvature, confidence,
                              maximum_curvature, feedback_cap=None):
    """Lead path feed-forward while bounding RPP's feedback component.

    The feedback term is RPP's error correction on top of what the path
    itself asks for. A loop with ~0.4 s of measured delay cannot stably
    close large corrections — uncapped, they limit-cycle (the S-snake on
    straights). The cap bounds error response; path-following authority
    (the feed-forward term) stays at full executable curvature."""
    feedback = rpp_curvature - current_path_curvature
    if feedback_cap is not None:
        feedback = clamp(feedback, -abs(feedback_cap), abs(feedback_cap))
    future_command = future_path_curvature + feedback
    blended = rpp_curvature + confidence * (future_command - rpp_curvature)
    return clamp(blended, -maximum_curvature, maximum_curvature)


def limit_ackermann_twist(linear_velocity, angular_velocity,
                          maximum_forward_velocity,
                          maximum_reverse_velocity, maximum_curvature):
    """Project a Twist onto the vehicle's executable Ackermann envelope.

    Nav2 and Collision Monitor exchange instantaneous Twists, but this vehicle
    cannot rotate in place and its angular velocity is coupled to linear speed
    by curvature.  Preserve the requested curvature when possible, while
    bounding both speed and curvature to values the actuator bridge can execute.
    """
    raw_linear = float(linear_velocity)
    raw_angular = float(angular_velocity)
    if not math.isfinite(raw_linear) or not math.isfinite(raw_angular):
        return 0.0, 0.0
    if abs(raw_linear) < 1e-6:
        return 0.0, 0.0
    limited_linear = clamp(
        raw_linear, -abs(maximum_reverse_velocity),
        abs(maximum_forward_velocity))
    curvature = clamp(
        raw_angular / raw_linear, -abs(maximum_curvature),
        abs(maximum_curvature))
    return limited_linear, limited_linear * curvature


def conservative_curvature_limit(nominal_curvature, steering_models,
                                 minimum_observations=42,
                                 residual_limit=0.50,
                                 minimum_curvature=0.25):
    """Derive a safe shared curvature limit from all learned steering branches.

    The experiment-derived nominal limit remains the outward safety boundary.
    Runtime learning may only shrink the executable envelope when every
    forward/reverse and left/right branch has credible evidence that the
    vehicle no longer achieves that boundary.
    """
    nominal = abs(float(nominal_curvature))
    if not math.isfinite(nominal) or nominal <= 0.0:
        raise ValueError('nominal_curvature must be finite and positive')
    predictions = []
    for direction in ('forward', 'reverse'):
        for side, command in (('negative', -nominal),
                              ('positive', nominal)):
            model = (steering_models or {}).get(f'{direction}_{side}', {})
            try:
                gain = float(model['gain'])
                bias = float(model['bias'])
                observations = int(model['observations'])
                residual = float(model['residual_ema'])
            except (KeyError, TypeError, ValueError):
                return nominal
            prediction = gain * command + bias
            if (observations < minimum_observations or
                    not all(math.isfinite(value)
                            for value in (gain, bias, residual, prediction)) or
                    not 0.40 <= gain <= 1.60 or
                    not 0.0 <= residual <= residual_limit or
                    prediction * command <= 0.0):
                return nominal
            predictions.append(abs(prediction))
    learned = min(predictions)
    return clamp(min(nominal, learned),
                 min(abs(float(minimum_curvature)), nominal), nominal)


def path_curvature_floor(raw_curvature, current_path_curvature,
                         future_path_curvature, preview_confidence,
                         lateral_error, heading_error, maximum_curvature,
                         maximum_lateral_error=0.20,
                         maximum_heading_error=0.50):
    """Supply missing path feed-forward without masking MPPI recovery.

    MPPI remains authoritative when it asks for stronger steering or steering
    opposite the local path (a recovery correction). When it requests the same
    turn direction but under-commands the feasible path curvature, raise only
    the curvature magnitude to the lag-led pure-pursuit requirement.
    """
    values = (raw_curvature, current_path_curvature, future_path_curvature,
              preview_confidence, lateral_error, heading_error,
              maximum_curvature)
    if not all(math.isfinite(float(value)) for value in values):
        return float(raw_curvature), False
    raw = clamp(float(raw_curvature), -abs(maximum_curvature),
                abs(maximum_curvature))
    if (abs(lateral_error) > abs(maximum_lateral_error) or
            abs(heading_error) > abs(maximum_heading_error)):
        return raw, False
    confidence = clamp(float(preview_confidence), 0.0, 1.0)
    path = (float(current_path_curvature) + confidence *
            (float(future_path_curvature) - float(current_path_curvature)))
    path = clamp(path, -abs(maximum_curvature), abs(maximum_curvature))
    same_direction = (abs(raw) < 0.02 or raw * path > 0.0)
    if same_direction and abs(path) > abs(raw):
        return path, True
    return raw, False


def update_cusp_guard(minimum_distance, cusp_distance, entry_distance,
                      departure_distance):
    """Track approach to a cusp and detect departure on the old segment."""
    distance = float(cusp_distance)
    previous = float(minimum_distance)
    if not math.isfinite(distance):
        return previous, False
    closest = min(previous, distance)
    missed = (closest <= abs(float(entry_distance)) and
              distance >= closest + abs(float(departure_distance)))
    return closest, missed


def path_direction_runs(samples):
    """Return inclusive (start, end, direction) runs for oriented poses."""
    if len(samples) < 2:
        return []
    directions = []
    previous = 1
    for first, second in zip(samples, samples[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        projection = math.cos(first[2]) * dx + math.sin(first[2]) * dy
        direction = (previous if math.hypot(dx, dy) < 1e-6
                     else (1 if projection >= 0.0 else -1))
        directions.append(direction)
        previous = direction
    runs = []
    start = 0
    direction = directions[0]
    for index, candidate in enumerate(directions[1:], start=1):
        if candidate != direction:
            runs.append((start, index, direction))
            start = index
            direction = candidate
    runs.append((start, len(samples) - 1, direction))
    return runs


def polyline_projection(samples, x, y, yaw):
    """Return closest (distance, heading error, along-track distance)."""
    best = None
    travelled = 0.0
    for first, second in zip(samples, samples[1:]):
        vx, vy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(vx, vy)
        squared = length * length
        fraction = (0.0 if squared < 1e-9 else clamp(
            ((float(x) - first[0]) * vx +
             (float(y) - first[1]) * vy) / squared, 0.0, 1.0))
        px, py = first[0] + fraction * vx, first[1] + fraction * vy
        delta_yaw = angle_difference(second[2], first[2])
        path_yaw = first[2] + fraction * delta_yaw
        candidate = (
            math.hypot(float(x) - px, float(y) - py),
            abs(angle_difference(float(yaw), path_yaw)),
            travelled + fraction * length)
        if best is None or candidate[0] < best[0]:
            best = candidate
        travelled += length
    return best if best is not None else (math.inf, math.inf, 0.0)


TRACKABILITY_BRANCHES = (
    'forward_negative', 'forward_positive',
    'reverse_negative', 'reverse_positive')


def segment_goal_checker(requested_goal_checker, final_segment,
                         cusp_goal_checker='cusp_goal_checker',
                         default_goal_checker='goal_checker'):
    """Choose an explicit checker for a proxied FollowPath segment.

    Nav2 permits an empty FollowPath goal_checker_id only when the controller
    server has a single checker. Once the dispatcher adds its dedicated cusp
    checker, forwarding the BT's normally empty ID makes every final segment
    fail before control starts.
    """
    if not final_segment:
        return str(cusp_goal_checker)
    requested = str(requested_goal_checker or '').strip()
    return requested or str(default_goal_checker)


def segment_abort_reason(now, segment_length, last_progress_at,
                         wrong_direction_since, blocked_path_since=None,
                         no_progress_timeout=6.0,
                         wrong_direction_timeout=.75,
                         blocked_path_timeout=.75,
                         minimum_watched_length=.18):
    """Return why a committed direction segment must be replanned."""
    now = float(now)
    if (wrong_direction_since is not None and
            now - float(wrong_direction_since) >=
            float(wrong_direction_timeout)):
        return 'controller reversed inside a committed segment'
    if (blocked_path_since is not None and
            now - float(blocked_path_since) >= float(blocked_path_timeout)):
        return 'remaining committed path became invalid'
    if (float(segment_length) >= float(minimum_watched_length) and
            now - float(last_progress_at) >= float(no_progress_timeout)):
        return 'no chronological progress on committed segment'
    return ''


def gentle_motion_requested(requested_speed, segment_remaining,
                            maximum_request=.10, maximum_segment=0.0,
                            active_segment=True):
    """Preserve low-speed/short-distance intent despite an ESC deadband."""
    terminal_distance = abs(float(maximum_segment))
    return bool(active_segment) and (
        abs(float(requested_speed)) <= abs(float(maximum_request)) or
        (terminal_distance > 0.0 and
         float(segment_remaining) <= terminal_distance))


def limit_gentle_launch_effort(target_effort, learned_breakaway,
                               forward, maximum_extra=0.019):
    """Bound launch effort beyond learned breakaway for pulse-and-coast.

    Unit-agnostic ordering math; operates in the normalized effort domain
    where forward drive effort is more NEGATIVE."""
    target = float(target_effort)
    learned = float(learned_breakaway)
    extra = abs(float(maximum_extra))
    return max(learned - extra, target) if forward else min(
        learned + extra, target)


class TrackabilityEstimator:
    """Conservative per-branch learner for dynamically trackable curvature."""

    def __init__(self, prior_curvature, physical_limit,
                 minimum_curvature=0.25, state=None,
                 learned_observations_per_branch=3, session_id=None):
        self.prior = abs(float(prior_curvature))
        self.physical_limit = abs(float(physical_limit))
        self.minimum = min(abs(float(minimum_curvature)), self.prior)
        self.learned_observations = max(
            1, int(learned_observations_per_branch))
        # Session consolidation (ODAAC steal #1): a session_id is passed
        # only by the OWNING dispatcher at boot; read-only constructions
        # (launch envelope) pass None and never trigger consolidation.
        self.session_id = session_id
        self.consolidated = []
        saved = (state or {}).get('branches', {})
        new_session = (session_id is not None and
                       session_id != (state or {}).get('last_session_id'))
        self.branches = {}
        for name in TRACKABILITY_BRANCHES:
            branch = saved.get(name, {})
            try:
                estimate = float(branch.get('estimate_1pm', self.prior))
            except (TypeError, ValueError):
                estimate = self.prior
            if not math.isfinite(estimate):
                estimate = self.prior
            estimate = clamp(estimate, self.minimum, self.physical_limit)
            self.branches[name] = {
                'estimate_1pm': estimate,
                'eligible_observations': max(
                    0, int(branch.get('eligible_observations', 0))),
                'clean_passes': max(0, int(branch.get('clean_passes', 0))),
                'clean_failures': max(0, int(branch.get('clean_failures', 0))),
                'promotion_streak': max(
                    0, int(branch.get('promotion_streak', 0))),
                'failure_streak': max(0, int(branch.get('failure_streak', 0))),
                'last_demand_1pm': float(
                    branch.get('last_demand_1pm', 0.0)),
                'last_pass': bool(branch.get('last_pass', False)),
                'evidence': list(branch.get('evidence', []))[-64:],
            }
            # One session's excursion is a CANDIDATE, not identity: at the
            # first boot of a new session, an estimate whose evidence all
            # comes from a single session decays halfway back toward the
            # prior (a dead-pack afternoon taught 'radius 2.5 m' at
            # confidence 1.0 on 2026-07-14 — this gate makes that class
            # of poisoning structurally impossible). Multi-session
            # evidence persists untouched.
            if new_session:
                data = self.branches[name]
                sessions = {entry.get('session', 'legacy')
                            for entry in data['evidence']}
                if (len(sessions) < 2 and
                        abs(data['estimate_1pm'] - self.prior) > 1e-6):
                    before = data['estimate_1pm']
                    data['estimate_1pm'] = clamp(
                        self.prior + 0.5 * (before - self.prior),
                        self.minimum, self.physical_limit)
                    data['promotion_streak'] = 0
                    data['failure_streak'] = 0
                    self.consolidated.append(
                        (name, before, data['estimate_1pm']))

    @property
    def curvature_limit(self):
        return min(item['estimate_1pm'] for item in self.branches.values())

    @property
    def confidence(self):
        return min(min(
            1.0, item['eligible_observations'] / self.learned_observations)
            for item in self.branches.values())

    @property
    def source(self):
        if self.confidence >= 1.0:
            return 'learned'
        if any(item['eligible_observations'] for item in self.branches.values()):
            return 'mixed_prior_and_evidence'
        return 'bootstrap_prior'

    def observe(self, branch_name, demand_curvature, passed, eligible=True):
        """Update one branch; returns True only when its estimate changes."""
        if branch_name not in self.branches or not eligible:
            return False
        demand = clamp(abs(float(demand_curvature)),
                       self.minimum, self.physical_limit)
        branch = self.branches[branch_name]
        branch['eligible_observations'] += 1
        branch['last_demand_1pm'] = demand
        branch['last_pass'] = bool(passed)
        branch['evidence'].append({
            'demand_1pm': demand, 'passed': bool(passed),
            'session': self.session_id or 'legacy'})
        branch['evidence'] = branch['evidence'][-64:]
        before = branch['estimate_1pm']
        if passed:
            branch['clean_passes'] += 1
            branch['failure_streak'] = 0
            # A planner cannot demonstrate demand above its own limit. Clean
            # tracking near the current boundary therefore earns a small
            # exploration step for the next launch. This is deliberately much
            # slower than contraction and never exceeds the physical envelope.
            if demand >= 0.85 * before and before < self.physical_limit - 0.005:
                branch['promotion_streak'] += 1
                if branch['promotion_streak'] >= 3:
                    target = max(demand, min(
                        self.physical_limit, before + 0.10))
                    branch['estimate_1pm'] = clamp(
                        before + 0.20 * (target - before),
                        self.minimum, self.physical_limit)
                    branch['promotion_streak'] = 0
            else:
                branch['promotion_streak'] = 0
        else:
            branch['clean_failures'] += 1
            branch['promotion_streak'] = 0
            if demand <= before + 0.05:
                branch['failure_streak'] += 1
                # A single segment may be disturbed. Two clean-entry failures
                # are required, then contract faster than promotion expands.
                if branch['failure_streak'] >= 2:
                    target = max(self.minimum, 0.90 * demand)
                    branch['estimate_1pm'] = clamp(
                        0.50 * before + 0.50 * min(before, target),
                        self.minimum, self.physical_limit)
                    branch['failure_streak'] = 0
            else:
                branch['failure_streak'] = 0
        return not math.isclose(
            before, branch['estimate_1pm'], rel_tol=0.0, abs_tol=1e-12)

    def state(self):
        return {
            'version': 1,
            'prior_curvature_1pm': self.prior,
            'physical_limit_1pm': self.physical_limit,
            'planner_curvature_1pm': self.curvature_limit,
            'planner_radius_m': 1.0 / self.curvature_limit,
            'planner_source': self.source,
            'planner_confidence': self.confidence,
            'learned_observations_per_branch': self.learned_observations,
            'last_session_id': self.session_id,
            'branches': self.branches,
        }


def learned_planner_curvature(physical_curvature, utilization,
                              prior_radius, state=None):
    """Select launch curvature from persisted trackability and physical cap."""
    physical_cap = abs(float(physical_curvature)) * clamp(
        float(utilization), 0.50, 1.0)
    prior = min(physical_cap, 1.0 / abs(float(prior_radius)))
    estimator = TrackabilityEstimator(
        prior, physical_cap, state=state)
    return min(physical_cap, estimator.curvature_limit)


def scan_point_clearance(distance, scan_angle, lidar_x, lidar_y, lidar_yaw,
                         front_x, rear_x, half_width, curvature=0.0):
    """Transform one scan point and return (external, front, rear clearance).

    The clearance corridor BENDS with the commanded arc (parabolic
    small-angle approximation y = kappa*x^2/2, valid within the ~1 m
    stopping horizon). A straight corridor stopped the robot for walls
    its arc was steering AWAY from — with maneuver-tight planning the
    nose sweeps past close obstacles constantly (2026-07-15: 'forward
    obstacle stop' at 0.23 m while turning kappa 1.3 away from a wall).
    curvature=0 preserves the original straight corridor exactly."""
    base_angle = scan_angle + lidar_yaw
    x = lidar_x + distance * math.cos(base_angle)
    y = lidar_y + distance * math.sin(base_angle)
    if rear_x <= x <= front_x and abs(y) <= half_width:
        return False, None, None
    lateral = y - 0.5 * curvature * x * x
    front = x - front_x if abs(lateral) <= half_width and x > front_x else None
    rear = rear_x - x if abs(lateral) <= half_width and x < rear_x else None
    return True, front, rear


def stopping_clearance(speed, reaction_time, minimum_deceleration,
                       minimum_clearance):
    speed = abs(float(speed))
    dynamic = (speed * reaction_time +
               speed * speed / (2.0 * minimum_deceleration))
    return max(minimum_clearance, dynamic)


class DelayEstimator:
    """Evidence-weighted delay estimate with entropy-based confidence."""

    def __init__(self, candidates, forgetting_factor, minimum_observations,
                 state=None):
        self.candidates = tuple(sorted(float(x) for x in candidates))
        self.forgetting_factor = float(forgetting_factor)
        self.minimum_observations = int(minimum_observations)
        self.mse = {delay: None for delay in self.candidates}
        self.observations = 0
        if state:
            saved = state.get('mse', {})
            for delay in self.candidates:
                value = saved.get(str(delay), saved.get(delay))
                if value is not None and math.isfinite(float(value)):
                    self.mse[delay] = max(0.0, float(value))
            self.observations = max(0, int(state.get('observations', 0)))

    def update(self, squared_errors):
        alpha = 1.0 - self.forgetting_factor
        updated = False
        for delay in self.candidates:
            if delay not in squared_errors:
                continue
            error = max(0.0, float(squared_errors[delay]))
            previous = self.mse[delay]
            self.mse[delay] = (error if previous is None else
                               self.forgetting_factor * previous + alpha * error)
            updated = True
        if updated:
            self.observations += 1

    def weights(self):
        available = {key: value for key, value in self.mse.items()
                     if value is not None and math.isfinite(value)}
        if not available:
            return {}
        best = min(available.values())
        scale = max(best, 1e-9)
        effective_count = min(
            self.observations, 1.0 / (1.0 - self.forgetting_factor))
        raw = {
            delay: math.exp(clamp(
                -0.5 * effective_count * (error - best) / scale, -50.0, 0.0))
            for delay, error in available.items()}
        total = sum(raw.values())
        return {delay: value / total for delay, value in raw.items()}

    @property
    def estimate(self):
        weights = self.weights()
        if not weights:
            return 0.0
        return sum(delay * weight for delay, weight in weights.items())

    @property
    def confidence(self):
        weights = self.weights()
        if len(weights) < 2:
            return 0.0
        entropy = -sum(weight * math.log(max(weight, 1e-15))
                       for weight in weights.values())
        normalized_entropy = entropy / math.log(len(weights))
        evidence = clamp(
            self.observations / max(1, self.minimum_observations), 0.0, 1.0)
        return evidence * clamp(1.0 - normalized_entropy, 0.0, 1.0)

    def state(self):
        return {
            'observations': self.observations,
            'mse': {str(key): value for key, value in self.mse.items()},
            'weights': {str(key): value for key, value in self.weights().items()},
            'estimate_s': self.estimate,
            'confidence': self.confidence,
        }


class PathGeometry:
    """Signed path curvature and preview without crossing direction cusps."""

    def __init__(self, samples, frame_id='odom'):
        if len(samples) < 3:
            raise ValueError('a path requires at least three poses')
        self.samples = tuple((float(x), float(y), float(yaw))
                             for x, y, yaw in samples)
        self.frame_id = frame_id
        self.segment_length = []
        self.segment_direction = []
        for first, second in zip(self.samples, self.samples[1:]):
            dx, dy = second[0] - first[0], second[1] - first[1]
            length = math.hypot(dx, dy)
            projection = math.cos(first[2]) * dx + math.sin(first[2]) * dy
            self.segment_length.append(length)
            self.segment_direction.append(1 if projection >= 0.0 else -1)

    def nearest_index(self, x, y):
        return min(range(len(self.samples)),
                   key=lambda index: ((self.samples[index][0] - x) ** 2 +
                                      (self.samples[index][1] - y) ** 2))

    def nearest_index_between(self, x, y, lower, upper):
        """Nearest sample inside an inclusive, ordered path-index window."""
        lower = min(max(0, int(lower)), len(self.samples) - 1)
        upper = min(max(lower, int(upper)), len(self.samples) - 1)
        return min(range(lower, upper + 1),
                   key=lambda index: ((self.samples[index][0] - x) ** 2 +
                                      (self.samples[index][1] - y) ** 2))

    def direction_at(self, index):
        segment = min(max(0, index), len(self.segment_direction) - 1)
        return self.segment_direction[segment]

    def segment_end_index(self, index):
        """Sample index at the end of the direction segment containing index."""
        current = min(max(0, int(index)), len(self.segment_direction) - 1)
        direction = self.segment_direction[current]
        while (current < len(self.segment_direction) and
               self.segment_direction[current] == direction):
            current += 1
        return current

    def advance(self, index, distance, direction):
        travelled = 0.0
        current = min(index, len(self.samples) - 1)
        cusp_distance = math.inf
        while current < len(self.samples) - 1:
            if self.segment_direction[current] != direction:
                cusp_distance = travelled
                break
            length = self.segment_length[current]
            if travelled + length > distance:
                break
            travelled += length
            current += 1
        return current, travelled, cusp_distance

    def remaining_path(self, index):
        return sum(
            self.segment_length[min(index, len(self.segment_length)):])

    def cusp_distance(self, index, direction):
        distance = 0.0
        for segment in range(min(index, len(self.segment_length)),
                             len(self.segment_length)):
            if self.segment_direction[segment] != direction:
                return distance
            distance += self.segment_length[segment]
        return math.inf

    def curvature_at(self, index, direction, window_length):
        half = max(window_length, 1e-6) / 2.0
        lower = upper = min(max(index, 0), len(self.samples) - 1)
        distance = 0.0
        while lower > 0 and distance < half:
            segment = lower - 1
            if self.segment_direction[segment] != direction:
                break
            distance += self.segment_length[segment]
            lower -= 1
        distance = 0.0
        while upper < len(self.samples) - 1 and distance < half:
            if self.segment_direction[upper] != direction:
                break
            distance += self.segment_length[upper]
            upper += 1
        signed_distance = 0.0
        yaw_change = 0.0
        for segment in range(lower, upper):
            if self.segment_direction[segment] != direction:
                break
            signed_distance += direction * self.segment_length[segment]
            yaw_change += angle_difference(
                self.samples[segment + 1][2], self.samples[segment][2])
        if abs(signed_distance) < 1e-6:
            return 0.0
        return yaw_change / signed_distance

    def pure_pursuit_curvature(self, index, direction, lookahead_distance):
        """Kinematic pure-pursuit curvature from a path pose to its carrot."""
        carrot, _, _ = self.advance(index, lookahead_distance, direction)
        x, y, yaw = self.samples[index]
        cx, cy, _ = self.samples[carrot]
        dx, dy = cx - x, cy - y
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance_squared = dx * dx + dy * dy
        if distance_squared < 1e-9:
            return 0.0
        return 2.0 * local_y / distance_squared

    def tracking_error(self, index, x, y, yaw):
        px, py, pyaw = self.samples[index]
        dx, dy = x - px, y - py
        lateral = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        return lateral, angle_difference(yaw, pyaw), math.hypot(dx, dy)
