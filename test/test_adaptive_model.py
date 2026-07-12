import math

from ackermann_robot.adaptive_model import (
    DelayEstimator, PathGeometry, TrackabilityEstimator,
    compose_preview_curvature,
    conservative_curvature_limit,
    gentle_motion_requested, learned_planner_curvature,
    limit_ackermann_twist, limit_gentle_launch_pulse,
    path_curvature_floor, path_direction_runs, polyline_projection,
    scan_point_clearance, segment_abort_reason, segment_goal_checker,
    stopping_clearance,
    update_cusp_guard)


def credible_steering_models(gain=1.0, bias=0.0):
    return {
        f'{direction}_{side}': {
            'gain': gain, 'bias': bias, 'observations': 100,
            'residual_ema': .05,
        }
        for direction in ('forward', 'reverse')
        for side in ('negative', 'positive')
    }


def test_learned_curvature_limit_can_only_contract_shared_envelope():
    models = credible_steering_models(gain=.80)
    assert math.isclose(conservative_curvature_limit(1.15, models), .92)
    models = credible_steering_models(gain=1.20)
    assert math.isclose(conservative_curvature_limit(1.15, models), 1.15)


def test_learned_curvature_limit_requires_all_four_credible_branches():
    models = credible_steering_models(gain=.75)
    models['reverse_positive']['observations'] = 3
    assert math.isclose(conservative_curvature_limit(1.15, models), 1.15)


def test_path_curvature_floor_supplies_missing_same_sign_turn():
    command, active = path_curvature_floor(
        -.218, -1.15, -1.15, .8, .01, -.04, 1.15)
    assert active
    assert math.isclose(command, -1.15)


def test_path_curvature_floor_preserves_mppi_recovery_and_stronger_commands():
    opposite, active = path_curvature_floor(
        .40, -1.0, -1.1, .8, .02, .10, 1.15)
    assert not active and math.isclose(opposite, .40)
    stronger, active = path_curvature_floor(
        -1.10, -.8, -.9, .8, .02, .10, 1.15)
    assert not active and math.isclose(stronger, -1.10)
    far, active = path_curvature_floor(
        -.2, -1.0, -1.1, .8, .30, .10, 1.15)
    assert not active and math.isclose(far, -.2)


def test_cusp_guard_trips_only_after_close_approach_then_departure():
    closest = math.inf
    for distance in (.50, .25, .17, .11):
        closest, missed = update_cusp_guard(closest, distance, .18, .08)
        assert not missed
    closest, missed = update_cusp_guard(closest, .20, .18, .08)
    assert missed


def test_direction_runs_split_cusps_chronologically_with_shared_indices():
    samples = [(0., 0., 0.), (.2, 0., 0.), (.4, 0., 0.),
               (.2, 0., 0.), (0., 0., 0.), (.2, 0., 0.)]
    assert path_direction_runs(samples) == [
        (0, 2, 1), (2, 4, -1), (4, 5, 1)]


def test_direction_runs_reject_non_executable_input():
    assert path_direction_runs([]) == []
    assert path_direction_runs([(0., 0., 0.)]) == []


def test_polyline_projection_reports_chronological_progress():
    samples = [(0., 0., 0.), (1., 0., 0.), (1., 1., math.pi / 2)]
    distance, heading, along = polyline_projection(
        samples, 1.1, .4, math.pi / 2)
    assert math.isclose(distance, .1)
    assert math.isclose(heading, .3 * math.pi)
    assert math.isclose(along, 1.4)


def test_segment_goal_checker_never_forwards_empty_id_with_multiple_checkers():
    assert segment_goal_checker('', final_segment=True) == 'goal_checker'
    assert segment_goal_checker(
        'custom_final', final_segment=True) == 'custom_final'
    assert segment_goal_checker(
        '', final_segment=False) == 'cusp_goal_checker'


def test_segment_abort_detects_wrong_direction_and_no_progress():
    assert segment_abort_reason(10., 1., 9., 9.0) == (
        'controller reversed inside a committed segment')
    assert segment_abort_reason(10., 1., 3., None) == (
        'no chronological progress on committed segment')
    assert segment_abort_reason(10., .2, 0., None) == ''
    assert segment_abort_reason(10., 1., 9., None) == ''
    assert segment_abort_reason(10., 1., 9., None, 9.0) == (
        'remaining committed path became invalid')


def test_gentle_motion_preserves_slow_and_short_segment_intent():
    assert gentle_motion_requested(.09, 2.0)
    assert gentle_motion_requested(.30, .12)
    assert not gentle_motion_requested(.12, 2.0)
    assert not gentle_motion_requested(
        .09, .12, active_segment=False)
    assert limit_gentle_launch_pulse(1400., 1422., True) == 1416.
    assert limit_gentle_launch_pulse(1620., 1589., False) == 1595.


def test_trackability_contracts_faster_than_it_promotes():
    estimator = TrackabilityEstimator(1.0 / 1.3, .92)
    branch = 'forward_negative'
    assert not estimator.observe(branch, .70, False)
    assert estimator.observe(branch, .70, False)
    contracted = estimator.branches[branch]['estimate_1pm']
    assert contracted < 1.0 / 1.3
    assert not estimator.observe(branch, .85, True)
    assert not estimator.observe(branch, .85, True)
    assert estimator.observe(branch, .85, True)
    promoted = estimator.branches[branch]['estimate_1pm']
    assert contracted < promoted < .85


def test_trackability_explores_after_repeated_clean_boundary_segments():
    estimator = TrackabilityEstimator(1.0 / 1.3, .92)
    before = estimator.curvature_limit
    for branch in estimator.branches:
        assert not estimator.observe(branch, .75, True)
        assert not estimator.observe(branch, .75, True)
        assert estimator.observe(branch, .75, True)
    assert before < estimator.curvature_limit <= before + .021


def test_trackability_ignores_contaminated_evidence_and_round_trips():
    first = TrackabilityEstimator(1.0 / 1.3, .92)
    assert not first.observe('reverse_positive', .60, False, eligible=False)
    assert first.branches['reverse_positive']['eligible_observations'] == 0
    first.observe('reverse_positive', .65, True)
    second = TrackabilityEstimator(1.0 / 1.3, .92, state=first.state())
    assert second.state()['branches'] == first.state()['branches']


def test_trackability_labels_prior_until_every_branch_has_evidence():
    estimator = TrackabilityEstimator(1.0 / 1.3, .92)
    assert estimator.source == 'bootstrap_prior'
    for branch in estimator.branches:
        for _ in range(3):
            estimator.observe(branch, .70, True)
    assert estimator.source == 'learned'
    assert math.isclose(estimator.confidence, 1.0)


def test_learned_planner_curvature_uses_prior_then_worst_branch():
    physical = 1.15
    assert math.isclose(
        learned_planner_curvature(physical, .80, 1.30), 1.0 / 1.30)
    estimator = TrackabilityEstimator(1.0 / 1.3, physical * .80)
    estimator.branches['reverse_positive']['estimate_1pm'] = .60
    assert math.isclose(
        learned_planner_curvature(
            physical, .80, 1.30, estimator.state()), .60)


def arc_samples(radius, count=21, reverse=False):
    samples = []
    for index in range(count):
        theta = 0.5 * index / (count - 1)
        if reverse:
            samples.append((-radius * math.sin(theta),
                            radius * (1.0 - math.cos(theta)), -theta))
        else:
            samples.append((radius * math.sin(theta),
                            radius * (1.0 - math.cos(theta)), theta))
    return samples


def test_forward_arc_curvature():
    path = PathGeometry(arc_samples(2.0))
    assert path.direction_at(10) == 1
    assert math.isclose(path.curvature_at(10, 1, .5), .5, rel_tol=.03)
    assert math.isclose(path.pure_pursuit_curvature(5, 1, .5), .5,
                        rel_tol=.05)


def test_reverse_arc_has_same_geometric_curvature():
    path = PathGeometry(arc_samples(2.0, reverse=True))
    assert path.direction_at(10) == -1
    assert math.isclose(path.curvature_at(10, -1, .5), .5, rel_tol=.03)
    assert math.isclose(path.pure_pursuit_curvature(5, -1, .5), .5,
                        rel_tol=.05)


def test_preview_stops_at_direction_cusp():
    samples = [(0., 0., 0.), (.25, 0., 0.), (.5, 0., 0.),
               (.75, 0., 0.), (.5, 0., 0.), (.25, 0., 0.)]
    path = PathGeometry(samples)
    index, travelled, cusp = path.advance(1, 2.0, 1)
    assert index == 3
    assert math.isclose(travelled, .5)
    assert math.isclose(cusp, .5)


def test_segment_window_does_not_jump_across_overlapping_cusp():
    # The last reverse point overlaps the first forward point. An unrestricted
    # nearest search can jump to the later maneuver; the committed window may not.
    samples = [(0., 0., 0.), (.25, 0., 0.), (.50, 0., 0.),
               (.25, 0., 0.), (0., 0., 0.)]
    path = PathGeometry(samples)
    assert path.direction_at(0) == 1
    assert path.segment_end_index(0) == 2
    assert path.nearest_index(0., 0.) in (0, 4)
    assert path.nearest_index_between(0., 0., 0, path.segment_end_index(0)) == 0
    assert path.direction_at(path.segment_end_index(0)) == -1


def test_tracking_error_uses_path_pose_orientation():
    path = PathGeometry([(0., 0., 0.), (.5, 0., 0.), (1., 0., 0.)])
    lateral, heading, distance = path.tracking_error(1, .5, .1, .2)
    assert math.isclose(lateral, .1)
    assert math.isclose(heading, .2)
    assert math.isclose(distance, .1)


def test_delay_estimator_converges_between_candidates():
    estimator = DelayEstimator([0., .1, .2, .3, .4, .5], .98, 12)
    actual = .27
    for _ in range(120):
        estimator.update({delay: .01 + (delay - actual) ** 2
                          for delay in estimator.candidates})
    assert .24 < estimator.estimate < .31
    assert estimator.confidence > .25


def test_delay_confidence_stays_zero_without_identifying_evidence():
    estimator = DelayEstimator([0., .1, .2, .3], .98, 12)
    for _ in range(30):
        estimator.update({delay: .1 for delay in estimator.candidates})
    assert math.isclose(estimator.confidence, 0.0, abs_tol=1e-12)


def test_delay_state_round_trip():
    first = DelayEstimator([0., .1, .2], .98, 12)
    for _ in range(20):
        first.update({0.: .2, .1: .05, .2: .1})
    second = DelayEstimator([0., .1, .2], .98, 12, first.state())
    assert second.observations == first.observations
    assert math.isclose(second.estimate, first.estimate)
    assert math.isclose(second.confidence, first.confidence)


def test_preview_preserves_feedback_and_clamps():
    # RPP asks 0.6 on a 0.5 path: retain +0.1 feedback while leading to 0.8.
    assert math.isclose(compose_preview_curvature(.6, .5, .8, 1., 1.15), .9)
    assert math.isclose(compose_preview_curvature(.6, .5, .8, 0., 1.15), .6)
    assert math.isclose(compose_preview_curvature(1., .5, 1.2, 1., 1.15), 1.15)


def test_ackermann_twist_limiter_clamps_speed_and_curvature():
    linear, angular = limit_ackermann_twist(.35, 6.27, .30, .30, 1.15)
    assert math.isclose(linear, .30)
    assert math.isclose(angular, .30 * 1.15)

    linear, angular = limit_ackermann_twist(-.35, .70, .30, .30, 1.15)
    assert math.isclose(linear, -.30)
    assert math.isclose(angular, .30 * 1.15)


def test_ackermann_twist_limiter_preserves_feasible_twist_and_rejects_spin():
    assert limit_ackermann_twist(.20, -.10, .30, .30, 1.15) == (.20, -.10)
    assert limit_ackermann_twist(0.0, 2.0, .30, .30, 1.15) == (0.0, 0.0)
    assert limit_ackermann_twist(float('nan'), 0.0, .30, .30, 1.15) == (0.0, 0.0)


def test_learned_preview_reduces_known_delay_error():
    dt = .1
    delay_steps = 3
    path = [math.sin(index * .12) for index in range(150)]
    baseline_output = [0.0] * delay_steps + path[:-delay_steps]
    preview_input = path[delay_steps:] + [path[-1]] * delay_steps
    preview_output = [0.0] * delay_steps + preview_input[:-delay_steps]
    baseline_error = statistics_mean_absolute(
        expected=path[delay_steps * 2:-delay_steps],
        actual=baseline_output[delay_steps * 2:-delay_steps])
    preview_error = statistics_mean_absolute(
        expected=path[delay_steps * 2:-delay_steps],
        actual=preview_output[delay_steps * 2:-delay_steps])
    assert preview_error < baseline_error


def statistics_mean_absolute(expected, actual):
    return sum(abs(a - b) for a, b in zip(expected, actual)) / len(expected)


def test_rotated_offset_lidar_ignores_robot_and_measures_edge_clearance():
    args = (.237, 0., math.pi, .40, -.10, .16)
    # A common 0.18 m return in raw angle zero lands inside the chassis.
    assert scan_point_clearance(.18, 0., *args) == (False, None, None)
    # Raw pi points forward because the lidar frame is rotated pi.
    external, front, rear = scan_point_clearance(.40, math.pi, *args)
    assert external and math.isclose(front, .237) and rear is None
    # Raw zero at a longer range is behind the rear footprint edge.
    external, front, rear = scan_point_clearance(.50, 0., *args)
    assert external and front is None and math.isclose(rear, .163)


def test_stopping_clearance_uses_recorded_deceleration_and_minimum_guard():
    assert math.isclose(stopping_clearance(.20, .10, .45, .20), .20)
    expected = .45 * .10 + .45 ** 2 / (.90)
    assert math.isclose(stopping_clearance(.45, .10, .45, .20), expected)
