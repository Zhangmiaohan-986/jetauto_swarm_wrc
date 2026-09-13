#!/usr/bin/env python
"""ROS-free, finite route selection and progress-gated transition reservations.

The old whole-route planner is retained as a feasible seed, not copied. Ordered
seed routes are compiled into local dependencies. A robot may use every cleared
prefix without waiting for the preceding robot's final slot. No clock releases a
reservation. Guarantees remain conditional on the configured tracking envelope.
"""
from __future__ import print_function
import bisect
import math
from fleet_geometry import distance, segment_distance, assign_slots, plan_transition


def lengths(path):
    result = [0.0]
    for a, b in zip(path, path[1:]):
        # Stable generated YAML across Windows Python3 and VM Python2/3 libm.
        result.append(round(result[-1] + distance(a, b), 8))
    return result


def densify(path, step):
    points, corners = [list(path[0])], []
    for a, b in zip(path, path[1:]):
        count = max(1, int(math.ceil(distance(a, b) / step)))
        points.extend([[round(a[k] + (b[k]-a[k])*i/count, 8) for k in (0, 1)]
                       for i in range(1, count+1)])
        corners.append(len(points)-1)
    return points, corners


def compile_gates(tasks, reservation, step, margin):
    result = []
    for seed in tasks:
        points, corners = densify(seed['path'], step)
        task = dict(seed, path=points, corners=corners, arc=lengths(points),
                    gates=[{} for _ in points], after=[])
        for k, (a, b) in enumerate(zip(points, points[1:]), 1):
            for earlier in result:
                last = 0
                for j, (c, d) in enumerate(zip(earlier['path'], earlier['path'][1:]), 1):
                    if segment_distance(a, b, c, d) < reservation:
                        last = j
                if last:
                    # Reaching a route's last milestone means ARRIVED, not merely
                    # projecting near its endpoint. Other milestones include lag.
                    task['gates'][k][earlier['robot']] = min(
                        earlier['arc'][-1], round(earlier['arc'][last] + margin, 8))
        result.append(task)
    return result


def estimate(tasks):
    """Distance-equivalent critical path; ranking proxy, NOT measured seconds."""
    clocks = {}
    for task in tasks:
        times = [0.0]
        for i in range(1, len(task['path'])):
            ready = times[-1]
            for other, milestone in task['gates'][i].items():
                arc, prior = clocks[other]
                ready = max(ready, prior[min(bisect.bisect_left(arc, milestone), len(prior)-1)])
            times.append(ready + task['arc'][i]-task['arc'][i-1])
        clocks[task['robot']] = (task['arc'], times)
    return max(v[1][-1] for v in clocks.values())


def map_assignment(grid, names, starts, slots):
    """Bottleneck map distance first, minimum sum under that bound second."""
    n = len(names)
    costs = []
    for name in names:
        row = []
        for slot in slots:
            route = grid.plan(starts[name], slot)
            row.append(lengths(route)[-1] if route else float('inf'))
        costs.append(row)
    best = {0: 0.0}
    for row in costs:
        following = {}
        for mask, value in best.items():
            for j, cost in enumerate(row):
                if not mask & (1 << j) and not math.isinf(cost):
                    key = mask | (1 << j)
                    following[key] = min(following.get(key, float('inf')), max(value, cost))
        best = following
    bound = best.get((1 << n)-1)
    if bound is None:
        raise ValueError('No map-connected perfect slot assignment')
    best = {0: (0.0, [])}
    for row in costs:
        following = {}
        for mask, (value, chosen) in best.items():
            for j, cost in enumerate(row):
                if not mask & (1 << j) and cost <= bound + 1e-9:
                    key = mask | (1 << j)
                    candidate = (value+cost, chosen+[j])
                    if candidate < following.get(key, (float('inf'), [])):
                        following[key] = candidate
        best = following
    return dict((name, slots[j]) for name, j in zip(names, best[(1 << n)-1][1]))


def plan_gated_transition(grid, starts, slots, planning, seed_goals=None, fixed=False):
    names = sorted(starts)
    assignments = [seed_goals] if fixed else [assign_slots(names, starts, slots),
                                             map_assignment(grid, names, starts, slots)]
    if seed_goals is not None and seed_goals not in assignments:
        assignments.append(seed_goals)
    selected, failures, seen = None, [], set()
    # ponytail: at most nine geometric candidates, not exhaustive TAPF/CBS. If
    # none is feasible, stop with diagnostics; never loop/replan indefinitely.
    for goals in assignments:
        identity = tuple(tuple(goals[n]) for n in names)
        if identity in seen:
            continue
        seen.add(identity)
        for order in ('shortest', 'front_first', 'rear_first'):
            try:
                seed = plan_transition(grid, starts, goals, planning['parked_clearance'],
                                       planning['route_reservation'], order)
            except ValueError as exc:
                failures.append(str(exc))
                continue
            tasks = compile_gates(seed, planning['route_reservation'],
                                  planning['gate_step'], planning['release_margin'])
            score = (estimate(tasks), max(t['length'] for t in tasks),
                     sum(t['length'] for t in tasks))
            if selected is None or score < selected[0]:
                selected = (score, goals, tasks, order)
    if selected is None:
        raise ValueError('No gated transition candidate; '+str(failures[-2:]))
    score, goals, tasks, order = selected
    return goals, tasks, {'scheduler': 'progress_gates', 'seed_order': order,
                          'assignment_candidates': len(seen),
                          'critical_path_distance': round(score[0], 3),
                          'total_route_length': round(score[2], 3)}


def permitted_target(task, cleared, arrived, progress):
    """Furthest permitted prefix, capped at the next original path corner."""
    corner = next((i for i in task['corners'] if task['arc'][i] > progress+0.045),
                  task['corners'][-1])
    result = 0
    for i in range(1, corner+1):
        if task['arc'][i] <= progress:
            result = i
            continue
        if any(other not in arrived and cleared.get(other, 0.0)+1e-8 < value
               for other, value in task['gates'][i].items()):
            break
        result = i
    return result


def project_progress(task, pose, previous, committed, retreat):
    """Project only onto the committed prefix and a local monotonic window."""
    candidates = []
    for i in range(1, committed+1):
        if task['arc'][i] < previous-retreat:
            continue
        a, b = task['path'][i-1:i+1]
        dx, dy = b[0]-a[0], b[1]-a[1]
        ratio = max(0., min(1., ((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/max(dx*dx+dy*dy, 1e-12)))
        point = (a[0]+ratio*dx, a[1]+ratio*dy)
        value = task['arc'][i-1]+ratio*(task['arc'][i]-task['arc'][i-1])
        candidates.append((distance(pose, point), value))
    return min(candidates) if candidates else (distance(pose, task['path'][0]), 0.0)


def remaining_route(task, pose, progress, target):
    index = bisect.bisect_right(task['arc'], progress+1e-8)
    return [list(pose[:2])] + task['path'][index:target+1] if index <= target else [list(pose[:2])]*2


def validate_gates(tasks, goals):
    seen = {}
    for task in tasks:
        name = task['robot']
        if name in seen or name not in goals:
            raise ValueError('Duplicate/unknown gated robot')
        points, arc, gates = task['path'], task['arc'], task['gates']
        if len(points) < 2 or len(points) != len(arc) or len(points) != len(gates):
            raise ValueError('Gated path/arc/dependency sizes differ')
        if any(len(p) != 2 or any(math.isnan(v) or math.isinf(v) for v in p) for p in points):
            raise ValueError('Non-finite gated path')
        if any(abs(a-b)>1e-6 for a, b in zip(arc, lengths(points))):
            raise ValueError('Gated path arc is inconsistent')
        if any(b <= a for a, b in zip(arc, arc[1:])):
            raise ValueError('Gated path has zero-length edges')
        if not task['corners'] or task['corners'][-1] != len(points)-1 or any(
                not isinstance(i, int) or i <= 0 or i >= len(points) for i in task['corners']):
            raise ValueError('Invalid gated corner index')
        for gate in gates:
            for other, value in gate.items():
                if other not in seen or not 0 < value <= seen[other]:
                    raise ValueError('Cyclic/invalid gated milestone')
        goal = goals[name]
        if distance(points[-1], (goal['x'], goal['y'])) > 0.001:
            raise ValueError('Gated endpoint differs from slot')
        seen[name] = arc[-1]
    if set(seen) != set(goals):
        raise ValueError('Gated tasks do not cover slots')
