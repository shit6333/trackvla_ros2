#!/usr/bin/env python3
"""One target on a loop, two distractors crossing it, and their ground truth.

The scenario demonstrates the one thing this checkpoint does reliably: once it
has locked onto a person it tends to stay with them. So the target starts close
and dead ahead, where the tracker will take it, and then walks a loop around
the room while two other people pace back and forth through its path. Nothing
here asks the instruction to choose; that was measured separately and the
checkpoint does not do it in this world.

This file is the single definition of where those people are. It emits the SDF
that Gazebo loads and answers `position_at(name, t)` for an evaluator, from the
same numbers, so the world and the measurement cannot disagree. Gazebo
publishes no pose for an actor at all, which is why ground truth has to be
computed rather than read.

Emit the world block:

    python3 sim/scripts/actor_paths.py

Ground truth from another script:

    from actor_paths import position_at, ACTORS
    x, y = position_at('person_green_shirt', sim_time)

Gazebo interpolates an actor's trajectory with a spline whose `tension` decides
how closely it follows its waypoints; at 1.0 it goes straight between them, and
only then does linear interpolation here give the exact position. Every
trajectory is emitted at tension 1.0 for that reason.
"""

import math

# Room ---------------------------------------------------------------------
#
# The Indoor Lightmap model, centred at (2, 0): x in [-2.88, 6.88] and
# y in [-4.88, 4.88]. The robot spawns at the origin facing +x with a 90 degree
# camera. Every vertex below keeps at least 0.88 m from a wall.

SPEED = 0.39        # m/s, well inside the robot's 1.0 m/s limit
STAND_TIME = 3.0    # seconds standing before setting off
TURN_TIME = 1.0     # seconds to turn in place at a vertex, so nobody walks sideways

# Skins ---------------------------------------------------------------------
#
# Only the Mingfei actor carries animation data; the other two are static
# meshes that hold a pose and slide, which is cosmetic. `z` lifts each mesh to
# put its feet on the floor (the Mingfei skin is built around the hips).
# `yaw_off` corrects for which way the author pointed the mesh's front: the two
# static skins face -y at yaw 0 and need a quarter turn added.

_MINGFEI = ('https://fuel.gazebosim.org/1.0/Mingfei/models/actor'
            '/tip/files/meshes/walk.dae')
_FEMALE = ('https://fuel.gazebosim.org/1.0/plateau/models/Casual female'
           '/1/files/meshes/casual_female.dae')
_MALE = ('https://fuel.gazebosim.org/1.0/abmohit/models/walking person'
         '/1/files/meshes/walking.dae')
# Scaled on disk, not in the SDF: <skin><scale> is ignored for this .obj (6.1
# and 24 render the same 0.29 m figure), so the vertices were multiplied by
# 6.1 into cache/ (gitignored; the texture alone is 10.7 MB). See
# scripts/fetch_sim_assets.sh.
_SPIDERMAN = '/workspace/trackvla_ros2/cache/spiderman_scaled/meshes/model.obj'

# Paths --------------------------------------------------------------------
#
# `loop` walks the vertices in order and then back to the first. `bounce`
# walks to the end and retraces. Each walker turns in place at every vertex.
#
# The target starts at (2.5, 0): 2.5 m out, bearing 0, roughly 135 pixels tall
# and the only thing in the middle of the frame. The distractors start further
# out and off-axis, in view but not dominant, and their lines cut across the
# loop so the crowding is real rather than decorative. Their periods differ
# from the target's and from each other's, so the encounters do not repeat in
# the same place every lap.

# The loop is small on purpose. A first version ran the target out to 1.4 m
# from the walls, which reads as "walking into the wall" from the robot's low
# camera and put most of the lap far from where the tracker had to be. A
# 2 x 3 m rectangle keeps everyone in the middle of the room and every vertex
# at least 2.4 m clear of a wall.
#
# `stand` is per walker and is the knob that keeps the distractors from
# walking straight through the target. Geometry alone cannot: any line that
# cuts the loop will coincide with the target somewhere within a few laps,
# because the periods are incommensurate. Delaying each distractor's start
# shifts its phase, and a small search over start delays and line positions
# found values where every pair passes at 0.5 to 1.4 m over three laps: a
# brush-past the camera can still separate, rather than two figures merging
# into one blob, which would make \"did it stay on the green shirt\"
# unanswerable.
ACTORS = {
    'person_green_shirt': dict(
        mode='loop',
        path=[(2.5, 0.0), (2.5, 1.5), (4.5, 1.5), (4.5, -1.5), (2.5, -1.5)],
        skin=_MINGFEI, z=1.0, yaw_off=0.0, animated=True,
    ),
    'person_spiderman': dict(
        mode='bounce',
        path=[(4.0, 1.6), (4.0, -1.6)], stand=9.0,
        skin=_SPIDERMAN, z=0.0, yaw_off=0.0, animated=False,
    ),
    'person_white_shirt': dict(
        mode='bounce',
        path=[(3.0, -1.9), (5.2, -1.9), (5.2, 0.6)], stand=6.0,
        skin=_MALE, z=0.0, yaw_off=math.pi / 2, animated=False,
    ),
}


def _vertices(spec):
    """Return the vertex sequence one full cycle visits, ending at the start."""
    path = list(spec['path'])
    if spec['mode'] == 'loop':
        return path + [path[0]]
    return path + path[-2::-1]          # bounce: out, then back the way it came


def waypoints(name):
    """Return the trajectory as (time, x, y, yaw) tuples, in order."""
    spec = ACTORS[name]
    verts = _vertices(spec)
    yaw_off = spec['yaw_off']

    def heading(a, b):
        return math.atan2(b[1] - a[1], b[0] - a[0]) + yaw_off

    t = 0.0
    x, y = verts[0]
    yaw = heading(verts[0], verts[1])
    stand = spec.get('stand', STAND_TIME)
    out = [(t, x, y, yaw), (t + stand, x, y, yaw)]
    t += stand
    for i in range(1, len(verts)):
        a, b = verts[i - 1], verts[i]
        t += math.dist(a, b) / SPEED
        out.append((t, b[0], b[1], heading(a, b)))          # arrive
        nxt = verts[i + 1] if i + 1 < len(verts) else verts[1]
        if nxt != b:
            t += TURN_TIME
            out.append((t, b[0], b[1], heading(b, nxt)))    # turn in place
    return out


def period(name):
    """Return the length of one loop of the script, in seconds."""
    return waypoints(name)[-1][0]


def position_at(name, t):
    """Return the walker's (x, y) at simulation time `t`.

    Matches Gazebo exactly for a trajectory emitted at tension 1.0.
    """
    wps = waypoints(name)
    t = t % period(name)
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(wps, wps[1:]):
        if t0 <= t <= t1:
            f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
    return wps[-1][1], wps[-1][2]


def _actor_sdf(name):
    """Return the <actor> block for one walker."""
    spec = ACTORS[name]
    lines = [f'    <actor name="{name}">',
             '      <skin>',
             f'        <filename>{spec["skin"]}</filename>',
             '        <scale>1.0</scale>',
             '      </skin>']
    if spec['animated']:
        lines += ['      <animation name="walk">',
                  f'        <filename>{spec["skin"]}</filename>',
                  '        <interpolate_x>true</interpolate_x>',
                  '      </animation>']
    else:
        lines += ['      <!-- No animation data in this mesh: the figure holds'
                  ' its pose and slides. -->']
    lines += ['      <script>',
              '        <loop>true</loop>',
              '        <delay_start>0.0</delay_start>',
              '        <auto_start>true</auto_start>',
              '        <trajectory id="0" type="walk" tension="1.0">']
    for t, x, y, yaw in waypoints(name):
        yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi   # keep it readable
        lines.append(f'          <waypoint><time>{t:.2f}</time>'
                     f'<pose>{x:.4f} {y:.4f} {spec["z"]:g} 0 0 {yaw:.4f}</pose>'
                     '</waypoint>')
    lines += ['        </trajectory>',
              '      </script>',
              '    </actor>']
    return '\n'.join(lines)


def world_block():
    """Return every actor, ready to paste into a world file."""
    header = (
        '    <!-- One target on a loop and two distractors crossing it,\n'
        '         generated by sim/scripts/actor_paths.py. Edit the paths\n'
        '         there and re-emit; the same module answers position_at()\n'
        '         for an evaluator, so the world and the measurement cannot\n'
        '         drift apart. -->'
    )
    return header + '\n' + '\n\n'.join(_actor_sdf(n) for n in ACTORS)


if __name__ == '__main__':
    print(world_block())
