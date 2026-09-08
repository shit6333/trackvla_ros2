"""Render two Habitat humanoids side by side from a Spot-jaw camera.

Produces the single opening frame the first-frame selection experiment feeds
to the planner, plus a JSON sidecar with where each person is (lane, bearing,
distance) so the trajectory's direction can be scored against ground truth.

Poses are applied exactly the way habitat-lab's KinematicHumanoid and
HumanoidRearrangeController do it, without importing habitat-lab (which needs
hydra, absent in this env). Three details that were wrong in earlier attempts:

  * load from the URDF with maintain_link_order=True; the .ao_config template
    path reorders links and a 54x4 clip frame lands on the wrong joints;
  * compose the root as (base @ offset_transform_base^-1) @ offset, with
    offset = rot_y(pi) @ transform_array[k] (z travel zeroed) and the feet at
    base.y - 0.9;
  * place the AGENT, not the sensor. The sensor is a child of the agent node,
    which in a loaded scene has a default transform of its own; writing the
    sensor's transformation only sets a local offset on top of that.

Run inside omtrackvla-dev under the omtrack env, from /workspace/OmTrackVLA:

    python /tmp/render_pairs.py --left female_31 --right male_36 --out /tmp/pairs/blank
    python /tmp/render_pairs.py --left female_31 --right male_36 --out /tmp/pairs/scene \
        --scene data/scene_datasets/hm3d/val/00892-bzCsHPLDztK/bzCsHPLDztK.basis.glb \
        --cam-pos 1.8229 2.5823 4.7063 --cam-yaw 0.5240

Camera: 384x384, hfov 90, level, EYE_H above the agent's feet. --cam-pos is a
navmesh (floor) point, e.g. an episode's start_position, and --cam-yaw its
rotation about +Y. People are placed relative to the camera: x to its right,
DIST ahead. Without --scene the agent sits at the origin on a blank background.
"""
import argparse
import json
import os
import pickle

import habitat_sim
import magnum as mn
import numpy as np
from PIL import Image

ROOT = '/workspace/OmTrackVLA/data/humanoids/humanoid_data'
EYE_H = 0.65        # Spot jaw camera above the floor
BASE_Y = 0.9        # humanoid root above its feet (KinematicHumanoid base_offset)

OFFSET_TRANSFORM_BASE = (mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(0, 0, 1.0))
                         @ mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(0, 1.0, 0)))
ROT_X_NEG90 = mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(1, 0, 0))
ROT_Y_180 = mn.Matrix4.rotation(mn.Rad(np.pi), mn.Vector3(0, 1.0, 0))


def parse():
    p = argparse.ArgumentParser()
    p.add_argument('--left', required=True)
    p.add_argument('--right', required=True)
    p.add_argument('--out', required=True, help='output prefix; writes <out>.png and <out>.json')
    p.add_argument('--lane', type=float, default=0.9, help='lateral offset of each person, m')
    p.add_argument('--dist', type=float, default=2.5, help='distance ahead of the camera, m')
    p.add_argument('--facing', choices=('away', 'toward'), default='away')
    p.add_argument('--pose', choices=('walk', 'stand'), default='walk')
    p.add_argument('--walk-frame', type=int, default=40)
    p.add_argument('--scene', default=None, help='HM3D/HSSD .glb; omit for a blank background')
    p.add_argument('--cam-pos', type=float, nargs=3, default=None, help='agent x y z, a floor point')
    p.add_argument('--cam-yaw', type=float, default=0.0, help='agent yaw about +Y, radians')
    return p.parse_args()


def load_motion(ident):
    pk = pickle.load(open('%s/%s/%s_motion_data_smplx.pkl' % (ROOT, ident, ident), 'rb'))
    return pk['walk_motion'], pk['stop_pose']


def make_sim(scene):
    cfg = habitat_sim.SimulatorConfiguration()
    cfg.scene_id = scene if scene else 'NONE'
    cfg.enable_physics = True
    cfg.scene_light_setup = habitat_sim.gfx.DEFAULT_LIGHTING_KEY
    specs = []
    for uuid, stype in (('color', habitat_sim.SensorType.COLOR), ('depth', habitat_sim.SensorType.DEPTH)):
        s = habitat_sim.CameraSensorSpec()
        s.uuid = uuid
        s.sensor_type = stype
        s.resolution = [384, 384]
        s.hfov = 90
        s.position = [0.0, EYE_H, 0.0]       # level, EYE_H above the agent's feet
        specs.append(s)
    ag = habitat_sim.agent.AgentConfiguration()
    ag.sensor_specifications = specs
    return habitat_sim.Simulator(habitat_sim.Configuration(cfg, [ag]))


def add(om, ident):
    urdf = '%s/%s/%s.urdf' % (ROOT, ident, ident)
    h = om.add_articulated_object_from_urdf(
        urdf, fixed_base=False, maintain_link_order=True,
        light_setup_key=habitat_sim.gfx.DEFAULT_LIGHTING_KEY)
    if h is None:
        raise RuntimeError('failed to load ' + urdf)
    h.motion_type = habitat_sim.physics.MotionType.KINEMATIC
    return h


def place(h, joints_54x4, clip_root_4x4, world_xz, floor_y, fwd_xz, walking):
    """Pose and place one humanoid; fwd_xz is the planar direction it faces."""
    h.joint_positions = np.asarray(joints_54x4, dtype=np.float32).reshape(-1).tolist()
    if walking:
        offset = ROT_Y_180 @ mn.Matrix4(np.asarray(clip_root_4x4, dtype=np.float32))
        t = offset.translation
        offset.translation = mn.Vector3(t.x, t.y, 0.0)
    else:
        offset = mn.Matrix4()
    f = mn.Vector3(fwd_xz[0], 0.0, fwd_xz[1]).normalized()
    fwd_norm = mn.Vector3(f.z, f.y, -f.x)
    p = mn.Vector3(world_xz[0], floor_y + BASE_Y, world_xz[1])
    base = mn.Matrix4.look_at(p, p + fwd_norm, mn.Vector3.y_axis()) @ ROT_X_NEG90
    h.transformation = (base @ OFFSET_TRANSFORM_BASE.inverted()) @ offset


def main():
    a = parse()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or '.', exist_ok=True)

    sim = make_sim(a.scene)
    om = sim.get_articulated_object_manager()
    agent = sim.get_agent(0)

    cx, cy, cz = a.cam_pos if a.cam_pos is not None else (0.0, 0.0, 0.0)
    state = habitat_sim.AgentState()
    state.position = np.array([cx, cy, cz])
    state.rotation = np.quaternion(np.cos(a.cam_yaw / 2), 0.0, np.sin(a.cam_yaw / 2), 0.0)
    agent.set_state(state)
    rot = mn.Matrix4.rotation(mn.Rad(a.cam_yaw), mn.Vector3(0, 1.0, 0))

    def to_world(local_x, local_z):
        v = rot.transform_vector(mn.Vector3(local_x, 0.0, local_z))
        return (cx + v.x, cz + v.z)

    fwd_local = (0.0, -1.0) if a.facing == 'away' else (0.0, 1.0)
    fv = rot.transform_vector(mn.Vector3(fwd_local[0], 0.0, fwd_local[1]))

    people = {}
    for side, ident, lane in (('left', a.left, -a.lane), ('right', a.right, +a.lane)):
        h = add(om, ident)
        walk, stop = load_motion(ident)
        if a.pose == 'walk':
            joints, root = walk['joints_array'][a.walk_frame], walk['transform_array'][a.walk_frame]
        else:
            joints, root = stop['joints'], stop['transform']
        wx, wz = to_world(lane, -a.dist)
        place(h, joints, root, (wx, wz), cy, (fv.x, fv.z), a.pose == 'walk')
        # Bearing in the robot's convention: positive to the left of the axis.
        bearing = float(np.degrees(np.arctan2(-lane, a.dist)))
        people[side] = dict(identity=ident, lane_m=lane, dist_m=a.dist,
                            bearing_deg=bearing, world_xz=[wx, wz])

    obs = sim.get_sensor_observations()
    rgb = np.asarray(Image.fromarray(obs['color'], 'RGBA').convert('RGB'))
    Image.fromarray(rgb).save(a.out + '.png')
    d = obs['depth']
    fg = np.isfinite(d) & (d > 0) & (d < 100)
    meta = dict(
        left=people['left'], right=people['right'],
        facing=a.facing, pose=a.pose, walk_frame=a.walk_frame,
        scene=a.scene, cam_pos=[cx, cy + EYE_H, cz], cam_yaw=a.cam_yaw,
        image=dict(width=384, height=384, hfov_deg=90),
        valid_depth_fraction=float(fg.mean()),
    )
    json.dump(meta, open(a.out + '.json', 'w'), indent=1)
    print('wrote %s.png  valid depth %.0f%%  median %.2f m' % (
        a.out, 100 * fg.mean(), np.median(d[fg]) if fg.any() else -1))
    sim.close()


if __name__ == '__main__':
    main()
