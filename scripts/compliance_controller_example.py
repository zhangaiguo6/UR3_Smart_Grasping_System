#!/usr/bin/env python

# The MIT License (MIT)
#
# Copyright (c) 2018-2021 Cristian Beltran
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Cristian Beltran

import os
import sys
import signal
import subprocess
from ur_control import utils, traj_utils
from ur_control.hybrid_controller import ForcePositionController
from ur_control.compliance_controller import CompliantController
from ur_control.constants import GripperType
from ur_control.exceptions import InverseKinematicsException
import argparse
import rospy
import numpy as np
np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

SIM_MODE = False

# Filled during -f table slide; saved to PNG at end of task.
_task_data_logger = None


def is_sim_mode():
    if SIM_MODE:
        return True
    return rospy.has_param("use_gazebo_sim") and rospy.get_param("use_gazebo_sim")


def default_max_force_torque():
    if is_sim_mode():
        return [150.0, 150.0, 150.0, 25.0, 25.0, 25.0]
    return [50.0, 50.0, 50.0, 15.0, 15.0, 15.0]


def slide_max_force_torque():
    """Abort thresholds; Z very loose so hybrid slide is not cut off by contact spikes."""
    if is_sim_mode():
        return np.array(rospy.get_param(
            "~slide_max_force_torque",
            [100.0, 100.0, 200.0, 30.0, 30.0, 30.0],
        ))
    return np.array(rospy.get_param(
        "~slide_max_force_torque",
        [80.0, 80.0, 120.0, 25.0, 25.0, 25.0],
    ))


def filtered_wrench_topic(namespace=""):
    base = utils.solve_namespace(namespace)
    return base.rstrip("/") + "/wrench/filtered"


def ensure_ft_filter_ready(namespace="", timeout=15.0):
    """
    -f in simulation needs /wrench/filtered before Arm() subscribes.
    If missing, optionally auto-start ft_filter (default on).
    """
    if not (is_sim_mode() or (
            rospy.has_param("use_gazebo_sim") and rospy.get_param("use_gazebo_sim")
    )):
        return True

    topic = filtered_wrench_topic(namespace)
    auto_start = bool(rospy.get_param("~auto_start_ft_filter", True))

    if utils.topic_exist(topic):
        rospy.loginfo("Using filtered FT topic: {}".format(topic))
        rospy.sleep(1.5)
        return True

    if auto_start:
        rospy.logwarn("Starting ft_filter in background for sim force control...")
        subprocess.Popen(
            [
                "rosrun", "ur_control", "ft_filter.py",
                "-t", "wrench", "-ot", "wrench/filtered", "-z",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    t0 = rospy.get_time()
    while rospy.get_time() - t0 < timeout and not rospy.is_shutdown():
        if utils.topic_exist(topic):
            rospy.loginfo("FT filter ready on {}".format(topic))
            rospy.sleep(2.0)
            return True
        rospy.sleep(0.3)

    rospy.logfatal(
        "No {} after {:.0f}s.\n"
        "In another terminal (while Gazebo runs), start:\n"
        "  rosrun ur_control ft_filter.py -t wrench -ot wrench/filtered -z\n"
        "Then run -f again. Or set _auto_start_ft_filter:=true (default).".format(
            topic, timeout
        )
    )
    return False


_wrench_clamp_installed = False
_orig_get_wrench = None


_wrench_lpf_state = None


def install_sim_wrench_clamp(for_slide=False):
    """Limit FT readings so hybrid control does not explode Gazebo."""
    global _wrench_clamp_installed, _orig_get_wrench
    if not is_sim_mode():
        return
    if _wrench_clamp_installed and not for_slide:
        return
    if for_slide:
        max_f = float(rospy.get_param("~slide_ft_clamp_force", 50.0))
        max_t = float(rospy.get_param("~slide_ft_clamp_torque", 8.0))
    else:
        max_f = float(rospy.get_param("~ft_clamp_force", 60.0))
        max_t = float(rospy.get_param("~ft_clamp_torque", 12.0))
    _orig_get_wrench = arm.get_wrench

    def clamped(base_frame_control=False, hand_frame_control=False):
        w = np.array(
            _orig_get_wrench(
                base_frame_control=base_frame_control,
                hand_frame_control=hand_frame_control,
            ),
            dtype=float,
        )
        if not np.all(np.isfinite(w)):
            return np.zeros(6)
        w[:3] = np.clip(w[:3], -max_f, max_f)
        w[3:] = np.clip(w[3:], -max_t, max_t)
        return w

    arm.get_wrench = clamped
    _wrench_clamp_installed = True
    rospy.loginfo("Sim wrench clamp: |F|<={:.0f} N, |T|<={:.0f} Nm".format(max_f, max_t))


def install_slide_wrench_lowpass():
    """Extra smoothing on FT during table slide (raw sim spikes -> runaway)."""
    global _wrench_lpf_state, _orig_get_wrench
    if not is_sim_mode():
        return
    alpha = float(rospy.get_param("~slide_wrench_lpf_alpha", 0.12))
    _wrench_lpf_state = np.zeros(6)

    inner = arm.get_wrench

    def filtered(base_frame_control=False, hand_frame_control=False):
        global _wrench_lpf_state
        raw = np.array(
            inner(
                base_frame_control=base_frame_control,
                hand_frame_control=hand_frame_control,
            ),
            dtype=float,
        )
        _wrench_lpf_state = alpha * raw + (1.0 - alpha) * _wrench_lpf_state
        return _wrench_lpf_state.copy()

    arm.get_wrench = filtered
    rospy.loginfo("Slide wrench low-pass alpha={:.2f}".format(alpha))


def install_slide_wrench_z_axis_only():
    """Hybrid slide uses Z wrench for force loop; ignore XY FT spikes from drag."""
    inner = arm.get_wrench

    def z_only(base_frame_control=False, hand_frame_control=False):
        w = np.array(
            inner(
                base_frame_control=base_frame_control,
                hand_frame_control=hand_frame_control,
            ),
            dtype=float,
        )
        out = np.zeros(6, dtype=float)
        out[2] = w[2]
        return out

    arm.get_wrench = z_only
    rospy.loginfo("Slide FT: using Z axis only for force control.")


def wait_for_stable_wrench(timeout=5.0, max_force_norm=None):
    if max_force_norm is None:
        max_force_norm = float(rospy.get_param("~slide_stable_force_max", 4.0 if is_sim_mode() else 25.0))
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < timeout and not rospy.is_shutdown():
        w = arm.get_wrench(base_frame_control=True)
        fn = np.linalg.norm(w[:3])
        if np.all(np.isfinite(w)) and fn < max_force_norm:
            rospy.loginfo("Wrench stable before slide |F|={:.2f} N".format(fn))
            return True
        rospy.sleep(0.15)
    rospy.logwarn("Wrench not stable (|F| > {:.0f} N) — slide may fail".format(max_force_norm))
    return False


def hold_at_current_joints(duration=2.0):
    """Hold pose without reading FT (prevents post-grasp sim explosion)."""
    q = arm.joint_angles()
    arm.set_joint_positions(positions=q, wait=True, target_time=duration)
    rospy.sleep(0.2)


def default_target_force_z():
    """
    Z-axis target force [N] during table slide.
    Sim default is higher (8 N): 1 N is too small vs Gazebo contact / FT bias.
    Override: _target_force_z:=5  or  _sim_target_force_z:=10
    """
    if rospy.has_param("~target_force_z"):
        return float(rospy.get_param("~target_force_z"))
    if is_sim_mode():
        return float(rospy.get_param("~sim_target_force_z", 12.0))
    return 1.0


def link_attacher_ready():
    return (
        arm.gripper is not None
        and getattr(arm.gripper, "attach_srv", None) is not None
    )


def require_grasp_plugin_for_force_task():
    if not rospy.get_param("grasp_plugin", False):
        rospy.logfatal(
            "grasp_plugin is false. Restart Gazebo:\n"
            "  roslaunch ur_gripper_gazebo ur3_cubes_example.launch grasp_plugin:=1"
        )
        return False
    if not link_attacher_ready():
        rospy.logfatal(
            "link_attacher not available (attach_srv missing). "
            "Restart Gazebo with grasp_plugin:=1, then rerun -f."
        )
        return False
    return True


def default_cube_width():
    return float(rospy.get_param("~cube_width", 0.04))


def move_ee_relative_base(delta_xyz, target_time=1.5):
    """Translate EE in base_link (x, y, z)."""
    transformation = [delta_xyz[0], delta_xyz[1], delta_xyz[2], 0.0, 0.0, 0.0]
    arm.move_relative(
        target_time=target_time,
        transformation=transformation,
        relative_to_tcp=False,
        wait=True,
    )


def table_force_selection_matrix():
    """XY + orientation: position; Z: force."""
    return [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]


def default_grasp_approach_clearance():
    """Height above cube top while aligning XY (keep small)."""
    default = 0.04 if is_sim_mode() else 0.05
    return float(rospy.get_param("~grasp_approach_clearance", default))


def default_grasp_touch_offset():
    """Extra tool0 offset at grasp. Sim: no downward jam into block/table."""
    if rospy.has_param("~grasp_touch_offset"):
        return np.array(rospy.get_param("~grasp_touch_offset"), dtype=float)
    if is_sim_mode():
        return np.array([0.0, 0.0, 0.0], dtype=float)
    return np.array([0.0, 0.0, -0.012], dtype=float)


def clamp_grasp_tool0_z(tool0_xyz, cube_center):
    """Keep gripper_tip above cube bottom (avoid pressing into table in Gazebo)."""
    tool0_xyz = np.array(tool0_xyz, dtype=float)
    cube_center = np.array(cube_center, dtype=float)
    half = default_cube_width() / 2.0
    tip_off = grasp_tool0_to_tip_offset()
    tip_min_z = cube_center[2] - half + float(rospy.get_param("~grasp_cube_bottom_clearance", 0.008))
    min_tool0_z = tip_min_z - tip_off[2]
    if tool0_xyz[2] < min_tool0_z - 1e-4:
        rospy.logwarn(
            "Clamp grasp tool0 Z {:.4f} -> {:.4f} (tip above cube bottom)".format(
                tool0_xyz[2], min_tool0_z
            )
        )
        tool0_xyz = tool0_xyz.copy()
        tool0_xyz[2] = min_tool0_z
    return tool0_xyz


def grasp_pre_close_lift(move_t):
    """Slight +Z before closing to unload table contact (sim stability)."""
    lift = float(rospy.get_param("~grasp_pre_close_lift", 0.008 if is_sim_mode() else 0.0))
    if lift <= 0.0:
        return
    move_ee_relative_base([0.0, 0.0, lift], target_time=max(move_t * 0.35, 0.4))
    rospy.sleep(0.2)
    hold_at_current_joints(0.3)


def default_grasp_gripper_max_effort():
    """Low effort in sim — full 100 N crushes the block in Gazebo."""
    if rospy.has_param("~grasp_gripper_max_effort"):
        return float(rospy.get_param("~grasp_gripper_max_effort"))
    return 6.0 if is_sim_mode() else 80.0


def gripper_command_gap(gap_m, wait=True):
    """Finger gap [m] with limited max_effort (soft close)."""
    gap_m = float(np.clip(gap_m, 0.0, 0.085))
    effort = default_grasp_gripper_max_effort()
    arm.gripper._goal.command.max_effort = effort
    arm.gripper.command(gap_m, percentage=False, wait=wait)


def gentle_gripper_close_and_attach(cube_link, cube_w):
    """
    Partial close -> link_attacher -> slow final close (wide gap, low effort).
    Cube held mainly by link_attacher, not finger squeeze.
    """
    pre_close_gap = float(rospy.get_param("~grasp_pre_close_gap", cube_w + 0.014))
    attach_gap = float(rospy.get_param("~grasp_attach_gap", cube_w + 0.010))
    # Sim: keep ~20–26 mm gap — light touch only (40 mm cube).
    final_gap = float(rospy.get_param("~grasp_final_gap", 0.024 if is_sim_mode() else 0.012))
    final_gap = max(final_gap, cube_w * 0.45)
    step_sleep = float(rospy.get_param("~grasp_close_step_sleep", 0.55 if is_sim_mode() else 0.3))
    n_steps = int(rospy.get_param("~grasp_close_steps", 3 if is_sim_mode() else 2))
    effort = default_grasp_gripper_max_effort()

    rospy.loginfo(
        "Soft close: pre={:.3f} attach={:.3f} final={:.3f} m, max_effort={:.1f} N, {} steps".format(
            pre_close_gap, attach_gap, final_gap, effort, n_steps
        )
    )
    gripper_command_gap(pre_close_gap)
    rospy.sleep(step_sleep)
    hold_at_current_joints(0.25)

    gripper_command_gap(attach_gap)
    rospy.sleep(step_sleep)
    hold_at_current_joints(0.3)

    if link_attacher_ready():
        attach_parent = rospy.get_param("~attach_parent_link", "robot::wrist_3_link")
        arm.gripper.attach_link = attach_parent
        try:
            ok = arm.gripper.grab(link_name=cube_link)
            if ok:
                rospy.loginfo("Attached {} to {} (gap {:.3f} m, effort {:.1f} N).".format(
                    cube_link, attach_parent, attach_gap, effort
                ))
            else:
                rospy.logwarn("Link attacher ok=False for {}.".format(cube_link))
        except Exception as err:
            rospy.logwarn("Link attacher failed: {}".format(err))
    else:
        rospy.logwarn("grasp_plugin:=1 required for stable sim grasp.")

    gaps = list(np.linspace(attach_gap, final_gap, n_steps + 1)[1:])
    stop_ratio = float(rospy.get_param("~grasp_close_stop_ratio", 2.0 / 3.0))
    n_run = max(1, int(np.floor(len(gaps) * stop_ratio)))
    gaps = gaps[:n_run]
    last_gap = attach_gap
    for i, g in enumerate(gaps):
        gripper_command_gap(float(g))
        last_gap = float(g)
        rospy.sleep(step_sleep)
        hold_at_current_joints(0.2)
        rospy.loginfo(
            "  soft close {}/{} gap={:.3f} m (stop at {:.0f}% steps)".format(
                i + 1, len(gaps), g, stop_ratio * 100.0
            )
        )
    return last_gap


def log_ee_step(label, cube_xy=None):
    ee = arm.end_effector()
    msg = "  {} EE xyz={}".format(label, np.round(ee[:3], 4).tolist())
    if cube_xy is not None:
        xy_err = np.linalg.norm(ee[:2] - np.array(cube_xy[:2]))
        msg += ", xy err vs cube={:.4f} m".format(xy_err)
    rospy.loginfo(msg)
    return ee


# Tuned for ur_gripper_85_cubes.launch spawn (used only for orientation + IK seed).
GRASP_REF_JOINTS = [1.82225, -1.55525, 1.86741, -2.03039, -1.60938, 0.24935]


def list_gazebo_cube_models():
    """Return model names like cube, cube1, cube2, ... present in Gazebo."""
    try:
        from gazebo_msgs.srv import GetWorldProperties
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=3.0)
        props = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
        names = []
        for n in props.model_names:
            if n == "cube" or (n.startswith("cube") and len(n) > 4 and n[4:].isdigit()):
                names.append(n)
            elif n.startswith("cube"):
                names.append(n)
        return sorted(set(names))
    except Exception as err:
        rospy.logwarn("list_gazebo_cube_models: {}".format(err))
        return []


def resolve_cube_link():
    """
    Which block to grasp. Default: auto — use the only cube* model in Gazebo.
    Override: _cube_link:=cube1::link  or  _cube_model:=cube1
    """
    link = str(rospy.get_param("~cube_link", "auto"))
    if link and link != "auto":
        return link if "::" in link else link + "::link"

    model = str(rospy.get_param("~cube_model", "auto"))
    if model and model != "auto":
        return model if "::" in model else model + "::link"

    cubes = list_gazebo_cube_models()
    if len(cubes) == 1:
        rospy.loginfo("Auto-selected cube model: {}".format(cubes[0]))
        return cubes[0] + "::link"
    if len(cubes) > 1:
        rospy.logwarn(
            "Multiple cubes in Gazebo {} — using {}. Pick one: _cube_link:=cube1::link".format(
                cubes, cubes[0]
            )
        )
        return cubes[0] + "::link"
    rospy.logwarn("No cube model in Gazebo, default cube::link")
    return "cube::link"


def _cube_link_offset_local():
    """link frame origin offset from model origin (cubes_task.world: 0 0 0.02)."""
    return np.array([0.0, 0.0, float(rospy.get_param("~cube_center_z_offset", 0.02))], dtype=float)


def _pose_center_with_link_offset(position, orientation, offset_local):
    """Model pose + rotated link offset -> geometric center in same frame."""
    from tf import transformations as tft
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    rot = tft.quaternion_matrix(q)[:3, :3]
    p = np.array([position.x, position.y, position.z], dtype=float)
    return p + rot.dot(np.asarray(offset_local, dtype=float))


def _transform_point_to_base_link(point_xyz, source_frame="world", timeout=5.0):
    """Point in source_frame -> base_link via /tf."""
    import tf
    from geometry_msgs.msg import PointStamped
    listener = tf.TransformListener()
    listener.waitForTransform(
        "base_link", source_frame, rospy.Time(0), rospy.Duration(timeout)
    )
    ps = PointStamped()
    ps.header.frame_id = source_frame
    ps.header.stamp = rospy.Time(0)
    ps.point.x, ps.point.y, ps.point.z = point_xyz
    out = listener.transformPoint("base_link", ps)
    if hasattr(out, "point"):
        return np.array([out.point.x, out.point.y, out.point.z], dtype=float)
    return np.array([out.x, out.y, out.z], dtype=float)


def _cube_center_from_gazebo(cube_model, offset_local, reference_frame):
    from gazebo_msgs.srv import GetModelState
    rospy.wait_for_service("/gazebo/get_model_state", timeout=3.0)
    proxy = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    resp = proxy(cube_model, reference_frame)
    if not resp.success:
        return None
    return _pose_center_with_link_offset(resp.pose.position, resp.pose.orientation, offset_local)


def _cube_center_spawn_fallback(cube_model):
    """World pose from launch params -> base_link (matches ur_gripper_85_cubes spawn)."""
    world_xyz = rospy.get_param(
        "~cube_world_xyz",
        rospy.get_param("cube_world_xyz", rospy.get_param("cube2_world_xyz", [0.4, 0.0, 0.795])),
    )
    spawn_xyz = rospy.get_param("~robot_spawn_xyz", rospy.get_param("robot_spawn_xyz", [0.11, 0.0, 0.69]))
    spawn_yaw = float(rospy.get_param("~robot_spawn_yaw", rospy.get_param("robot_spawn_yaw", -1.5707)))
    dx = float(world_xyz[0]) - float(spawn_xyz[0])
    dy = float(world_xyz[1]) - float(spawn_xyz[1])
    dz = float(world_xyz[2]) - float(spawn_xyz[2])
    c, s = np.cos(spawn_yaw), np.sin(spawn_yaw)
    center = np.array([c * dx + s * dy, -s * dx + c * dy, dz], dtype=float)
    rospy.loginfo("Cube {} center (spawn fallback base_link): {}".format(
        cube_model, np.round(center, 4).tolist()))
    return center


def _pick_cube_center_in_base_link(sources, cube_model):
    """
    Choose cube center for FK/IK. Gazebo model pose in robot::base_link matches KDL;
    tf:world is only used when it actually transforms (not identity / missing TF).
    """
    if bool(rospy.get_param("~grasp_log_cube_pose", True)) and len(sources) > 1:
        for name, pt in sources.items():
            rospy.loginfo("Cube {} pose [{}]: {}".format(
                cube_model, name, np.round(pt, 4).tolist()))

    # Table cube in base_link is near the arm (~0.1-0.35 m Z), not world height ~0.8 m.
    max_z = float(rospy.get_param("~grasp_cube_max_base_z", 0.45))

    def sane_base_link(pt):
        pt = np.asarray(pt, dtype=float)
        return pt[2] <= max_z and np.linalg.norm(pt[:2]) < 1.2

    priority = (
        "gazebo:robot::base_link",
        "gazebo:base_link",
        "gazebo:robot",
    )
    for key in priority:
        if key in sources and sane_base_link(sources[key]):
            return sources[key], key

    world = sources.get("gazebo:world")
    tf_pt = sources.get("tf:world")
    if tf_pt is not None and world is not None:
        if np.linalg.norm(np.asarray(tf_pt) - np.asarray(world)) > 0.01 and sane_base_link(tf_pt):
            return np.asarray(tf_pt, dtype=float), "tf:world"
        rospy.logwarn(
            "Ignoring tf:world cube pose (same as world or Z too high); "
            "use Gazebo robot::base_link."
        )

    for key in priority:
        if key in sources:
            return np.asarray(sources[key], dtype=float), key

    fallback = _cube_center_spawn_fallback(cube_model)
    return fallback, "spawn_fallback"


def cube_center_in_base_link(cube_model=None):
    """Cube geometric center [x,y,z] in base_link (same frame as arm FK/IK)."""
    cube_model = cube_model or resolve_cube_link().split("::")[0]
    offset_local = _cube_link_offset_local()
    sources = {}

    if is_sim_mode():
        for ref in ("robot::base_link", "base_link", "robot"):
            try:
                center = _cube_center_from_gazebo(cube_model, offset_local, ref)
                if center is not None:
                    sources["gazebo:{}".format(ref)] = center
            except Exception as err:
                rospy.logwarn("get_model_state({} ref={}): {}".format(cube_model, ref, err))

        try:
            world_center = _cube_center_from_gazebo(cube_model, offset_local, "world")
            if world_center is not None:
                sources["gazebo:world"] = world_center
                try:
                    sources["tf:world"] = _transform_point_to_base_link(world_center, "world")
                except Exception as err:
                    rospy.logwarn("TF world->base_link: {}".format(err))
        except Exception as err:
            rospy.logwarn("get_model_state({} world): {}".format(cube_model, err))

    if sources:
        center, key = _pick_cube_center_in_base_link(sources, cube_model)
        rospy.loginfo("Cube {} center in base_link ({}): {}".format(
            cube_model, key, np.round(center, 4).tolist()))
        return center

    return _cube_center_spawn_fallback(cube_model)


def coarse_approach_above_cube(align_xy, approach_z, quat, move_t):
    """Move near cube XY at safe Z; IK jump optional, incremental path on failure."""
    ee = arm.end_effector()
    align_xy = np.array(align_xy[:2], dtype=float)
    if np.linalg.norm(ee[:2] - align_xy) <= float(rospy.get_param("~grasp_coarse_xy", 0.12)):
        return

    high_z = max(ee[2], approach_z + float(rospy.get_param("~grasp_safe_lift", 0.05)))
    high_pose = np.concatenate((np.array([align_xy[0], align_xy[1], high_z]), quat))

    if bool(rospy.get_param("~grasp_use_coarse_ik", True)):
        seeds = [arm.joint_angles(), GRASP_REF_JOINTS]
        for seed in seeds:
            try:
                q_hi = arm.inverse_kinematics(high_pose, seed=seed)
                arm.set_joint_positions(positions=q_hi, wait=True, target_time=move_t)
                log_ee_step("coarse IK above cube", align_xy)
                return
            except InverseKinematicsException:
                continue
        rospy.logwarn("Coarse IK failed for high pose, using incremental XY at z={:.3f}".format(
            high_z))

    ee = arm.end_effector()
    if ee[2] < high_z - 0.005:
        move_ee_relative_base([0.0, 0.0, high_z - ee[2]], target_time=move_t)
    n = int(rospy.get_param("~grasp_coarse_xy_steps", 4))
    for _ in range(n):
        ee = arm.end_effector()
        dx = align_xy[0] - ee[0]
        dy = align_xy[1] - ee[1]
        if abs(dx) > 0.008:
            move_ee_relative_base([dx * 0.45, 0.0, 0.0], target_time=move_t * 0.5)
        ee = arm.end_effector()
        if abs(dy) > 0.008:
            move_ee_relative_base([0.0, (align_xy[1] - ee[1]) * 0.45, 0.0], target_time=move_t * 0.5)
        if np.linalg.norm(ee[:2] - align_xy) <= float(rospy.get_param("~grasp_coarse_xy", 0.12)):
            break
    log_ee_step("incremental approach above cube", align_xy)


def grasp_tool0_to_tip_offset():
    """tool0 -> gripper_tip in base_link (use current joints after pan for accuracy)."""
    if rospy.has_param("~grasp_tcp_offset"):
        return np.array(rospy.get_param("~grasp_tcp_offset"), dtype=float)
    if bool(rospy.get_param("~grasp_use_current_tip_offset", True)):
        tip = arm.end_effector(tip_link="gripper_tip_link")
        t0 = arm.end_effector()
    else:
        tip = arm.end_effector(joint_angles=GRASP_REF_JOINTS, tip_link="gripper_tip_link")
        t0 = arm.end_effector(joint_angles=GRASP_REF_JOINTS)
    offset = tip[:3] - t0[:3]
    bias = np.array(rospy.get_param("~grasp_tip_xy_bias", [0.0, 0.0, 0.0]), dtype=float)
    offset = offset + bias
    rospy.loginfo("Grasp tool0->gripper_tip offset (base_link): {}".format(
        np.round(offset, 4).tolist()))
    return offset


def grasp_xy_alignment_error(cube_center, align_xy):
    """Metric for XY pass/fail: tip-on-cube when fine-align enabled, else tool0 target."""
    cube_xy = np.array(cube_center[:2], dtype=float)
    if bool(rospy.get_param("~grasp_fine_align_tip", True)):
        tip = arm.end_effector(tip_link="gripper_tip_link")
        return float(np.linalg.norm(tip[:2] - cube_xy)), "gripper_tip_vs_cube"
    ee = arm.end_effector()
    return float(np.linalg.norm(ee[:2] - np.array(align_xy[:2]))), "tool0_vs_target"


def fine_align_tip_xy_to_cube(cube_center, z_fixed, move_t):
    """Iterative XY correction: gripper_tip -> cube center (reduces pinch offset)."""
    cube_xy = np.array(cube_center[:2], dtype=float)
    tol = float(rospy.get_param("~grasp_tip_xy_tol", 0.006))
    iters = int(rospy.get_param("~grasp_fine_align_iters", 4))
    gain = float(rospy.get_param("~grasp_fine_align_gain", 0.85))
    for i in range(iters):
        tip = arm.end_effector(tip_link="gripper_tip_link")
        err = cube_xy - tip[:2]
        rospy.loginfo(
            "  fine align iter {}: tip_xy err={:.4f} m".format(i, np.linalg.norm(err))
        )
        if np.linalg.norm(err) < tol:
            return
        move_ee_relative_base([err[0] * gain, err[1] * gain, 0.0], target_time=move_t * 0.35)
        ee = arm.end_effector()
        if abs(ee[2] - z_fixed) > 0.004:
            move_ee_relative_base([0.0, 0.0, z_fixed - ee[2]], target_time=move_t * 0.2)


def tool0_xyz_for_cube_center(cube_center):
    """Map cube geometric center to tool0 XYZ (pinch point at gripper_tip_link)."""
    cube_center = np.array(cube_center, dtype=float)
    if bool(rospy.get_param("~grasp_align_tip", True)):
        return cube_center - grasp_tool0_to_tip_offset()
    return cube_center.copy()


def aim_base_pan_at_cube(cube_center):
    """Rotate shoulder_pan toward the cube before IK approach."""
    q = np.array(arm.joint_angles(), dtype=float)
    pan = float(np.arctan2(cube_center[1], cube_center[0]))
    pan += float(rospy.get_param("~grasp_pan_offset", 0.0))
    q[0] = pan
    arm.set_joint_positions(positions=q, wait=True, target_time=1.5)


def grasp_tool_orientation():
    """Keep gripper attitude from the original cube2 grasp joints."""
    return np.array(arm.end_effector(joint_angles=GRASP_REF_JOINTS)[3:], dtype=float)


def build_table_grasp_poses(cube_center, quat):
    """tool0 poses: gripper_tip over cube center, then descend to grasp height."""
    cube_center = np.array(cube_center, dtype=float)
    touch = default_grasp_touch_offset()
    half = default_cube_width() / 2.0
    tool0_xyz = clamp_grasp_tool0_z(tool0_xyz_for_cube_center(cube_center), cube_center)
    cube_top_z = cube_center[2] + half
    grasp_xyz = clamp_grasp_tool0_z(tool0_xyz + touch, cube_center)
    approach_z = max(
        grasp_xyz[2] + default_grasp_approach_clearance(),
        cube_top_z + default_grasp_approach_clearance(),
    )
    approach_pose = np.concatenate((
        np.array([tool0_xyz[0], tool0_xyz[1], approach_z]),
        quat,
    ))
    grasp_pose = np.concatenate((grasp_xyz, quat))
    return approach_pose, grasp_pose, grasp_xyz, tool0_xyz


def align_xy_at_height(xy_target, z_fixed, move_t):
    """Move only in X, then Y, at fixed height (no diagonal sweep over cube)."""
    xy_target = np.array(xy_target[:2], dtype=float)
    ee = arm.end_effector()
    if ee[2] < z_fixed - 0.004:
        move_ee_relative_base([0.0, 0.0, z_fixed - ee[2]], target_time=move_t)
    ee = arm.end_effector()
    dx = xy_target[0] - ee[0]
    dy = xy_target[1] - ee[1]
    if abs(dx) > 0.001:
        move_ee_relative_base([dx, 0.0, 0.0], target_time=move_t)
    ee = arm.end_effector()
    if abs(dy) > 0.001:
        move_ee_relative_base([0.0, dy, 0.0], target_time=move_t)
    ee = arm.end_effector()
    if abs(ee[2] - z_fixed) > 0.004:
        move_ee_relative_base([0.0, 0.0, z_fixed - ee[2]], target_time=move_t * 0.5)


def vertical_descend_only(z_target, descend_t):
    """Pure Z in small steps toward z_target (gripper open)."""
    n_steps = int(rospy.get_param("~grasp_descend_steps", 12))
    step_t = descend_t / float(max(n_steps, 1))
    tol = float(rospy.get_param("~grasp_z_tol", 0.003))
    for _ in range(n_steps):
        ee = arm.end_effector()
        dz = z_target - ee[2]
        if abs(dz) <= tol:
            break
        move_ee_relative_base([0.0, 0.0, dz], target_time=step_t)
        rospy.sleep(0.08)


def move_to_table_grasp(cube_center, quat, move_t, descend_t):
    """
    1) Lift at current XY (clear table)
    2) XY only → tool0 above cube (gripper_tip aligned to cube center)
    3) Z only → grasp height
    """
    approach_pose, grasp_pose, grasp_xyz, tool0_xy_target = build_table_grasp_poses(
        cube_center, quat
    )
    approach_z = approach_pose[2]
    grasp_z = grasp_pose[2]
    cube_center = np.array(cube_center, dtype=float)
    align_xy = np.array(tool0_xy_target[:2], dtype=float)

    rospy.loginfo(
        "Grasp plan: cube center {} tool0 xy ({:.3f},{:.3f}) z {:.3f}->{:.3f}".format(
            np.round(cube_center, 3).tolist(),
            align_xy[0], align_xy[1], approach_z, grasp_z,
        )
    )

    log_ee_step("start", align_xy)

    coarse_approach_above_cube(align_xy, approach_z, quat, move_t)

    # 1) Vertical lift at current XY (avoid dragging through cube at low height).
    ee = arm.end_effector()
    safe_z = max(ee[2], approach_z + float(rospy.get_param("~grasp_safe_lift", 0.03)))
    if ee[2] < safe_z - 0.005:
        move_ee_relative_base([0.0, 0.0, safe_z - ee[2]], target_time=move_t)
        log_ee_step("lift clear", align_xy)

    # 2) Horizontal only: tool0 XY over cube, then fine-tune gripper_tip on cube center.
    align_xy_at_height(align_xy, approach_z, move_t)
    if bool(rospy.get_param("~grasp_fine_align_tip", True)):
        fine_align_tip_xy_to_cube(cube_center, approach_z, move_t)
    log_ee_step("XY above cube", align_xy)

    xy_err, xy_metric = grasp_xy_alignment_error(cube_center, align_xy)
    max_xy = float(rospy.get_param("~grasp_max_xy_err", 0.018))
    if xy_err > max_xy:
        raise InverseKinematicsException(
            "Not above cube: {}={:.4f} m (max {:.4f})".format(xy_metric, xy_err, max_xy)
        )

    # 3) Vertical down to grasp height, re-check tip XY at grasp Z.
    vertical_descend_only(grasp_z, descend_t)
    if bool(rospy.get_param("~grasp_fine_align_tip", True)):
        fine_align_tip_xy_to_cube(cube_center, grasp_z, move_t * 0.5)
    log_ee_step("at grasp height", align_xy)

    ee = arm.end_effector()
    tool0_xy_err = float(np.linalg.norm(ee[:2] - align_xy))
    xy_err, xy_metric = grasp_xy_alignment_error(cube_center, align_xy)
    z_err = abs(ee[2] - grasp_xyz[2])
    if xy_err > max_xy:
        raise InverseKinematicsException(
            "After descend {}={:.4f} m".format(xy_metric, xy_err)
        )
    if bool(rospy.get_param("~grasp_log_cube_pose", True)):
        tip = arm.end_effector(tip_link="gripper_tip_link")
        tip_xy_err = float(np.linalg.norm(tip[:2] - cube_center[:2]))
        rospy.loginfo(
            "Grasp align OK: {}={:.4f} m, tool0 xy_err={:.4f} m, z_err={:.4f} m, "
            "gripper_tip xy vs cube={:.4f} m".format(
                xy_metric, xy_err, tool0_xy_err, z_err, tip_xy_err
            )
        )
    else:
        rospy.loginfo(
            "Grasp align OK: {}={:.4f} m, z_err={:.4f} m".format(xy_metric, xy_err, z_err)
        )
    return grasp_xyz


def build_xy_square_trajectory(start_pose, side, segments_per_edge=8):
    """Square path in the base_link XY plane; keep pose orientation from start."""
    start_pose = np.array(start_pose, dtype=float)
    px, py, pz = start_pose[:3]
    quat = start_pose[3:]
    half = side / 2.0
    corners = [
        (px + half, py - half, pz),
        (px + half, py + half, pz),
        (px - half, py + half, pz),
        (px - half, py - half, pz),
        (px + half, py - half, pz),
    ]
    trajectory = []
    for edge in range(4):
        a = np.array(corners[edge][:3])
        b = np.array(corners[edge + 1][:3])
        for i in range(segments_per_edge):
            ratio = float(i + 1) / float(segments_per_edge)
            xyz = a + (b - a) * ratio
            trajectory.append(np.concatenate((xyz, quat)))
    return np.array(trajectory)


def grasp_cube(cube_link=None):
    """Table-level: down to block height, XY align, short Z grasp, close, attach."""
    if arm.gripper is None:
        raise RuntimeError("Gripper not available. Launch ur_gripper_gazebo with gripper enabled.")

    cube_w = default_cube_width()
    settle_s = float(rospy.get_param("~grasp_settle_time", 1.5 if is_sim_mode() else 0.8))
    move_t = float(rospy.get_param("~grasp_move_time", 3.0))
    descend_t = float(rospy.get_param("~grasp_descend_time", 5.0))

    saved_model = getattr(arm, "model", None)
    arm.model = None

    arm.gripper.open()
    rospy.sleep(0.4)
    if hasattr(arm.gripper, "command"):
        arm.gripper.command(0.085, percentage=False, wait=True)
    rospy.sleep(0.3)

    if cube_link is None:
        cube_link = resolve_cube_link()
    rospy.loginfo("Grasp target link: {}".format(cube_link))
    cube_center = cube_center_in_base_link(cube_link.split("::")[0])
    quat = grasp_tool_orientation()

    if bool(rospy.get_param("~grasp_aim_pan", True)):
        aim_base_pan_at_cube(cube_center)

    # Optional coarse joints toward table (off by default — wrong X for current cube pose).
    if bool(rospy.get_param("~grasp_use_joint_seed", False)):
        arm.set_joint_positions(positions=GRASP_REF_JOINTS, wait=True, target_time=move_t)
        rospy.sleep(0.2)

    try:
        try:
            grasp_xyz = move_to_table_grasp(cube_center, quat, move_t, descend_t)
        except InverseKinematicsException as err:
            rospy.logerr("Grasp failed: {}".format(err))
            return False

        ee = arm.end_effector()
        tip = arm.end_effector(tip_link="gripper_tip_link")
        rospy.loginfo(
            "At grasp: tool0 z={:.4f} target z={:.4f} | tip z={:.4f} cube z={:.4f}".format(
                ee[2], grasp_xyz[2], tip[2], cube_center[2]
            )
        )

        rospy.loginfo("Gentle grasp close (no downward force — joint hold only).")
        grasp_pre_close_lift(move_t)
        hold_gap = gentle_gripper_close_and_attach(cube_link, cube_w)
        rospy.set_param("~grasp_hold_gap", hold_gap)
        rospy.sleep(settle_s)
        hold_at_current_joints(1.0)

        hold_at_current_joints(float(rospy.get_param("~post_attach_settle", 3.5)))
        for _ in range(3):
            arm.zero_ft_sensor()
            rospy.sleep(0.4)
        rospy.loginfo("Grasp done at z={:.4f}, ready for XY slide.".format(arm.end_effector()[2]))
        return True
    finally:
        arm.model = saved_model


def move_joints(wait=True):
    # desired joint configuration 'q'
    q = [1.57, -1.57, 1.36, -1.57, -1.57, 1.57]

    # go to desired joint configuration
    # in t time (seconds)
    # wait is for waiting to finish the motion before executing
    # anything else or ignore and continue with whatever is next
    arm.set_joint_positions(positions=q, wait=wait, target_time=2.0)


def spiral_trajectory():
    """
        Force/Position control. Follow a spiral trajectory on the world's YZ plan while controlling force on Z 
    """
    initial_q = [1.57, -1.57, 1.26, -1.57, -1.57, 0]

    arm.set_joint_positions(positions=initial_q, wait=True, target_time=2)

    plane = "YZ"
    radius = 0.02
    radius_direction = "+Z"
    revolutions = 3

    steps = 100 # Number of waypoints of the spiral trajectory
    duration = 30.0 # Duration of the trajectory, affects speed

    arm.zero_ft_sensor()

    initial_pose = arm.end_effector()
    trajectory = traj_utils.compute_trajectory(initial_pose, plane, radius, radius_direction,
                                               steps, revolutions, trajectory_type="spiral", from_center=True,
                                               wiggle_direction="X", wiggle_angle=np.deg2rad(0.0), wiggle_revolutions=1.0)
    execute_trajectory(trajectory, duration=duration, use_force_control=True)


def circular_trajectory():
    """
        Force/Position control. Follow a circular trajectory on the world's YZ plan while controlling force on Z 
    """
    initial_q = [1.57, -1.57, 1.26, -1.57, -1.57, 0]
    
    arm.set_joint_positions(positions=initial_q, wait=True, target_time=1)

    plane = "YZ"
    radius = 0.02
    radius_direction = "+Z"
    revolutions = 1

    steps = 100 # Number of waypoints of the circular trajectory
    duration = 30.0 # Duration of the trajectory, affects speed

    arm.zero_ft_sensor()

    initial_pose = arm.end_effector()
    trajectory = traj_utils.compute_trajectory(initial_pose, plane, radius, radius_direction,
                                               steps, revolutions, trajectory_type="circular", from_center=False,
                                               wiggle_direction="X", wiggle_angle=np.deg2rad(0.0), wiggle_revolutions=10.0)
    execute_trajectory(trajectory, duration=duration, use_force_control=True)


def execute_trajectory(trajectory, duration, use_force_control=False, termination_criteria=None):
    if use_force_control:
        pf_model = init_force_control([1., 1., 1., 1., 1., 1.])
        target_force = np.array([0., 0., default_target_force_z(), 0., 0., 0.])
        max_force_torque = np.array(default_max_force_torque())

        def termination_criteria(current_pose, standby): return False # Dummy function

        full_force_control(target_force, trajectory, pf_model, timeout=duration,
                           relative_to_ee=False, max_force_torque=max_force_torque, termination_criteria=termination_criteria)

    else:
        joint_trajectory = []
        for point in trajectory:
            try:
                joint_trajectory.append(arm.inverse_kinematics(point, verbose=False))
            except InverseKinematicsException:
                rospy.logwarn("Skipping trajectory point with no IK solution.")
        if not joint_trajectory:
            rospy.logerr("No valid IK solutions for joint trajectory.")
            return
        arm.set_joint_trajectory(trajectory=joint_trajectory, target_time=duration)


def init_force_control(selection_matrix, dt=None, table_slide=False):
    if dt is None:
        dt = 0.12 if is_sim_mode() else 0.05
    if table_slide and is_sim_mode():
        Kp = np.array([0.28, 0.28, 0.28, 0.15, 0.15, 0.15])
        kz = float(rospy.get_param("~slide_kp_force_z", 0.0012))
        Kp_force = np.array([0.0, 0.0, kz, 0.0, 0.0, 0.0])
        dt = float(rospy.get_param("~slide_control_dt", 0.1))
    elif is_sim_mode():
        Kp = np.array([0.5, 0.5, 0.5, 0.25, 0.25, 0.25])
        Kp_force = np.array([0.005, 0.005, 0.005, 0.005, 0.005, 0.005])
    else:
        Kp = np.array([2., 2., 2., 1., 1., 1.])
        Kp_force = np.array([0.02, 0.02, 0.02, 0.02, 0.02, 0.02])
    Kp_pos = Kp
    Kd_pos = Kp * 0.01
    Ki_pos = Kp * 0.0
    position_pd = utils.PID(Kp=Kp_pos, Ki=Ki_pos, Kd=Kd_pos, dynamic_pid=True)

    Kp = Kp_force
    Kp_force = Kp
    Kd_force = Kp * 0.0
    Ki_force = Kp * 0.01
    force_pd = utils.PID(Kp=Kp_force, Kd=Kd_force, Ki=Ki_force)
    pf_model = ForcePositionController(
        position_pd=position_pd, force_pd=force_pd, alpha=np.diag(selection_matrix), dt=dt)

    return pf_model


class TaskDataLogger(object):
    """
    Records desired vs actual during hybrid table slide.
    Position: tool0 XYZ in base_link (arm.end_effector(), same frame as IK).
    Wrench: 6D FT in base_link (get_wrench(base_frame_control=True)), not gripper_tip.
    """

    def __init__(self):
        self.time_s = []
        self.pos_des = []
        self.pos_act = []
        self.force_des = []
        self.force_act = []
        self._last_print = 0.0

    def record(self, t, pos_des, pos_act, force_des, force_act):
        self.time_s.append(float(t))
        self.pos_des.append(np.asarray(pos_des, dtype=float))
        self.pos_act.append(np.asarray(pos_act, dtype=float))
        self.force_des.append(np.asarray(force_des, dtype=float))
        self.force_act.append(np.asarray(force_act, dtype=float))

    def __len__(self):
        return len(self.time_s)

    def print_latest(self, interval=2.0):
        if not self.time_s:
            return
        now = rospy.get_time()
        if now - self._last_print < interval:
            return
        self._last_print = now
        i = -1
        pd, pa = self.pos_des[i], self.pos_act[i]
        fd, fa = self.force_des[i], self.force_act[i]
        rospy.loginfo(
            "Track t={:.1f}s | pos des/act xyz={} / {} err={:.4f}m | "
            "F des/act xyz={} / {} err={:.3f}N".format(
                self.time_s[i],
                np.round(pd, 4).tolist(),
                np.round(pa, 4).tolist(),
                np.linalg.norm(pd - pa),
                np.round(fd[:3], 3).tolist(),
                np.round(fa[:3], 3).tolist(),
                np.linalg.norm(fd[:3] - fa[:3]),
            )
        )

    def save_plots(self, out_dir=None, prefix="table_slide"):
        if len(self.time_s) < 2:
            rospy.logwarn("Not enough samples ({}) to plot.".format(len(self.time_s)))
            return None, None
        out_dir = out_dir or rospy.get_param(
            "~task_plot_dir",
            "/workspace/ur3_sim/task_plots",
        )
        os.makedirs(out_dir, exist_ok=True)
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError as err:
            rospy.logwarn("matplotlib not available, skip plots: %s", err)
            return None, None

        t = np.array(self.time_s)
        pos_des = np.vstack(self.pos_des)
        pos_act = np.vstack(self.pos_act)
        f_des = np.vstack(self.force_des)
        f_act = np.vstack(self.force_act)
        labels = ["X", "Y", "Z"]
        stamp = int(rospy.Time.now().to_sec())

        ee_name = getattr(arm, "ee_link", "tool0") if arm is not None else "tool0"
        fig_pos, axes_pos = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig_pos.suptitle(
            "{} position in base_link frame: desired vs actual".format(ee_name)
        )
        for i, ax in enumerate(axes_pos):
            ax.plot(t, pos_des[:, i], "b--", linewidth=1.5, label="desired")
            ax.plot(t, pos_act[:, i], "r-", linewidth=1.2, label="actual")
            ax.set_ylabel("{} [m]".format(labels[i]))
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
        axes_pos[-1].set_xlabel("time [s]")
        fig_pos.tight_layout(rect=[0, 0.02, 1, 0.96])
        pos_path = os.path.join(out_dir, "{}_{}_position.png".format(prefix, stamp))
        fig_pos.savefig(pos_path, dpi=120)
        plt.close(fig_pos)

        fig_f, axes_f = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig_f.suptitle(
            "Wrench in base_link frame (FT at wrist/tool0): desired vs actual"
        )
        for i, ax in enumerate(axes_f):
            ax.plot(t, f_des[:, i], "b--", linewidth=1.5, label="desired")
            ax.plot(t, f_act[:, i], "r-", linewidth=1.2, label="actual")
            ax.set_ylabel("F{} [N]".format(labels[i]))
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
        axes_f[-1].set_xlabel("time [s]")
        fig_f.tight_layout(rect=[0, 0.02, 1, 0.96])
        force_path = os.path.join(out_dir, "{}_{}_force.png".format(prefix, stamp))
        fig_f.savefig(force_path, dpi=120)
        plt.close(fig_f)

        rospy.loginfo("Saved position plot: {}".format(pos_path))
        rospy.loginfo("Saved force plot: {}".format(force_path))
        return pos_path, force_path


def _hybrid_step_callback(logger, print_interval):
    def _cb(t, pos_des, pos_act, force_des, force_act):
        logger.record(t, pos_des, pos_act, force_des, force_act)
        if bool(rospy.get_param("~task_print_compare", True)):
            logger.print_latest(interval=print_interval)
    return _cb


def full_force_control(
        target_force=None, target_positions=None, model=None,
        selection_matrix=[1., 1., 1., 1., 1., 1.],
        relative_to_ee=False, timeout=10.0, max_force_torque=None,
        termination_criteria=None, step_callback=None,
        time_compensation=False):
    """ 
      Use with caution!! 
      target_force: list[6], target force for each direction x,y,z,ax,ay,az
      target_position: list[7], target position for each direction x,y,z + quaternion
      selection_matrix: list[6], define which direction is controlled by position(1.0) or force(0.0)
      relative_to_ee: bool, whether to use the base_link of the robot as frame or the ee_link (+ ee_transform)
      timeout: float, duration in seconds of the force control
      termination_criteria: func, optional condition that would stop the compliance controller
    """
    arm.zero_ft_sensor()  # offset the force sensor
    arm.relative_to_ee = relative_to_ee

    if model is None:
        pf_model = init_force_control(selection_matrix)
    else:
        pf_model = model
        pf_model.selection_matrix = np.diag(selection_matrix)

    if max_force_torque is None:
        max_force_torque = default_max_force_torque()
    max_force_torque = np.array(max_force_torque)

    target_force = np.array([0., 0., 0., 0., 0., 0.]
                            ) if target_force is None else target_force

    target_positions = arm.end_effector(
    ) if target_positions is None else np.array(target_positions)

    pf_model.set_goals(force=target_force)
    arm.model = pf_model

    n_wp = len(target_positions) if getattr(target_positions, "ndim", 0) > 1 else 1
    rospy.loginfo(
        "Hybrid slide: {} waypoints, {:.1f}s, ~{:.2f}s/pt, Fz_des={:.2f} N".format(
            n_wp, timeout, timeout / float(max(n_wp, 1)),
            float(target_force[2]) if target_force is not None else 0.0,
        )
    )
    return arm.set_hybrid_control_trajectory(
        target_positions,
        max_force_torque=max_force_torque,
        timeout=timeout,
        stop_on_target_force=False,
        termination_criteria=termination_criteria,
        step_callback=step_callback,
        time_compensation=time_compensation,
    )


def execute_table_xy_slide(slide_pose, table_side, duration, fz):
    """
    Hybrid table slide: XY position + Z force (sim default Fz=8 N, tunable).
    Optional: _slide_use_joint_traj:=true for joint-only (no FT).
    """
    global _task_data_logger
    segments = int(rospy.get_param("~table_segments_per_edge", 6))
    trajectory = build_xy_square_trajectory(
        slide_pose, table_side, segments_per_edge=segments
    )

    if bool(rospy.get_param("~slide_use_joint_traj", False)):
        rospy.logwarn("Joint slide (no Z force control). Set _slide_use_joint_traj:=false for hybrid.")
        hold_at_current_joints(float(rospy.get_param("~pre_slide_settle", 2.0)))
        _task_data_logger = None
        return execute_table_xy_slide_joint(trajectory, duration)

    selection_matrix = table_force_selection_matrix()
    target_force = np.array([0.0, 0.0, fz, 0.0, 0.0, 0.0])

    hold_at_current_joints(float(rospy.get_param("~pre_slide_settle", 3.5)))
    for _ in range(int(rospy.get_param("~slide_zero_ft_count", 4))):
        arm.zero_ft_sensor()
        rospy.sleep(0.6)
    if not wait_for_stable_wrench(timeout=10.0):
        rospy.logwarn("FT still noisy — continuing with clamp+LPF.")
    install_sim_wrench_clamp(for_slide=True)
    install_slide_wrench_lowpass()
    install_slide_wrench_z_axis_only()

    pf_model = init_force_control(selection_matrix, table_slide=True)
    rospy.loginfo(
        "Table slide hybrid: {} pts, Fz={:.2f} N (sim default 8 N), z_ref={:.4f}, side={:.3f}m".format(
            len(trajectory), fz, slide_pose[2], table_side
        )
    )

    logger = None
    step_cb = None
    if bool(rospy.get_param("~task_save_plots", True)):
        logger = TaskDataLogger()
        _task_data_logger = logger
        interval = float(rospy.get_param("~task_print_interval", 2.0))
        step_cb = _hybrid_step_callback(logger, interval)

    result = full_force_control(
        target_force,
        target_positions=trajectory,
        model=pf_model,
        selection_matrix=selection_matrix,
        timeout=duration,
        max_force_torque=slide_max_force_torque(),
        step_callback=step_cb,
        time_compensation=False,
    )

    if logger is not None and len(logger) > 0:
        logger.save_plots()
    return result


def execute_table_xy_slide_joint(trajectory, duration):
    """Joint-space square path (no FT); use only if hybrid Fz is not needed."""
    joint_traj = []
    seed = arm.joint_angles()
    for point in trajectory:
        try:
            seed = arm.inverse_kinematics(point, seed=seed, verbose=False)
            joint_traj.append(seed)
        except InverseKinematicsException:
            rospy.logwarn("Skip slide waypoint with no IK.")
    if len(joint_traj) < 2:
        rospy.logerr("Slide joint trajectory too short.")
        return False
    rospy.loginfo("Joint slide: {} points, {:.1f}s".format(len(joint_traj), duration))
    arm.set_joint_trajectory(trajectory=joint_traj, target_time=duration)
    return True


def force_control():
    """
    1) 降到桌面高度，XY 对准方块，短距离下抓
    2) 桌面 XY 方形滑动（位置）
    3) Z 向恒力（仿真默认 8 N，可 _target_force_z:= 或 _sim_target_force_z:=）
    """
    if not require_grasp_plugin_for_force_task():
        return

    cube_link = resolve_cube_link()
    table_side = float(rospy.get_param("~table_side", 0.06 if is_sim_mode() else 0.04))
    table_duration = float(rospy.get_param("~table_duration", 40.0 if is_sim_mode() else 25.0))
    fz = default_target_force_z()

    rospy.loginfo(
        "Task: (1) table grasp (2) XY drag z fixed (3) Fz={:.2f} N | cube={} side={:.3f}m".format(
            fz, cube_link, table_side
        )
    )

    if not grasp_cube(cube_link=cube_link):
        rospy.logfatal("Grasp phase failed — fix alignment before slide.")
        return
    if not link_attacher_ready():
        rospy.logfatal("link_attacher missing — restart with grasp_plugin:=1")
        return

    hold_gap = float(rospy.get_param("~grasp_hold_gap", 0.033 if is_sim_mode() else 0.012))
    gripper_command_gap(hold_gap)
    hold_at_current_joints(float(rospy.get_param("~post_grasp_settle", 2.0)))

    slide_pose = np.array(arm.end_effector(), dtype=float)
    execute_table_xy_slide(slide_pose, table_side, table_duration, fz)
    if _task_data_logger is not None and len(_task_data_logger) > 0:
        rospy.loginfo("Task logging: {} samples recorded.".format(len(_task_data_logger)))


def main():
    """ Main function to be run. """
    parser = argparse.ArgumentParser(description='Test force control')
    parser.add_argument('-m', '--move', action='store_true',
                        help='move to joint configuration')
    parser.add_argument('-f', '--force', action='store_true',
                        help='Grasp cube then table XY move with constant Z force')
    parser.add_argument('--circle', action='store_true',
                        help='Circular rotation around a target pose')
    parser.add_argument('--spiral', action='store_true',
                        help='Spiral rotation around a target pose')
    parser.add_argument('--namespace', type=str, 
                        help='Namespace of arm', default=None)
    parser.add_argument('--sim', action='store_true',
                        help='Use softer limits for Gazebo (also auto-detected via use_gazebo_sim)')
    parser.add_argument('--target-force-z', type=float, default=None,
                        help='Z target force [N]; sim default 8 N via _sim_target_force_z')
    parser.add_argument('--table-side', type=float, default=None,
                        help='Table square side [m] (also: _table_side:=0.04)')
    parser.add_argument('--table-duration', type=float, default=None,
                        help='Table path duration [s] (also: _table_duration:=40)')
    parser.add_argument('--grasp-descend-z', type=float, default=None,
                        help='Descend along -Z before closing [m] (also: _grasp_descend_z:=0.045)')
    parser.add_argument('--cube-link', type=str, default=None,
                        help='Gazebo link to attach, e.g. cube::link or cube1::link')
    parser.add_argument('--cube-model', type=str, default=None,
                        help='Gazebo model name, e.g. cube or cube1 (auto if omitted)')
    # Allow ROS private params such as _target_force_z:=2.0 after -f.
    args, _unknown = parser.parse_known_args()

    rospy.init_node('ur3e_compliance_control')
    if args.target_force_z is not None:
        rospy.set_param('~target_force_z', args.target_force_z)
    if args.table_side is not None:
        rospy.set_param('~table_side', args.table_side)
    if args.table_duration is not None:
        rospy.set_param('~table_duration', args.table_duration)
    if args.grasp_descend_z is not None:
        rospy.set_param('~grasp_descend_z', args.grasp_descend_z)
    if args.cube_link is not None:
        rospy.set_param('~cube_link', args.cube_link)
    if args.cube_model is not None:
        rospy.set_param('~cube_model', args.cube_model)
    global SIM_MODE
    SIM_MODE = args.sim

    ns = ''
    joints_prefix = None
    tcp_link = None

    if args.namespace:
        ns = args.namespace
        joints_prefix = args.namespace + '_'

    if args.force:
        if not ensure_ft_filter_ready(namespace=ns):
            return

    global arm
    default_model = init_force_control([1., 1., 1., 1., 1., 1.])
    arm = CompliantController(
        model=default_model,
        namespace=ns,
        joint_names_prefix=joints_prefix,
        gripper_type=GripperType.GENERIC,
    )

    if args.move:
        move_joints()
    if args.circle:
        circular_trajectory()
    if args.spiral:
        spiral_trajectory()
    if args.force:
        force_control()


if __name__ == "__main__":
    main()
