#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import rospy

from gazebo_msgs.srv import GetWorldProperties
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped


ARM_CONTROLLER = "scaled_pos_joint_traj_controller"
GRIPPER_CONTROLLER = "gripper_controller"
WRENCH_TOPIC = "/wrench/filtered"


class SmartGraspStateMachine:
    def __init__(self):
        rospy.init_node("smart_grasp_state_machine")

        self.table_side = rospy.get_param("~table_side", 0.02)
        self.table_duration = rospy.get_param("~table_duration", 40)
        self.target_force_z = rospy.get_param("~target_force_z", 5)
        self.max_retries = rospy.get_param("~max_retries", 1)

        self.state = "INIT"
        self.retry_count = 0
        self.ft_filter_proc = None

    def log_state(self, state):
        self.state = state
        rospy.loginfo("=" * 60)
        rospy.loginfo("[STATE] %s", state)
        rospy.loginfo("=" * 60)

    def wait_for_gazebo(self):
        self.log_state("CHECK_GAZEBO")
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=8.0)
            proxy = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
            resp = proxy()
            rospy.loginfo("Gazebo models: %s", resp.model_names)

            has_robot = any("robot" in name for name in resp.model_names)
            has_cube = any("cube" in name for name in resp.model_names)

            if not has_robot:
                raise RuntimeError("No robot model found in Gazebo.")
            if not has_cube:
                raise RuntimeError("No cube model found in Gazebo.")

            return True
        except Exception as err:
            rospy.logerr("Gazebo check failed: %s", err)
            return False

    def check_controllers(self):
        self.log_state("CHECK_CONTROLLERS")
        try:
            rospy.wait_for_service("/controller_manager/list_controllers", timeout=8.0)
            proxy = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
            resp = proxy()

            running = {}
            for c in resp.controller:
                running[c.name] = c.state
                rospy.loginfo("Controller: %-35s state=%s type=%s", c.name, c.state, c.type)

            if running.get(ARM_CONTROLLER) != "running":
                raise RuntimeError("%s is not running" % ARM_CONTROLLER)
            if running.get(GRIPPER_CONTROLLER) != "running":
                raise RuntimeError("%s is not running" % GRIPPER_CONTROLLER)

            return True
        except Exception as err:
            rospy.logerr("Controller check failed: %s", err)
            return False

    def ensure_wrench_filtered(self):
        self.log_state("CHECK_WRENCH")
        try:
            msg = rospy.wait_for_message(WRENCH_TOPIC, WrenchStamped, timeout=5.0)
            rospy.loginfo("FT ready: Fz=%.3f", msg.wrench.force.z)
            return True
        except Exception:
            rospy.logwarn("No %s. Starting ft_filter.py...", WRENCH_TOPIC)

        try:
            self.ft_filter_proc = subprocess.Popen(
                ["rosrun", "ur_control", "ft_filter.py", "-t", "wrench", "-ot", "wrench/filtered", "-z"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            msg = rospy.wait_for_message(WRENCH_TOPIC, WrenchStamped, timeout=10.0)
            rospy.loginfo("FT filter started: Fz=%.3f", msg.wrench.force.z)
            return True
        except Exception as err:
            rospy.logerr("FT filter failed: %s", err)
            return False

    def run_low_level_grasp_task(self, conservative=False):
        self.log_state("RUN_GRASP_TASK")

        table_side = self.table_side
        table_duration = self.table_duration
        target_force_z = self.target_force_z

        extra_params = [
            "_task_plot_dir:=/home/robot/ur3_sim/task_plots",
            "_task_save_plots:=true",
        ]

        if conservative:
            rospy.logwarn("Using conservative recovery parameters.")
            table_side = 0.015
            table_duration = 45
            target_force_z = 4
            extra_params += [
                "_slide_kp_force_z:=0.0005",
                "_slide_wrench_lpf_alpha:=0.05",
                "_slide_ft_clamp_force:=30",
            ]

        cmd = [
            "rosrun", "ur_control", "compliance_controller_example.py",
            "-f", "--sim",
            "--table-side", str(table_side),
            "--table-duration", str(table_duration),
            "--target-force-z", str(target_force_z),
        ] + extra_params

        rospy.loginfo("Executing: %s", " ".join(cmd))
        ret = subprocess.call(cmd)

        if ret == 0:
            rospy.loginfo("Low-level grasp task finished successfully.")
            return True

        rospy.logerr("Low-level grasp task failed with return code: %s", ret)
        return False

    def verify_result(self):
        self.log_state("VERIFY_RESULT")

        plot_dir = "/home/robot/ur3_sim/task_plots"
        if not os.path.isdir(plot_dir):
            rospy.logwarn("Plot dir does not exist: %s", plot_dir)
            return False

        files = sorted(os.listdir(plot_dir))
        pos_files = [f for f in files if f.endswith("_position.png")]
        force_files = [f for f in files if f.endswith("_force.png")]

        rospy.loginfo("Position plots: %s", pos_files[-3:])
        rospy.loginfo("Force plots: %s", force_files[-3:])

        if not pos_files or not force_files:
            rospy.logwarn("No task plots found.")
            return False

        return True

    def run(self):
        self.log_state("INIT")

        if not self.wait_for_gazebo():
            self.log_state("ERROR_GAZEBO")
            return False

        if not self.check_controllers():
            self.log_state("ERROR_CONTROLLERS")
            return False

        if not self.ensure_wrench_filtered():
            self.log_state("ERROR_WRENCH")
            return False

        ok = self.run_low_level_grasp_task(conservative=False)

        if not ok and self.retry_count < self.max_retries:
            self.retry_count += 1
            self.log_state("ERROR_RECOVERY")
            rospy.logwarn("Retrying task. retry_count=%d", self.retry_count)
            ok = self.run_low_level_grasp_task(conservative=True)

        if not ok:
            self.log_state("FAILED")
            return False

        if not self.verify_result():
            self.log_state("DONE_WITH_WARNINGS")
            return True

        self.log_state("DONE")
        return True


if __name__ == "__main__":
    sm = SmartGraspStateMachine()
    success = sm.run()
    sys.exit(0 if success else 1)
