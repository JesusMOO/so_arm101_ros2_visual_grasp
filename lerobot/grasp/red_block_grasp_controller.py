from dataclasses import dataclass

from lerobot.grasp.grasp_joint_targets import HOME_JOINT_POSITIONS, READY_JOINT_POSITIONS
from lerobot.grasp.grasp_pose_planner import build_grasp_plan
from lerobot.kinematics.so101_ik import solve_position_ik
from lerobot.kinematics.so101_model import load_so101_model
from lerobot.so101.arm_backend import DEFAULT_ACC, DEFAULT_SPEED,CLOSED_GRIPPER_RAD
from lerobot.so101.joint_mapping import JOINT_NAMES
from lerobot.vision.camera_geometry import block_location_to_base_point


@dataclass(frozen=True)
class GraspExecutionResult:
    success: bool
    message: str
    states: list


class RedBlockGraspController:
    def __init__(
        self,
        backend,
        model=None,
        block_point_provider=block_location_to_base_point,
        pose_planner=build_grasp_plan,
        ik_solver=solve_position_ik,
        state_callback=None,
        speed=DEFAULT_SPEED,
        acc=DEFAULT_ACC,
        wait=True,
        fixed_grasp_wrist_roll_rad=0.0,
    ):
        self.backend = backend
        self.model = model if model is not None else load_so101_model()
        self.block_point_provider = block_point_provider
        self.pose_planner = pose_planner
        self.ik_solver = ik_solver
        self.state_callback = state_callback
        self.speed = speed
        self.acc = acc
        self.wait = wait
        self.fixed_grasp_wrist_roll_rad = float(fixed_grasp_wrist_roll_rad)
        self.states = []

    def execute_pick(self):
        self.states = []
        try:
            self._set_state("WAIT_FOR_BLOCK")
            block_base_point = self.block_point_provider()

            self._set_state("MOVE_READY")
            self._move_joints_tip_to_base(READY_JOINT_POSITIONS)

            self._set_state("PLAN_PRE_GRASP")
            plan = self.pose_planner(block_base_point)

            pre_grasp_joints = self._solve_pose(
                plan.pre_grasp,
                READY_JOINT_POSITIONS,
            )
            grasp_joints = self._solve_pose(plan.grasp, pre_grasp_joints)
            lift_joints = self._solve_pose(plan.lift, grasp_joints)
            place_joints = self._solve_pose(plan.place, lift_joints)

            grasp_joints["gripper"] = CLOSED_GRIPPER_RAD
            lift_joints["gripper"] = CLOSED_GRIPPER_RAD
            place_joints["gripper"] = CLOSED_GRIPPER_RAD

            self._set_state("MOVE_PRE_GRASP")
            self._move(pre_grasp_joints)

            self._set_state("OPEN_GRIPPER")
            self.backend.open_gripper(speed=self.speed, acc=self.acc, wait=self.wait)

            self._set_state("DESCEND")
            self._move(grasp_joints)

            self._set_state("CLOSE_GRIPPER")
            self.backend.close_gripper(speed=self.speed, acc=self.acc, wait=self.wait)

            self._set_state("LIFT")
            self._move(lift_joints)

            self._set_state("MOVE_PLACE")
            self._move(place_joints)

            self._set_state("OPEN_GRIPPER")
            self.backend.open_gripper(speed=self.speed, acc=self.acc, wait=self.wait)

            self._set_state("RETURN_READY")
            self._move(READY_JOINT_POSITIONS)

            self._set_state("RETURN_HOME")
            self._move_joints_tip_to_base(HOME_JOINT_POSITIONS)

            self._set_state("IDLE")
            return GraspExecutionResult(
                success=True,
                message="pick sequence completed",
                states=list(self.states),
            )
        except Exception as exc:
            self.backend.stop()
            self._set_state("ERROR")
            return GraspExecutionResult(
                success=False,
                message=str(exc),
                states=list(self.states),
            )

    def _solve_pose(self, pose, initial_positions):
        target_position = pose[:3, 3]
        result = self.ik_solver(
            self.model,
            target_position,
            initial_positions=initial_positions,
        )
        if not result.success:
            raise RuntimeError(result.message)
        joint_positions = dict(result.joint_positions)
        joint_positions["wrist_roll"] = self.fixed_grasp_wrist_roll_rad
        return joint_positions

    def _move(self, joint_positions):
        return self.backend.move_joints(
            joint_positions,
            speed=self.speed,
            acc=self.acc,
            wait=self.wait,
        )

    def _move_joints_tip_to_base(self, target_positions):
        for joint_name in reversed(JOINT_NAMES):
            self._move({joint_name: target_positions[joint_name]})

    def _set_state(self, state):
        self.states.append(state)
        if self.state_callback is not None:
            self.state_callback(state)
