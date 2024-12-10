import time
from threading import Event, Lock, Thread
from typing import Protocol, Sequence, Optional, Tuple

import numpy as np
from gello.driver import DynamixelDriverProtocol, DynamixelDriver

class DynamixelRobot:
    """A class representing a UR robot."""
    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = True,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        self.gripper_open_close: Optional[Tuple[float, float]]
        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol


        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            self._driver.set_torque_mode(False)
        else:
            raise NotImplementedError("Simulated robot not implemented yet")
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            current_joints = self.get_joint_state()
            assert current_joints.shape == start_joints.shape
            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]
            for c_joint, s_joint, joint_offset in zip(
                current_joints, start_joints, self._joint_offsets
            ):
                new_joint_offsets.append(
                    np.pi * 2 * np.round((s_joint - c_joint) / (2 * np.pi))
                    + joint_offset
                )
            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode
    
    def close(self):
        self._driver.close()


# def create_gello(
#     port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FAFX-if00-port0"
# ) -> DynamixelRobot:
# 	return DynamixelRobot(
# 				port=port,
# 				real=True,
# 				joint_ids=(1, 2, 3, 4, 5, 6, 7),
# 				joint_offsets=(
# 					5 * np.pi / 2,
# 					2 * np.pi / 2,
# 					0 * np.pi / 2,
# 					2 * np.pi / 2,
# 					2 * np.pi / 2,
# 					2 * np.pi / 2,
# 					(2 * np.pi / 2) - np.pi/4,
# 				),
# 				joint_signs=(1, 1, 1, 1, 1, -1, 1),
# 				gripper_config=(8, 195, 153),

def main():
    r = DynamixelRobot(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            5 * np.pi / 2,
            2 * np.pi / 2,
            0 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            (2 * np.pi / 2) - np.pi/4,
        ),
    )
    print(r.get_joint_state())

if __name__ == "__main__":
    main()  # Test the driver