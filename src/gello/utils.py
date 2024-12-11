from typing import Tuple

import numpy as np

from gello.driver import DynamixelDriver


def calculate_offsets(
          port: str = "/dev/ttyUSB0", 
          start_joints: Tuple[float, ...] = (0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4),
          joint_signs:  Tuple[float, ...] = (1, -1, 1, 1, 1, 1, 1),
          gripper: bool = True):

    assert len(joint_signs) == len(start_joints)
    for idx, j in enumerate(joint_signs):
        assert (
            j == -1 or j == 1
        ), f"Joint idx: {idx} should be -1 or 1, but got {j}."
    num_robot_joints = len(start_joints)
    extra_joints = 1 if gripper else 0
    num_joints = num_robot_joints + extra_joints

    joint_ids = list(range(1, num_joints + 1))
    driver = DynamixelDriver(joint_ids, port=port, baudrate=57600)

    # assume that the joint state shouold be args.start_joints
    # find the offset, which is a multiple of np.pi/2 that minimizes the error between the current joint state and args.start_joints
    # this is done by brute force, we seach in a range of +/- 8pi

    def get_error(offset: float, index: int, joint_state: np.ndarray) -> float:
        joint_sign_i = joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = start_joints[index]
        return np.abs(joint_i - start_i)

    for _ in range(10):
        driver.get_joints()  # warmup

    for _ in range(1):
        best_offsets = []
        curr_joints = driver.get_joints()
        for i in range(num_robot_joints):
            best_offset = 0
            best_error = 1e6
            for offset in np.linspace(
                -8 * np.pi, 8 * np.pi, 8 * 4 + 1
            ):  # intervals of pi/2
                error = get_error(offset, i, curr_joints)
                if error < best_error:
                    best_error = error
                    best_offset = offset
            best_offsets.append(best_offset)
        print()
        print("best offsets               : ", [f"{x:.3f}" for x in best_offsets])
        print(
            "best offsets function of pi: ["
            + ", ".join([f"{int(np.round(x/(np.pi/2)))}*np.pi/2" for x in best_offsets])
            + " ]",
        )
        if gripper:
            print(
                "gripper open (degrees)       ",
                np.rad2deg(driver.get_joints()[-1]) - 0.2,
            )
            print(
                "gripper close (degrees)      ",
                np.rad2deg(driver.get_joints()[-1]) - 42,
            )
        driver.close()
    

def check_joint_discrepency(q1, q2, max_joint_delta = 0.8) -> bool:
	q1 = np.array(q1)
	q2 = np.array(q2)
	abs_deltas = np.abs(q1 - q2)
	id_max_joint_delta = np.argmax(abs_deltas)
	res = True
	if abs_deltas[id_max_joint_delta] > max_joint_delta:
		id_mask = abs_deltas > max_joint_delta
		print()
		ids = np.arange(len(id_mask))[id_mask]
		for i, delta, joint, current_j in zip(
			ids,
			abs_deltas[id_mask],
			q1[id_mask],
			q2[id_mask],
		):
			print(f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}")
			res = False
	return res