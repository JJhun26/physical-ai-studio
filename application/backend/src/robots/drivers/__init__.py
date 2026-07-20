"""Self-contained RobotClient drivers for arms not covered by the physicalai package.

Currently: Fairino FR5 follower (Ethernet / Fairino SDK) and uArm leader (Feetech
STS3215 serial bus). Both expose the same 6 normalized joint features
(``j1.pos`` .. ``j6.pos``) so the name-matched ``TeleoperateWorker`` can drive the
FR5 from the uArm exactly like an SO-ARM leader/follower pair.

The FR5 follower additionally carries a DH Robotics PGEA-100-40 parallel gripper on
its wrist flange, exposed as a 7th feature ``gripper.pos``. It is a separate DOF from
the six arm joints (driven by ``MoveGripper``, not ``ServoJ``), so ``NUM_JOINTS`` stays
6 and the gripper is appended after the arm joints. ``gripper.pos`` is carried in
METRES over the full stroke [0, ``GRIPPER_STROKE_M``], 0 = closed, matching the two
prismatic carriage joints in ``fr5_pgea.urdf`` so the viewer renders it directly.
"""

NUM_JOINTS = 6
JOINT_NAMES: list[str] = [f"j{i}" for i in range(1, NUM_JOINTS + 1)]
POS_FEATURES: list[str] = [f"{name}.pos" for name in JOINT_NAMES]

# PGEA-100-40 gripper, carried as one extra feature after the six arm joints.
GRIPPER_FEATURE = "gripper.pos"
# Per-carriage travel in metres (drawing: MIN 0.8mm .. MAX 40.8mm opening -> 20mm each).
# gripper.pos spans [0, GRIPPER_STROKE_M]; 0 = closed, GRIPPER_STROKE_M = fully open.
GRIPPER_STROKE_M = 0.020
