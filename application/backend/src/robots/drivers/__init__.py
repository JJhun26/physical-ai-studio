"""Self-contained RobotClient drivers for arms not covered by the physicalai package.

Currently: Fairino FR5 follower (Ethernet / Fairino SDK) and uArm leader (Feetech
STS3215 serial bus). Both expose the same 6 normalized joint features
(``j1.pos`` .. ``j6.pos``) so the name-matched ``TeleoperateWorker`` can drive the
FR5 from the uArm exactly like an SO-ARM leader/follower pair.
"""

NUM_JOINTS = 6
JOINT_NAMES: list[str] = [f"j{i}" for i in range(1, NUM_JOINTS + 1)]
POS_FEATURES: list[str] = [f"{name}.pos" for name in JOINT_NAMES]
