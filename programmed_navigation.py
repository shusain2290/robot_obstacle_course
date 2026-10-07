
import time
import math
import pybullet as p
import pybullet_data


FRONT_LEFT_WHEEL = 2
FRONT_RIGHT_WHEEL = 3
REAR_LEFT_WHEEL = 4
REAR_RIGHT_WHEEL = 5


LEFT_WHEELS = [
    FRONT_LEFT_WHEEL,
    REAR_LEFT_WHEEL
]


RIGHT_WHEELS = [
    FRONT_RIGHT_WHEEL,
    REAR_RIGHT_WHEEL
]


def set_wheel_speeds(
    robot_id,
    left_speed,
    right_speed
):
    for wheel_joint in LEFT_WHEELS:
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=wheel_joint,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=100
        )

    for wheel_joint in RIGHT_WHEELS:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=wheel_joint,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=100
        )


def run_for_duration(duration):
    start_time = time.perf_counter()

    while (
        p.isConnected()
        and time.perf_counter() - start_time < duration
    ):
        p.stepSimulation()
        time.sleep(1 / 240)


def stop_robot(robot_id):
    set_wheel_speeds(
        robot_id,
        left_speed=0,
        right_speed=0
    )


def move_forward(robot_id, speed, duration):
    print(
        f"Moving forward at speed {speed} "
        f"for {duration} seconds"
    )

    set_wheel_speeds(
        robot_id,
        left_speed=speed,
        right_speed=speed
    )

    run_for_duration(duration)
    stop_robot(robot_id)


def move_backward(robot_id, speed, duration):
    print(
        f"Moving backward at speed {speed} "
        f"for {duration} seconds"
    )

    set_wheel_speeds(
        robot_id,
        left_speed=-speed,
        right_speed=-speed
    )

    run_for_duration(duration)
    stop_robot(robot_id)


def turn_left(robot_id, speed, duration):
    print(
        f"Turning left at speed {speed} "
        f"for {duration} seconds"
    )

    set_wheel_speeds(
        robot_id,
        left_speed=-speed,
        right_speed=speed
    )

    run_for_duration(duration)
    stop_robot(robot_id)


def turn_right(robot_id, speed, duration):
    print(
        f"Turning right at speed {speed} "
        f"for {duration} seconds"
    )

    set_wheel_speeds(
        robot_id,
        left_speed=speed,
        right_speed=-speed
    )

    run_for_duration(duration)
    stop_robot(robot_id)


def report_position(robot_id, label):
    position, orientation = (
        p.getBasePositionAndOrientation(robot_id)
    )

    x_position = position[0]
    y_position = position[1]
    z_position = position[2]

    print()
    print(label)
    print(f"X position: {x_position:.2f}")
    print(f"Y position: {y_position:.2f}")
    print(f"Z position: {z_position:.2f}")
    print()


def create_box(
    position,
    half_extents,
    color,
    mass=0
):
    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents
    )

    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color
    )

    box_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )

    return box_id


client = p.connect(p.GUI)


if client < 0:
    raise RuntimeError(
        "Could not open the PyBullet GUI."
    )


p.resetDebugVisualizerCamera(
    cameraDistance=12,
    cameraYaw=0,
    cameraPitch=-60,
    cameraTargetPosition=[0, 1, 0]
)


p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)


p.setGravity(0, 0, -9.81)


plane_id = p.loadURDF("plane.urdf")


robot_position = [0, -4, 0.2]


robot_orientation = (
    p.getQuaternionFromEuler([0, 0, 0])
)


robot_orientation = p.getQuaternionFromEuler(
    [0, 0, math.radians(90)]
)


robot_id = p.loadURDF(
    "husky/husky.urdf",
    robot_position,
    robot_orientation
)


print("Robot ID:", robot_id)


# First barrier extends from the left wall toward the center.
create_box(
    position=[-2.0, -1.0, 0.5],
    half_extents=[2.0, 0.35, 0.5],
    color=[1, 0, 0, 1]
)


# Second barrier extends from the right wall toward the center.
create_box(
    position=[2.0, 1.0, 0.5],
    half_extents=[2.0, 0.35, 0.5],
    color=[0, 0, 1, 1]
)


# Third barrier extends from the left wall toward the center.
create_box(
    position=[-2.0, 3.0, 0.5],
    half_extents=[2.0, 0.35, 0.5],
    color=[1, 1, 0, 1]
)


# Finish marker near the end of the course.
create_box(
    position=[0, 5.0, 0.25],
    half_extents=[0.5, 0.5, 0.25],
    color=[0, 1, 0, 1]
)


create_box(
    position=[-4, 1, 0.5],
    half_extents=[0.25, 5, 0.5],
    color=[0.5, 0.5, 0.5, 1]
)


create_box(
    position=[4, 1, 0.5],
    half_extents=[0.25, 5, 0.5],
    color=[0.5, 0.5, 0.5, 1]
)


run_for_duration(1)


report_position(
    robot_id,
    "Starting position"
)


# Move closer to the first barrier.
move_forward(
    robot_id,
    speed=10,
  duration=1.00
)

turn_right(
    robot_id,
    speed=5,
  duration=1.00
)

move_forward(
    robot_id,
    speed=10,
  duration=0.90
)

turn_left(
    robot_id,
    speed=5,
  duration=0.90
)

move_forward(
    robot_id,
    speed=7,
  duration=0.80
)
 
turn_left(
    robot_id,
    speed=5,
  duration=1.50
)
move_forward(
    robot_id,
    speed=7,
  duration=1.20
)

turn_right(
    robot_id,
    speed=7,
  duration=1.00
)

move_forward(
    robot_id,
    speed=10,
  duration=1.00
)

turn_right(
    robot_id,
    speed=5,
  duration=1.50
)
move_forward(
    robot_id,
    speed=7,
  duration=1.50
)
turn_left(
    robot_id,
    speed=8,
  duration=1.30
)
move_forward(
    robot_id,
    speed=8,
  duration=1.10
)
turn_left(
    robot_id,
    speed=6,
  duration=1.0
)
move_forward(
    robot_id,
    speed=8,
  duration=2.10
)
turn_right(
    robot_id,
    speed=10,
  duration=1.00
)
move_forward(
    robot_id,
    speed=20,
  duration=15.50
)




report_position(
    robot_id,
    "Final position"
)


print("Programmed route completed.")
print("Close the PyBullet window to stop.")


while p.isConnected():
    p.stepSimulation()
    time.sleep(1 / 240)

