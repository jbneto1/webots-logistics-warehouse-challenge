from controller import Robot
from math import atan2, cos, pi, sin, sqrt


TIME_STEP = 32
EXAMPLE_BOX = 0

# Physical and navigation tuning mirrored from the C++ controller. Through
# tolerance/speeds affect intermediate waypoint slowdowns; position/angle
# tolerances affect final stop-and-align poses.
WHEEL_RADIUS_M = 0.022
AXLE_LENGTH_M = 0.128
MAX_WHEEL_SPEED_RAD_S = 18.5
POSITION_TOLERANCE_M = 0.025
THROUGH_TOLERANCE_M = 0.100
ANGLE_TOLERANCE_RAD = 0.055

FACE_EAST = 0.0
FACE_NORTH = pi / 2.0
FACE_SOUTH = -pi / 2.0

MAP_BOX_COUNT = 4
MAP_P10_WEST_CENTER = (-0.695, 0.000, FACE_NORTH)
MAP_P21_WEST_SOUTH = (-0.695, -0.425, FACE_EAST)
MAP_P22_CENTER_SOUTH = (0.000, -0.244, FACE_EAST)

MAP_IN_PICK = [
    (-0.695, 0.395, FACE_NORTH),
    (-0.545, 0.395, FACE_NORTH),
    (-0.400, 0.395, FACE_NORTH),
    (-0.245, 0.395, FACE_NORTH),
]

MAP_IN_FRONT = [
    (-0.695, 0.244, FACE_NORTH),
    (-0.545, 0.244, FACE_NORTH),
    (-0.400, 0.244, FACE_NORTH),
    (-0.245, 0.244, FACE_NORTH),
]

MAP_OUT_FRONT = [
    (0.245, -0.244, FACE_SOUTH),
    (0.395, -0.244, FACE_SOUTH),
    (0.545, -0.244, FACE_SOUTH),
    (0.695, -0.244, FACE_SOUTH),
]

MAP_OUT_DROP = [
    (0.245, -0.430, FACE_SOUTH),
    (0.395, -0.430, FACE_SOUTH),
    (0.545, -0.430, FACE_SOUTH),
    (0.695, -0.430, FACE_SOUTH),
]


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


class ExampleStudentPython(Robot):
    def __init__(self):
        super().__init__()

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.magnet_emitter = self.getDevice("magnet_emitter")
        self.task_receiver = self.getDevice("task_receiver")
        self.electromagnet_connector = self.getDevice("electromagnet_connector")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.task_receiver.enable(TIME_STEP)
        self.electromagnet_connector.enablePresence(TIME_STEP)

        self.pose = (0.0, 0.0, 0.0)
        self.have_pose = False
        self.order = ""
        self.score = 0
        self.attached_box = "none"
        self.magnet_on = False

        self.state = "WAIT_FOR_POSE"
        self.route_index = 0
        self.box_type = "?"
        self.wait_start_time = None
        self.final_message_printed = False

    def send_text(self, message):
        self.magnet_emitter.send(message)

    def read_messages(self):
        while self.task_receiver.getQueueLength() > 0:
            message = self.task_receiver.getString()
            parts = message.split()

            if len(parts) >= 6 and parts[0] == "POSE":
                self.pose = (float(parts[1]), float(parts[2]), normalize_angle(float(parts[3])))
                self.score = int(parts[4])
                self.attached_box = parts[5]
                self.have_pose = True
            elif len(parts) == 2 and parts[0] == "ORDER":
                self.order = parts[1]
                print(f"Python example received ORDER {self.order}")
            elif parts and parts[0] == "START":
                print("Python example received START")

            self.task_receiver.nextPacket()

    def set_wheel_speeds(self, linear_m_s, angular_rad_s):
        left = (linear_m_s - angular_rad_s * AXLE_LENGTH_M * 0.5) / WHEEL_RADIUS_M
        right = (linear_m_s + angular_rad_s * AXLE_LENGTH_M * 0.5) / WHEEL_RADIUS_M
        self.left_motor.setVelocity(clamp(left, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S))
        self.right_motor.setVelocity(clamp(right, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S))

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def drive_to_xy(self, x, y, stop_at_goal, through_waypoint):
        if not self.have_pose:
            self.stop()
            return False

        px, py, theta = self.pose
        dx = x - px
        dy = y - py
        distance = sqrt(dx * dx + dy * dy)
        tolerance = THROUGH_TOLERANCE_M if through_waypoint else POSITION_TOLERANCE_M

        if distance < tolerance:
            if stop_at_goal:
                self.stop()
            return True

        target_heading = atan2(dy, dx)
        heading_error = normalize_angle(target_heading - theta)
        linear = clamp(1.35 * distance, 0.085 if through_waypoint else 0.035, 0.225 if through_waypoint else 0.170)

        if abs(heading_error) > 1.05:
            linear = 0.0
        else:
            linear *= clamp(1.0 - abs(heading_error) / 1.20, 0.35, 1.0)

        angular = clamp(3.2 * heading_error, -1.70, 1.70)
        self.set_wheel_speeds(linear, angular)
        return False

    def go_through(self, pose):
        return self.drive_to_xy(pose[0], pose[1], False, True)

    def go_to_pose(self, pose):
        if not self.drive_to_xy(pose[0], pose[1], True, False):
            return False

        error = normalize_angle(pose[2] - self.pose[2])
        if abs(error) < ANGLE_TOLERANCE_RAD:
            self.stop()
            return True

        angular = clamp(2.5 * error, -1.25, 1.25)
        if abs(angular) < 0.22:
            angular = -0.22 if angular < 0.0 else 0.22
        self.set_wheel_speeds(0.0, angular)
        return False

    def back_to(self, pose):
        if not self.have_pose:
            self.stop()
            return False

        px, py, theta = self.pose
        dx = pose[0] - px
        dy = pose[1] - py
        distance = sqrt(dx * dx + dy * dy)

        if distance < 0.055:
            self.stop()
            return True

        desired_front_heading = normalize_angle(atan2(dy, dx) + pi)
        heading_error = normalize_angle(desired_front_heading - theta)
        linear = -clamp(1.85 * distance, 0.090, 0.220)
        if abs(heading_error) > 1.05:
            linear = 0.0
        else:
            linear *= clamp(1.0 - abs(heading_error) / 1.20, 0.55, 1.0)
        self.set_wheel_speeds(linear, clamp(3.4 * heading_error, -1.75, 1.75))
        return False

    def wait_seconds(self, seconds):
        if self.wait_start_time is None:
            self.wait_start_time = self.getTime()
            self.stop()

        if self.getTime() - self.wait_start_time >= seconds:
            self.wait_start_time = None
            return True

        self.stop()
        return False

    def magnet_pick(self):
        if self.magnet_on:
            return

        if self.electromagnet_connector.getPresence():
            self.electromagnet_connector.lock()

        if self.electromagnet_connector.isLocked():
            self.magnet_on = True
            self.send_text("MAGNET_ON")
            print("Python example connector locked")

    def magnet_drop(self):
        if self.magnet_on:
            self.electromagnet_connector.unlock()
            self.magnet_on = False
            self.send_text("MAGNET_OFF")
            print("Python example connector unlocked")

    def change_state(self, next_state):
        self.state = next_state
        self.route_index = 0
        self.wait_start_time = None
        print(f"Python example state: {self.state}")

    def follow_route(self, route):
        if self.route_index < len(route) - 1:
            if self.go_through(route[self.route_index]):
                self.route_index += 1
            return False

        return self.go_to_pose(route[-1])

    def run(self):
        while self.step(TIME_STEP) != -1:
            self.read_messages()

            if self.state == "WAIT_FOR_POSE":
                if self.have_pose:
                    self.change_state("WAIT_FOR_ORDER")

            elif self.state == "WAIT_FOR_ORDER":
                if len(self.order) >= MAP_BOX_COUNT:
                    self.box_type = self.order[EXAMPLE_BOX]
                    print(f"Python example: BOX_{EXAMPLE_BOX} type is {self.box_type}.")
                    self.change_state("ROUTE_TO_BOX")

            elif self.state == "ROUTE_TO_BOX":
                route = [
                    MAP_P21_WEST_SOUTH,
                    MAP_P10_WEST_CENTER,
                    MAP_IN_FRONT[EXAMPLE_BOX],
                    MAP_IN_PICK[EXAMPLE_BOX],
                ]
                if self.follow_route(route):
                    self.change_state("PICK_BOX")

            elif self.state == "PICK_BOX":
                self.magnet_pick()
                if self.magnet_on and self.wait_seconds(0.45):
                    self.change_state("CLEAR_BOX")

            elif self.state == "CLEAR_BOX":
                if self.back_to(MAP_P10_WEST_CENTER):
                    self.change_state("DECIDE_DESTINATION")

            elif self.state == "DECIDE_DESTINATION":
                if self.box_type == "B":
                    self.change_state("ROUTE_TO_OUTGOING")
                else:
                    print(f"Python example stops here: BOX_{EXAMPLE_BOX} is {self.box_type}, so it needs a machine route.")
                    self.change_state("FINISHED")

            elif self.state == "ROUTE_TO_OUTGOING":
                route = [
                    MAP_P10_WEST_CENTER,
                    MAP_P21_WEST_SOUTH,
                    MAP_P22_CENTER_SOUTH,
                    MAP_OUT_FRONT[EXAMPLE_BOX],
                    MAP_OUT_DROP[EXAMPLE_BOX],
                ]
                if self.follow_route(route):
                    self.change_state("DROP_AT_OUTGOING")

            elif self.state == "DROP_AT_OUTGOING":
                self.magnet_drop()
                if self.wait_seconds(0.40):
                    self.change_state("CLEAR_OUTGOING")

            elif self.state == "CLEAR_OUTGOING":
                if self.back_to(MAP_OUT_FRONT[EXAMPLE_BOX]):
                    self.change_state("FINISHED")

            elif self.state == "FINISHED":
                self.stop()
                if not self.final_message_printed:
                    print(f"Python example finished. Score={self.score} attached={self.attached_box}")
                    self.final_message_printed = True

        self.stop()


controller = ExampleStudentPython()
controller.run()
