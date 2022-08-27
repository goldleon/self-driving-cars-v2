import cv2
from numpy import interp

from .Detection.Lanes.lane_detection import detect_lanes
from .Detection.Signs.sign_detection import detect_signs
from .Detection.TrafficLights.trafic_lights_detection import detect_traffic_lights
from .config import config


class Control:
    def __init__(self):
        # Lane assist Variable
        self.angle = 0.0
        self.speed = 80
        # Cruise_Control Variable
        self.prev_mode = config.DETECTION
        self.increase_tire_speed_in_turns = False
        # Nav T-Junc Variable
        self.prev_mode_lt = config.DETECTION
        self.left_turn_iterations = 0
        self.frozen_angle = 0
        self.detected_left_turn = False
        self.activate_left_turn = False
        # Cross. Intersection Variables
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False
        self.crossing_intersection_timer = 0

    def drive(self, current_state):
        # Unpacking new with the old
        [dist, curve, img, mode, tracked_class, traffic_state, proximity_status] = current_state

        if (dist != 1000) and (curve != 1000):
            self.follow_lane(img.shape[1] / 4, dist, curve, mode, tracked_class)
        else:
            self.speed = 0.0  # Stop the car

        if tracked_class == "left_turn":
            self.obey_left_turn(mode)

        # Integrating the Intersection Nav. control.
        self.obey_traffic_lights(traffic_state, proximity_status)

        # Interpolating the angle and speed from real world to motor worlld
        angle_motor = interp(self.angle, [-45, 45], [0.5, -0.5])
        if self.speed != 0:
            speed_motor = interp(self.speed, [30, 90], [1, 2])
        else:
            speed_motor = 0.0

        return angle_motor, speed_motor

    def follow_lane(self, max_sane_dist, dist, curve, mode, tracked_class):
        # 2. Cruise control speed adjusted to match road speed limit
        if (tracked_class != 0) and (self.prev_mode == config.TRACKING) and (mode == config.DETECTION):
            if tracked_class == "speed_sign_30":
                self.speed = 30
            elif tracked_class == "speed_sign_60":
                self.speed = 60
            elif tracked_class == "speed_sign_90":
                self.speed = 90
            elif tracked_class == "stop":
                self.speed = 0
                print("Stopping Car !!!")

        self.prev_mode = mode  # set Current mode

        max_turn_angle = 90
        max_turn_angle_neg = -90
        req_turn_angle = 0
        if (dist > max_sane_dist) or (dist < (-1 * max_sane_dist)):
            if dist > max_sane_dist:
                req_turn_angle = max_turn_angle + curve
            else:
                req_turn_angle = max_turn_angle_neg + curve
        else:
            car_offset = interp(dist, [-max_sane_dist, max_sane_dist], [-max_turn_angle, max_turn_angle])
            req_turn_angle = car_offset + curve

        # Handle Overflow
        if (req_turn_angle > max_turn_angle) or (req_turn_angle < max_turn_angle_neg):
            if req_turn_angle > max_turn_angle:
                req_turn_angle = max_turn_angle
            else:
                req_turn_angle = max_turn_angle_neg

        # Handle Max car turn ability
        self.angle = interp(req_turn_angle, [max_turn_angle_neg, max_turn_angle], [-45, 45])
        if self.increase_tire_speed_in_turns and (tracked_class != "left_turn"):
            if self.angle > 30:
                self.speed = interp(self.angle, [30, 45], [80, 100])
            elif self.angle < (-30):
                self.speed = interp(self.angle, [-45, -30], [100, 80])

    def obey_left_turn(self, mode):
        self.speed = 50
        if (self.prev_mode_lt == config.DETECTION) and (mode == config.TRACKING):
            self.prev_mode_lt = config.TRACKING
            self.detected_left_turn = True
        elif (self.prev_mode_lt == config.TRACKING) and (mode == config.DETECTION):
            self.detected_left_turn = False
            self.activate_left_turn = True

            # Move left by 7 degree every 20th iteration after a few waiting a bit
            if (self.left_turn_iterations % 20 == 0) and (self.left_turn_iterations > 100):
                self.frozen_angle = self.frozen_angle - 7

            # After a time period has passed [ De-Activate Left Turn + Reset Left Turn Variables ]
            if self.left_turn_iterations == 250:
                self.prev_mode_lt = config.DETECTION
                self.activate_left_turn = False
                self.left_turn_iterations = 0

            self.left_turn_iterations = self.left_turn_iterations + 1

        # Angle of car adjusted here
        if self.activate_left_turn or self.detected_left_turn:
            # Follow previously Saved Route
            self.angle = self.frozen_angle

    def obey_traffic_lights(self, traffic_state, proximity_status):
        # Car is close to traffic lights signalling stop
        if traffic_state == 'Stop' and proximity_status:
            self.speed = 0  # Stropping Car
            self.STOP_MODE_ACTIVATED = True
        elif self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED:  # Car is Navigating over traffic lights
            # Car stopped at Red and now TL turned green
            if self.STOP_MODE_ACTIVATED and traffic_state == 'Go':
                self.STOP_MODE_ACTIVATED = False
                self.GO_MODE_ACTIVATED = True
            # Stop Mode is activated so car cannot move
            elif self.STOP_MODE_ACTIVATED:
                self.speed = 0
            # Go mode activated --> the Car moves straight ignoring lane assist for a moment while crossing intersection
            elif self.GO_MODE_ACTIVATED:
                self.angle = 0.0
                self.speed = 80  # Default speed
                if self.crossing_intersection_timer == 200:
                    self.GO_MODE_ACTIVATED = False
                    print("Intersection Crossed :D")
                    self.crossing_intersection_timer = 0  #

                self.crossing_intersection_timer += 1

        pass


class Car:

    def __init__(self):
        self.Control = Control()

    def display_state(self, frame_disp, angle_of_car, current_speed, tracked_class, traffic_state="",
                      close_proximity=False):
        # Translate [ ROS Car Control Range ===> Real World angle and speed  ]
        angle_of_car = interp(angle_of_car, [-0.5, 0.5], [45, -45])
        if current_speed != 0.0:
            current_speed = interp(current_speed, [1, 2], [30, 90])
        ###########################################  Displaying CONTROL STATE ####################################
        if angle_of_car < -10:
            direction_string = "[ Left ]"
            color_direction = (120, 0, 255)
        elif angle_of_car > 10:
            direction_string = "[ Right ]"
            color_direction = (120, 0, 255)
        else:
            direction_string = "[ Straight ]"
            color_direction = (0, 255, 0)

        if current_speed > 0:
            direction_string = "Moving --> " + direction_string
        else:
            color_direction = (0, 0, 255)

        cv2.putText(frame_disp, str(direction_string), (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.4, color_direction, 1)

        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "kmh ]"
        cv2.putText(frame_disp, str(angle_speed_str), (20, 20), cv2.FONT_HERSHEY_DUPLEX, 0.4, (0, 0, 255), 1)

        if tracked_class == "left_turn":
            font_scale = 0.32
            if self.Control.detected_left_turn:
                tracked_class = tracked_class + " : Detected { True } "
            else:
                tracked_class = tracked_class + " : Activated { " + str(self.Control.activate_left_turn) + " } "
        else:
            font_scale = 0.37
        cv2.putText(frame_disp, "Sign Detected ==> " + str(tracked_class), (20, 80), cv2.FONT_HERSHEY_COMPLEX,
                    font_scale, (0, 255, 255), 1)

        if traffic_state != "":
            cv2.putText(frame_disp, "Traffic Light State = [ " + traffic_state + " ] ", (20, 60),
                        cv2.FONT_HERSHEY_COMPLEX, 0.35, 255)
            if close_proximity:
                cv2.putText(frame_disp, " (P.Warning!) ", (220, 75), cv2.FONT_HERSHEY_COMPLEX, 0.35, (0, 0, 255))

    def drive_car(self, frame):
        img = frame[0:640, 238:1042]
        # resize to improve performance
        img = cv2.resize(img, (320, 240))

        original_img = img.copy()

        # ================================ [ Detection ] ============================================
        distance, curvature = detect_lanes(img)

        traffic_state, proximity_status = detect_traffic_lights(original_img.copy(), img)

        mode, tracked_class = detect_signs(original_img, img)

        # ========================= [ Updating Current State ] =======================================
        current_state = [distance, curvature, img, mode, tracked_class, traffic_state, proximity_status]

        # ================================= [ Control ] ============================================
        angle_m, speed_m = self.Control.drive(current_state)

        # ================================= [ Display ] ============================================
        self.display_state(img, angle_m, speed_m, tracked_class, traffic_state, proximity_status)

        return angle_m, speed_m, img
