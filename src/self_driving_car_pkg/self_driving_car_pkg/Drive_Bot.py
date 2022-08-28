import cv2
from .Detection.Lanes.lane_detection import detect_lanes
from .Detection.Signs.sign_detection import detect_signs
from .Detection.TrafficLights.trafic_lights_detection import detect_traffic_lights

from numpy import interp


class Control:
    def __init__(self):
        # Lane assist Variable
        self.angle = 0.0
        self.speed = 80
        # Cruise_Control Variable
        self.prev_mode = "Detection"
        self.increaseTireSpeedInTurns = False
        # Nav T-Junc Variable
        self.prev_mode_traffic_lights = "Detection"
        self.left_turn_iterations = 0
        self.frozen_angle = 0
        self.detected_left_turn = False
        self.activate_left_turn = False
        # Cross. Intersection Variables
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False
        self.crossing_intersection_timer = 0

    def follow_lane(self, max_sane_dist, dist, curv, mode, tracked_class):
        # 2. Cruise control speed adjusted to match road speed limit
        if (tracked_class != 0) and (self.prev_mode == "Tracking") and (mode == "Detection"):
            if tracked_class == "speed_sign_30":
                self.speed = 30
            elif tracked_class == "speed_sign_60":
                self.speed = 60
            elif tracked_class == "speed_sign_90":
                self.speed = 90
            elif tracked_class == "stop":
                self.speed = 0
                print("Stopping Car !!!")

        self.prev_mode = mode  # Set prevMode to current Mode

        max_turn_angle = 90;
        max_turn_angle_neg = -90;
        req_turn_angle = 0

        if (dist > max_sane_dist) or (dist < (-1 * max_sane_dist)):
            if dist > max_sane_dist:
                req_turn_angle = max_turn_angle + curv
            else:
                req_turn_angle = max_turn_angle_neg + curv
        else:
            car_offset = interp(dist, [-max_sane_dist, max_sane_dist], [-max_turn_angle, max_turn_angle])
            req_turn_angle = car_offset + curv

        # handle overflow
        if (req_turn_angle > max_turn_angle) or (req_turn_angle < max_turn_angle_neg):
            if req_turn_angle > max_turn_angle:
                req_turn_angle = max_turn_angle
            else:
                req_turn_angle = max_turn_angle_neg
        # Handle max car turn ability
        self.angle = interp(req_turn_angle, [max_turn_angle_neg, max_turn_angle], [-45, 45])
        if self.increaseTireSpeedInTurns and (tracked_class != "left_turn"):
            if self.angle > 30:
                car_speed_turn = interp(self.angle, [30, 45], [80, 100])
                self.speed = car_speed_turn
            elif self.angle < (-30):
                car_speed_turn = interp(self.angle, [-45, -30], [100, 80])
                self.speed = car_speed_turn

    def obey_left_turn(self, mode):

        self.speed = 50
        # Car starts tracking left turn...
        if (self.prev_mode_traffic_lights == "Detection") and (mode == "Tracking"):
            self.prev_mode_traffic_lights = "Tracking"
            self.detected_left_turn = True
        elif (self.prev_mode_traffic_lights == "Tracking") and (mode == "Detection"):
            self.detected_left_turn = False
            self.activate_left_turn = True
            # Move left by 7 degree every 20th iteration after a few waiting a bit
            if ((self.left_turn_iterations % 10) == 0) and (self.left_turn_iterations > 50):
                self.frozen_angle = self.frozen_angle - 7

            # After a time period has passed [ De-Activate Left Turn + Reset Left Turn Variables ]
            if self.left_turn_iterations == 170:
                self.prev_mode_traffic_lights = "Detection"
                self.activate_left_turn = False
                self.left_turn_iterations = 0

            self.left_turn_iterations = self.left_turn_iterations + 1

        # Angle of car adjusted here
        if self.activate_left_turn or self.detected_left_turn:
            # Follow previously Saved Route
            self.angle = self.frozen_angle

    def obey_traffic_lights(self, traffic_state, close_proximity):
        # A: Car was close to TL which was signalling stop
        if (traffic_state == "Stop") and close_proximity:
            self.speed = 0  # Stopping car
            self.STOP_MODE_ACTIVATED = True
        else:
            # B: Car is Nav. Traffic Light
            if self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED:
                # B-1: Car was stopped at Red and now TL has turned green
                if self.STOP_MODE_ACTIVATED and (traffic_state == "Go"):
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True
                # B-2: Stop Mode is activated so car cannot move
                elif self.STOP_MODE_ACTIVATED:
                    self.speed = 0
                # B-3: Go Mode is activated --> Car moves straight ignoring lane assist
                #                               for a few moments while it crosses intersection
                elif self.GO_MODE_ACTIVATED:
                    self.angle = 0.0
                    self.speed = 80  # Set default speed

                    if self.crossing_intersection_timer == 200:
                        self.GO_MODE_ACTIVATED = False
                        print("Intersection Crossed !!!")
                        self.crossing_intersection_timer = 0  # Reset

                    self.crossing_intersection_timer = self.crossing_intersection_timer + 1

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


class Car:
    def __init__(self):
        self.Control = Control()

    def display_state(self, frame_disp, angle_of_car, current_speed, tracked_class, traffic_state="",
                      close_proximity=False):

        # Translate [ ROS Car Control Range ===> Real World angle and speed  ]
        angle_of_car = interp(angle_of_car, [-0.5, 0.5], [45, -45])
        if current_speed != 0.0:
            current_speed = interp(current_speed, [1, 2], [30, 90])

        ####################################  Displaying CONTROL STATE ####################################

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

        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
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
        # resizing to minimize computation time while still achieving comparable results
        img = cv2.resize(img, (320, 240))

        img_orig = img.copy()

        # ================================ [ Detection ] ============================================
        distance, curvature = detect_lanes(img)

        traffic_state, proximity_status = detect_traffic_lights(img_orig.copy(), img)

        mode, tracked_class = detect_signs(img_orig, img)

        # ========================= [ Updating Current State ] =======================================
        Current_State = [distance, curvature, img, mode, tracked_class, traffic_state, proximity_status]

        # ================================= [ Control ] ============================================
        angle_m, speed_m = self.Control.drive(Current_State)

        # ================================= [ Display ] ============================================
        self.display_state(img, angle_m, speed_m, tracked_class, traffic_state, proximity_status)

        return angle_m, speed_m, img
