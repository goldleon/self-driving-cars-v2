import cv2

from .Detection.Lanes.lane_detection import detect_lanes


class Car:
    def drive_car(self, frame):
        img = frame[0:640, 238:1042]
        # resize to improve performance
        img = cv2.resize(img, (320, 240))

        original_img = img.copy()

        distance, curvature = detect_lanes(img)

        current_state = [distance, curvature, img]

        return
