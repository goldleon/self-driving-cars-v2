import cv2
import os

import numpy as np
import math

from .colour_segmentation import ColourSegment
from ..tracker import Tracker


class TrafficLightStates:
    def __init__(self):
        # State Variables
        self.traffic_state = "Unknown"
        self.prev_traffic_state = 0

        # cosmetic control variables
        self.draw_all_detected = True
        # Create colour_segment instance and assigning it as an instance variable to TL_States
        self.colour_segmenter = ColourSegment()

    @staticmethod
    def dist(a, b):
        return int(math.sqrt(((a[1] - b[1]) ** 2) + ((a[0] - b[0]) ** 2)))

    @staticmethod
    def draw_circ_n_center(img, pt, outer_color=(0, 255, 0), inner_color=(0, 0, 255), outer_thickness=1):
        # draw the outer circle
        cv2.circle(img, (pt[0], pt[1]), pt[2], outer_color, outer_thickness)
        # draw the center of the circle
        cv2.circle(img, (pt[0], pt[1]), 2, inner_color, 3)

    @staticmethod
    def are_circles_intersecting(center, center_cmp, r1, r2):
        x1, y1 = center
        x2, y2 = center_cmp
        distSq = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
        radSumSq = (r1 + r2) * (r1 + r2)
        if distSq == radSumSq:
            return 1
        elif distSq > radSumSq:
            return -1
        else:
            return 0

    def check_color_cmb(self, center, center_cmp):
        correct_color_comb = False
        A_hue = self.colour_segmenter.HLS[center[1] - 1, center[0] - 1, 0]
        B_hue = self.colour_segmenter.HLS[center_cmp[1] - 1, center_cmp[0] - 1, 0]
        C_hue = self.colour_segmenter.HLS[center[1] - 1, int((center[0] + center_cmp[0]) / 2), 0]

        if (A_hue < 8) or ((A_hue > 56) and (A_hue < 66)):
            # A is either red or green
            if A_hue < 8:
                # A is Red then B Should be green
                if (B_hue > 56) and (B_hue < 66):
                    # print("A is Red B is green")
                    if (C_hue > 28) and (C_hue < 32):
                        return True
                    else:
                        # print("Mid is not yellow")
                        return correct_color_comb
                else:
                    # print("A is Red B is NOT green")
                    return correct_color_comb
            else:
                # A is green then B should be red
                if B_hue < 8:
                    # B is red then A should be green
                    if (A_hue > 56) and (A_hue < 66):
                        # print("B is Red A is green")
                        if (C_hue > 28) and (C_hue < 32):
                            return True
                        else:
                            # print("Mid is not yellow")
                            return correct_color_comb
                    else:
                        # print("B is Red A is green")
                        return correct_color_comb
        else:
            # print("A is Neither Red B NOR green")
            return correct_color_comb

    def get_state(self, center, center_cmp):
        tl_update = "Unknown"
        # If Center is Brighter
        if ((int(self.colour_segmenter.HLS[center[1], center[0], 1]) - int(
                self.colour_segmenter.HLS[center_cmp[1], center_cmp[0], 1])) > 10):
            # Left was Brightest [Red]
            if center[0] < center_cmp[0]:
                tl_update = "Left was Brightest [Red]"
                self.traffic_state = "Stop"
            # Right was Brightest [Green]
            elif center[0] > center_cmp[0]:
                tl_update = "Right was Brightest [Green]"
                self.traffic_state = "Go"

        # ElseIf Center_cmp is Brighter
        elif ((int(self.colour_segmenter.HLS[center[1], center[0], 1]) - int(
                self.colour_segmenter.HLS[center_cmp[1], center_cmp[0], 1])) < -10):
            # Left was Darker [Green]
            if center[0] < center_cmp[0]:
                tl_update = "Left was Darker [Green]"
                self.traffic_state = "Go"
            # Right was Darker [Red]
            elif center[0] > center_cmp[0]:
                tl_update = "Right was Darker [Red]"
                self.traffic_state = "Stop"
        else:
            if self.prev_traffic_state != "Stop":
                self.traffic_state = "Unknown"  # Because No Traffic light is detected and we werent looking for Go then Reset Traffic State

        return tl_update

    def confirm_tl_nd_ret_state(self, gray, frame_draw):
        frame_draw_special = frame_draw.copy()
        tl_update = "Unknown"

        # 2. Apply the HoughCircles to detect the circular regions in the Image
        NumOfVotesForCircle = 16  # parameter 1 MinVotes needed to be classified as circle
        CannyHighThresh = 230  # High threshold value for applying canny
        minDistanceBtwnCircles = 5  # kept as sign will likely not be overlapping
        max_rad = 50  # smaller circles dont have enough votes so only maxRadius need to be controlled
        # As signs are right besides road so they will eventually be in view so ignore circles larger than said limit
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, minDistanceBtwnCircles, param1=CannyHighThresh,
                                   param2=NumOfVotesForCircle, minRadius=5, maxRadius=max_rad)
        # 3. Loop over detected Circles
        if circles is not None:
            circles = np.uint16(np.around(circles))

            # 4. Check if Circles larger then minim size
            for index, circle in enumerate(circles[0, :]):

                center = (int(circle[0]) - 1, int(circle[1]) - 1)
                radius = int(circle[2] + 5)

                for index_cmp, circle_cmp in enumerate(circles[0, :]):

                    # Dont want to compare any circle with itself
                    if index_cmp != index:
                        center_cmp = (int(circle_cmp[0]) - 1, int(circle_cmp[1]) - 1)
                        radius_cmp = int(circle_cmp[2] + 5)
                        # Check if detected ROI is actually TL or not!
                        point_Dist = self.dist(center, center_cmp)

                        if ((point_Dist > 10) and (point_Dist < 80) and (abs(center[0] - center_cmp[0]) < 80) and
                                (abs(center[1] - center_cmp[1]) < 5) and (abs(radius - radius_cmp) < 5) and
                                (self.are_circles_intersecting(center, center_cmp, radius, radius_cmp) < 0)):

                            Correct_Color_Comb = self.check_color_cmb(center, center_cmp)

                            if Correct_Color_Comb:
                                # Confirmed Traffic Light -> Retrieve Current State [Stop,Wait,Go]
                                tl_update = self.get_state(center, center_cmp)

                                # Identify detected Red and Green Lights by drawing circles
                                self.draw_circ_n_center(frame_draw_special, circle)
                                self.draw_circ_n_center(frame_draw_special, circle_cmp, (255, 0, 0))

                                cv2.imshow('Traffic Light Confirmed!! [Checking State!!!]', frame_draw_special)

                                # Display detected Red and Green Lights on main display
                                self.draw_circ_n_center(frame_draw, circle, (0, 255, 0), outer_thickness=3)
                                self.draw_circ_n_center(frame_draw, circle_cmp, (0, 255, 0), outer_thickness=3)

                    if self.draw_all_detected:
                        self.draw_circ_n_center(frame_draw, circle, (255, 0, 255))

                traffic_state_str = "Traffic State = " + self.traffic_state
                cv2.putText(frame_draw, traffic_state_str, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255))

            if self.traffic_state != self.prev_traffic_state:
                print("#################TRAFFIC STATE CHANGED####################")
                print("Traffic_State = ", self.traffic_state, " because ", tl_update)

            self.prev_traffic_state = self.traffic_state

        return self.traffic_state

    def check_tl_state(self, frame, frame_draw):
        gray_yellow_red_regions = self.colour_segmenter.isolate_yellow_red_regions(frame)

        # Localizing Potential Candidates and Classifying them in SignDetection
        self.confirm_tl_nd_ret_state(gray_yellow_red_regions, frame_draw)


class CascadeDetector:
    def __init__(self):
        # Instance Variables
        print("Initialized Object of Cascade_Detector class")
        self.traffic_lights_states = TrafficLightStates()

    # Class Variables
    TrafficLight_cascade_str = os.path.join(os.getcwd(),
                                            'src/self_driving_car_pkg/self_driving_car_pkg/data/TrafficLight_cascade.xml')
    traffic_light_cascade = cv2.CascadeClassifier()

    # -- 1. Load the cascades
    if not traffic_light_cascade.load(cv2.samples.findFile(TrafficLight_cascade_str)):
        print('--(!)Error loading face cascade')
        exit(0)

    def detect(self, img, img_draw):

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        traffic_light_confirmed = False
        traffic_state = "Unknown"
        tl_confirmed_mask = np.zeros_like(gray)

        target = self.traffic_light_cascade.detectMultiScale(img)

        for (x, y, w, h) in target:
            cv2.rectangle(img_draw, (x, y), (x + w, y + h), (0, 165, 255), 2)

            TL_Maybe_mask = np.zeros_like(gray)
            TL_Maybe_mask[y:y + h, x:x + w] = 255
            img_ROI = cv2.bitwise_and(img, img, mask=TL_Maybe_mask)

            cv2.imshow('Detected TL', img_ROI)

            # Reconfirm if detected Traffic Light was the desired one
            self.traffic_lights_states.check_tl_state(img_ROI, img_draw)
            if self.traffic_lights_states.traffic_state != "Unknown":
                print("Traffic State Received = ", traffic_state)
                # Confirm Traffic Light
                cv2.rectangle(img_draw, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Start Tracking
                traffic_light_confirmed = True
                tl_confirmed_mask = TL_Maybe_mask
                break

        return traffic_light_confirmed, tl_confirmed_mask, traffic_state, gray


cascade_detector = CascadeDetector()


class TrafficLightsTracker(Tracker):
    def __init__(self):
        super().__init__()
        self.tracked_roi_mask = None
        self.collision_imminent = False

    def init_tracker(self, label, gray, frame_draw, start_point, end_point, mask_to_track=None):
        # Calling parents class init_tracker() using super()
        super().init_tracker(label, gray, frame_draw, start_point, end_point, mask_to_track=mask_to_track)

        # Resetting instance variables [ Mask to track ,
        #                                Car <==(Proximity)==> Traffic Light]
        self.tracked_roi_mask = mask_to_track
        self.collision_imminent = False

    def santitze_points(self, pts_src, pts_dst):
        # Idea was to Order on Descending Order of Strongest Points [Strength here is
        # considered when two points have minimum distance between each other]
        pt_idx = 0
        dist_list = []
        for pt in pts_src:
            pt_b = pts_dst[pt_idx]
            dist_list.append(self.distance(pt, pt_b))
            pt_idx += 1

        pts_src_list = pts_src.tolist()
        pts_dst_list = pts_dst.tolist()

        pts_src_list = [x for _, x in sorted(zip(dist_list, pts_src_list))]
        pts_dst_list = [x for _, x in sorted(zip(dist_list, pts_dst_list))]

        pts_src = np.asarray(pts_src_list, dtype=np.float32)
        pts_dst = np.asarray(pts_dst_list, dtype=np.float32)

        return pts_src, pts_dst

    def EstimateTrackedRect(self, pts_src, pts_dst, img_draw):
        self.mode = "Tracking"

        if len(pts_src) >= 3:
            # ===================================== Fetching strongest 4 points ======================================
            # Remove Noisy Points
            pts_src, pts_dst = self.santitze_points(pts_src, pts_dst)
            # Only 4 points are required to estimate transform
            pts_src = pts_src[0:3][:]
            pts_dst = pts_dst[0:3][:]

            # ===================================== Estimating Tracked ROI from Tracked Points ======================================
            # We are only concerned with how much close TL is in the z direction
            M = cv2.getAffineTransform(pts_src, pts_dst)
            # Retrieving transformed ROI mask using Homography matrix
            self.tracked_roi_mask = cv2.warpAffine(self.tracked_roi_mask, M,
                                                   (self.tracked_roi_mask.shape[1], self.tracked_roi_mask.shape[0]),
                                                   flags=cv2.INTER_CUBIC)
            # https://stackoverflow.com/questions/39371507/image-loses-quality-with-cv2-warpperspective
            # Smoothing by warping is caused by interpolation

            # ===================================== Refining Tracked ROI =================================================
            tracked_roi_mask = np.zeros_like(self.tracked_roi_mask)
            # Connecting closely disconnected ends
            kernel = np.ones((2, 2), dtype=np.uint8)
            closing = cv2.morphologyEx(self.tracked_roi_mask, cv2.MORPH_CLOSE, kernel)
            # Retrieving the largest contour (Leaving out the noise)
            cnts = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
            cnt = max(cnts, key=cv2.contourArea)

            # ===================================== Estimating [ Car <--- (proximity)--> TL ]  ======================================
            x, y, w, h = cv2.boundingRect(cnt)
            # Check Proximity of car <===> TL [Set CollisionIminent to True if Close Enough]
            if abs((x + w) - self.tracked_roi_mask.shape[1]) < (0.3 * self.tracked_roi_mask.shape[1]):
                self.collision_imminent = True
            # Draw the (Refined) tracked_roi_mask on the empty image
            box = np.int0(cv2.boxPoints(cv2.minAreaRect(cnt)))
            cv2.drawContours(tracked_roi_mask, [box], 0, 255, -1)
            # Drawing Tracked Traffic Light Rect On img_draw for display
            cv2.drawContours(img_draw, [box], 0, (255, 0, 0), 2)
        else:
            print("Points less then 3, Error!!!")
            # Tracking failed ! ==>  ( Back to detection mode )
            self.mode = "Detection"
            # Set Img_dst_2 to Already saved Tracked Roi One last Time
            tracked_roi_mask = self.tracked_roi_mask
            self.collision_imminent = False

        return tracked_roi_mask

    def track(self, img, frame_draw):
        # 1 : Setting the Tracked ROI with Tracked_ROI_mask instance variable
        tracked_roi_mask = self.tracked_roi_mask
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.old_gray, gray, self.p0, None, **self.lk_params)

        # 5a. If no flow, look for new points
        if (p1 is None) or (len(p1[st == 1]) < 3):
            # if p1 is None:
            self.mode = "Detection"
            self.mask = np.zeros_like(frame_draw)
            self.reset()
        # 5b. If flow , Extract good points ... Update SignTrack class
        else:
            # Select good points
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]

            tracked_roi_mask = self.EstimateTrackedRect(good_old, good_new, frame_draw)

            # Draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = (int(x) for x in new.ravel())
                c, d = (int(x) for x in old.ravel())
                self.mask = cv2.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
                frame_draw = cv2.circle(frame_draw, (a, b), 5, self.color[i].tolist(), -1)
            frame_draw_ = frame_draw + self.mask  # Display the image with the flow lines
            np.copyto(frame_draw, frame_draw_)  # important to copy the data to same address as frame_draw
            self.old_gray = gray.copy()  # Update the previous frame and previous points
            self.p0 = good_new.reshape(-1, 1, 2)
        # 3 : Extracting Tracked ROi by masking original image using retrieved mask
        img_ROI_tracked = cv2.bitwise_and(img, img, mask=tracked_roi_mask)

        return img_ROI_tracked

    def reset(self):
        # Reset inherited instance variables +
        #       child class instance variables
        super().reset()

        self.tracked_roi_mask = None
        self.collision_imminent = False


tl_tracker_ = TrafficLightsTracker()
tl_states_ = TrafficLightStates()


def detect_traffic_lights(img, img_draw):
    curr_tl_state = "Unknown"

    if tl_tracker_.mode == "Detection":

        # Detect traffic light using cascade detector
        tl_confirmed, tl_confirmed_mask, tl_state_during_detect, gray = cascade_detector.detect(img, img_draw)

        if tl_confirmed:
            # Init Tracker
            print("Confirmed Traffic Light detection ===> ( Start Tracking )")
            curr_tl_state = tl_state_during_detect
            tl_tracker_.init_tracker(curr_tl_state, gray, img_draw, None, None, tl_confirmed_mask)
    else:
        Tracked_ROI = tl_tracker_.track(img, img_draw)
        tl_states_.check_tl_state(Tracked_ROI, img_draw)
        curr_tl_state = tl_states_.traffic_state
        if tl_states_.traffic_state != "Unknown":
            print("Traffic State Received While Tracking ", tl_states_.traffic_state)
            Collision_State = "Collision_State = " + str(tl_tracker_.collision_imminent)
            cv2.putText(img_draw, Collision_State, (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255))
            cv2.imshow("Tracking (Last_Known_State)", img_draw)

    return curr_tl_state, tl_tracker_.collision_imminent
