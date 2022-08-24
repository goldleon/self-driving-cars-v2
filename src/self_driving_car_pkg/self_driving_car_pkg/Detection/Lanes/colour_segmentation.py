import cv2
import numpy as np


# White Regions Range
hue_l = 0
ligth_l = 225
saturation_l = 0

# Yellow Regions Range

hue_l_y = 30
hue_h_y = 33
ligth_l_y = 160
saturation_l_y = 0


def maskextract():
    mask = clr_segment(hls, (hue_l, ligth_l, saturation_l), (255, 255, 255))
    mask_y = clr_segment(
        hls, (hue_l_y, ligth_l_y, saturation_l_y), (hue_h_y, 255, 255)
    )  # combine 6ms

    mask_ = mask != 0
    dst = src * (mask_[:, :, None].astype(src.dtype))

    mask_y_ = mask_y != 0
    dst_Y = src * (mask_y_[:, :, None].astype(src.dtype))

    cv2.imshow("white_regions", dst)
    cv2.imshow("yellow_regions", dst_Y)


def on_hue_low_change(val):
    global hue_l
    hue_l = val
    maskextract()


def on_lit_low_change(val):
    global lit_l
    lit_l = val
    maskextract()


def on_sat_low_change(val):
    global sat_l
    sat_l = val
    maskextract()


def on_hue_low_y_change(val):
    global hue_l_y
    hue_l_y = val
    maskextract()


def on_hue_high_y_change(val):
    global hue_h_y
    hue_h_y = val
    maskextract()


def on_lit_low_y_change(val):
    global lit_l_y
    lit_l_y = val
    maskextract()


def on_sat_low_y_change(val):
    global sat_l_y
    sat_l_y = val
    maskextract()


cv2.namedWindow("white_regions")
cv2.namedWindow("yellow_regions")

cv2.createTrackbar("Hue_L", "white_regions", hue_l, 255, on_hue_low_change)
cv2.createTrackbar("Lit_L", "white_regions", ligth_l, 255, on_lit_low_change)
cv2.createTrackbar("Sat_L", "white_regions", saturation_l, 255, on_sat_low_change)

cv2.createTrackbar("Hue_L_Y", "yellow_regions", hue_l_y, 255, on_hue_low_y_change)
cv2.createTrackbar("Hue_H_Y", "yellow_regions", hue_h_y, 255, on_hue_high_y_change)
cv2.createTrackbar("Lit_L_Y", "yellow_regions", ligth_l_y, 255, on_lit_low_y_change)
cv2.createTrackbar(
    "Sat_L_Y", "yellow_regions", saturation_l_y, 255, on_sat_low_y_change
)


def color_segment(hls_segment, lower_range, upper_range):
    range_mask = cv2.inRange(hls_segment, lower_range, upper_range)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(range_mask, cv2.MORPH_DILATE, kernel)


def segment_lanes(frame, min_area):
    hls_segment = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # Segment white region
    white_regions = color_segment(
        hls_segment, np.array([hue_l, ligth_l, saturation_l]), np.array([255, 255, 255])
    )

    # Segment yellow region
    yellow_regions = color_segment(
        hls_segment,
        np.array([hue_l_y, ligth_l_y, saturation_l_y]),
        np.array([hue_h_y, 255, 255]),
    )

    cv2.imshow("White regions", white_regions)
    cv2.imshow("Yellow regions", yellow_regions)

    cv2.waitKey(1)
