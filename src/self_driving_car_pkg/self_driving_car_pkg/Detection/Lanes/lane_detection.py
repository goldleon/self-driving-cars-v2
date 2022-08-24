from distutils.command.config import config
import cv2


from .colour_segmentation import segment_lanes
from ...config import config


def detect_lanes(img):
    # Cropping image juste bellow the horizon
    cropped_img = img[config.CropHeight_resized, :, :]
    # Call segmentation method
    segment_lanes(cropped_img, config.minArea_resized)
