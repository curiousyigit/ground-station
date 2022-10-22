import cv2
import os
from typing import Tuple

def find_nearest_body(frame, draw_color=(0, 0, 255)):
    body_cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname( __file__ ), '..', '..', 'resources', 'haarcascades', 'haarcascade_upperbody.xml'))
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bodys = body_cascade.detectMultiScale(grey, 1.1, 3)

    nearest_body = []
    nearest_body_center = []
    nearest_body_area = []

    for (x, y, w, h) in bodys:
        cx = x + (w//2)
        cy = y + (h//2)
        area = w * h
        nearest_body.append([x, y, w, h])
        nearest_body_center.append([cx, cy])
        nearest_body_area.append(area)


    if len(nearest_body_area) != 0:
        i = nearest_body_area.index(max(nearest_body_area))
        body = nearest_body[i]
        cv2.rectangle(frame, (body[0], body[1]), (body[0]+body[2], body[1]+body[3]), draw_color, 2)
        return frame, nearest_body_center[i], nearest_body_area[i]

    return frame, None, None

def calculate_tracking_error(body_center: Tuple[int, int], desired_body_to_video_ratio: float, distance_factor: int, video_w: int, video_h: int, pids, previous_error: Tuple[float, float, float]):
    # raw values
    error_w = (video_w // 2) - body_center[0]
    error_h = (video_h // 2) - body_center[1]
    area_body = body_center[0] * body_center[1]
    area_video = video_w * video_h
    error_d = (desired_body_to_video_ratio - (area_body / area_video)) * distance_factor

    # smoothened values using PID
    smooth_error_w = (pids[0][0] * (error_w - previous_error[0])) + (pids[0][2] * error_w)
    smooth_error_h = (pids[1][0] * (error_h - previous_error[1])) + (pids[1][2] * error_h)
    smooth_error_d = (pids[2][0] * (error_d - previous_error[2])) + (pids[2][2] * error_d)

    return [smooth_error_w, smooth_error_h, smooth_error_d]