import cv2
import os
from typing import Tuple

def find_nearest_face(frame, draw_color=(0, 0, 255)):
    face_cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname( __file__ ), '..', '..', 'resources', 'haarcascades', 'haarcascade_frontalface_default.xml'))
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(grey, 1.2, 4)

    nearest_face_center = []
    nearest_face_area = []

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), draw_color, 2)
        cx = x + (w//2)
        cy = y + (h//2)
        area = w * h
        nearest_face_center.append([cx, cy])
        nearest_face_area.append(area)


    if len(nearest_face_area) != 0:
        i = nearest_face_area.index(max(nearest_face_area))
        return frame, nearest_face_center[i], nearest_face_area[i]

    return frame, None, None

def calculate_tracking_error(face_center: Tuple[int, int], desired_face_to_video_ratio: float, video_w: int, video_h: int, pid: Tuple[float, float, float], previous_error: Tuple[float, float, float]):
    # raw values
    error_w = face_center[0] - video_w//2
    error_h = face_center[1] - video_h//2
    area_face = face_center[0] * face_center[1]
    area_video = video_w * video_h
    error_d = desired_face_to_video_ratio - (area_face / area_video)

    # smoothened values using PID
    smooth_error_w = (pid[0] * error_w) + (pid[1] * (error_w - previous_error[0]))
    smooth_error_h = (pid[0] * error_h) + (pid[1] * (error_h - previous_error[1]))
    smooth_error_d = (pid[0] * error_d) + (pid[1] * (error_d - previous_error[2]))

    return [smooth_error_w, smooth_error_h, smooth_error_d]