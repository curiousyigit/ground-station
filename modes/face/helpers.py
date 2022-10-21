import cv2
import os
from typing import Tuple

def find_nearest_face(frame, draw_color=(0, 0, 255)):
    face_cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname( __file__ ), '..', '..', 'resources', 'haarcascades', 'haarcascade_frontalface_default.xml'))
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(grey, 1.4, 4)

    nearest_face = []
    nearest_face_center = []
    nearest_face_area = []

    for (x, y, w, h) in faces:
        cx = x + (w//2)
        cy = y + (h//2)
        area = w * h
        nearest_face.append([x, y, w, h])
        nearest_face_center.append([cx, cy])
        nearest_face_area.append(area)


    if len(nearest_face_area) != 0:
        i = nearest_face_area.index(max(nearest_face_area))
        face = nearest_face[i]
        cv2.rectangle(frame, (face[0], face[1]), (face[0]+face[2], face[1]+face[3]), draw_color, 2)
        return frame, nearest_face_center[i], nearest_face_area[i]

    return frame, None, None

def calculate_tracking_error(face_center: Tuple[int, int], desired_face_to_video_ratio: float, distance_factor: int, video_w: int, video_h: int, pids, previous_error: Tuple[float, float, float]):
    # raw values
    error_w = (video_w // 2) - face_center[0]
    error_h = (video_h // 2) - face_center[1]
    area_face = face_center[0] * face_center[1]
    area_video = video_w * video_h
    error_d = (desired_face_to_video_ratio - (area_face / area_video)) * distance_factor

    # smoothened values using PID
    smooth_error_w = (pids[0][0] * error_w) + (pids[0][1] * (error_w - previous_error[0]))
    smooth_error_h = (pids[1][0] * error_h) + (pids[1][1] * (error_h - previous_error[1]))
    smooth_error_d = (pids[2][0] * error_d) + (pids[2][1] * (error_d - previous_error[2]))

    return [smooth_error_w, smooth_error_h, smooth_error_d]