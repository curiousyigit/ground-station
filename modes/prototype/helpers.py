import cv2
from typing import Tuple

def find_nearest_face(frame, cascade_classifier, draw_color=(0, 0, 255)):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = cascade_classifier.detectMultiScale(grey, 1.1, 1)
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
        return frame, face

    return frame, None

def pid_control(pos, setpoint, pid_gains, previous_error, previous_integral_error, clamp_p = None, clamp_i = None, clamp_d = None):
    kp = pid_gains[0]
    ki = pid_gains[1]
    kd = pid_gains[2]

    error = pos - setpoint

    proportional_error = kp * error
    integral_error = ki * previous_integral_error
    derivative_error = kd * (error - previous_error)

    if clamp_p:
        proportional_error = max(min(proportional_error, clamp_p[1]), clamp_p[0])
    if clamp_i:
        integral_error = max(min(integral_error, clamp_i[1]), clamp_i[0])
    if clamp_d:
        derivative_error = max(min(derivative_error, clamp_d[1]), clamp_d[0])

    combined_error = proportional_error + integral_error + derivative_error

    return combined_error, [proportional_error, integral_error, derivative_error]