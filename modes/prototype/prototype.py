import threading
import time
import asyncio
import cv2
import serial
import asyncer
import os
import mediapipe
import math

from drones.drone import get_drone
from peripherals.esp32_handheld.esp32_handheld import ESP32HandHeld
import modes.prototype.helpers as pt_helpers
from helpers.helpers import clamp, map

class CombinedMode():
    def __init__(self, drone_name, peripheral_mode):
        print(f'Mode: Prototype ({peripheral_mode})')

        self.cps = None # control loop per second
        self.previous_cpss = []
        self.vps = None # video loop per second
        self.previous_vpss = []

        self.quit = False
        self.airborne = False
        self.stabilized_since_last_iteration = True
        self.follow_face = False

        self.frame = None
        self.w, self.h = 0, 0
        self.font_size = 1
        self.battery = None

        self.peripheral_connected = False
        self.peripheral_manual_control = False
        self.led1, self.led2, self.led3 = None, None, None
        self.btn1, self.btn2, self.btn3, self.btn4 = None, None, None, None
        self.calb_system, self.calb_gyro, self.calb_accel, self.calb_mag = None, None, None, None
        self.yaw_raw, self.yaw_biased, self.roll_raw, self.roll_biased, self.pitch_raw, self.pitch_biased = None, None, None, None, None, None
        self.imu_safe_range = True

        self.video_command = None
        self.video_corrections = [0, 0, 0, 0] # [yaw, vertical, left_right, forward_backward]
        
        self.drone = get_drone(drone_name)
        self.peripheral = ESP32HandHeld(peripheral_mode, self.handle_peripheral_rx, self.handle_peripheral_disconnect,
            port = '/dev/ttyUSB0', baud_rate = 115200,
            service_uuid = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E',
            rx_characteristic_uuid = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E',
            tx_characteristic_uuid = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
        )

    def run(self):
        self.drone.initialize()
        self.battery = self.drone.get_battery()

        control_thread = threading.Thread(target=asyncio.run, args=(self.control_loop(),))
        control_thread.start()

        video_thread = threading.Thread(target=asyncio.run, args=(self.video_loop(),))
        video_thread.start()

        control_thread.join() # wait to finish
        video_thread.join() # wait to finish

    async def control_loop(self):
        await self.peripheral.connect()
        self.peripheral_connected = True
        # todo ble can't receive because its blocked, serial probably as well but bcz of the little asyncio.await its capable

        self.pids = self.drone.get_pids()

        async def send_to_peripheral(str):
            if self.peripheral_connected:
                await self.peripheral.send(str)
                pass

        await send_to_peripheral('led1=0')
        await send_to_peripheral('led2=0')
        await send_to_peripheral('led3=0')

        p_btn1 = self.btn1

        btn4_state = self.btn4
        btn4_prev_state = self.btn4
        btn4_debounce = 0

        p_airborne = self.airborne

        p_peripheral_connected = self.peripheral_connected
        await send_to_peripheral(f'led3={0 if self.peripheral_connected else 1}')

        await send_to_peripheral('get_continuous')

        p_vertical_speed = 0
        p_yaw = self.yaw_biased
        p_roll = self.roll_biased
        p_pitch = self.pitch_biased


        while not self.quit:
            start = time.time()

            if self.pitch_biased < -90 or self.pitch_biased > 90:
                self.imu_safe_range = False
                yaw_clamped = 0
                roll_clamped = 0
                pitch_clamped = 0
            else:
                self.imu_safe_range = True
                yaw_gain = 1.5
                roll_gain = 1.5
                pitch_gain = 2
                yaw_clamped = int(map(self.yaw_biased, 0, 359, -100, 100))
                roll_clamped = int(map(self.roll_biased, -89, 89, -100, 100))
                pitch_clamped = int(map(self.pitch_biased, -179, 179, -100, 100))

            if self.peripheral_connected != p_peripheral_connected:
                await send_to_peripheral(f'led3={0 if self.peripheral_connected else 1}')
                p_peripheral_connected = self.peripheral_connected

            if self.btn1 != p_btn1:
                await send_to_peripheral(f'led1={self.btn1}')
                p_btn1 = self.btn1
                self.peripheral_manual_control = True if self.btn1 else False

            if self.airborne != p_airborne:
                await send_to_peripheral(f'led2={1 if self.airborne else 0}')
                p_airborne = self.airborne

            # debounce btn4
            if self.btn4 != btn4_prev_state:
                btn4_debounce = time.time()
            if time.time() - btn4_debounce > 0.1:
                if self.btn4 != btn4_state:
                    # truly changed
                    btn4_state = self.btn4
                    if self.btn4:
                        if(self.airborne):
                            self.land()
                        else:
                            self.takeoff()
            btn4_prev_state = self.btn4

            if self.airborne:
                vertical_speed = 0
                if self.btn2:
                    vertical_speed = 50
                elif self.btn3:
                    vertical_speed = -50

                if self.imu_safe_range and (vertical_speed != p_vertical_speed or yaw_clamped != p_yaw or roll_clamped != p_roll or pitch_clamped != p_pitch):
                    if self.peripheral_manual_control:
                        self.drone.rc(yaw_clamped, vertical_speed, roll_clamped, pitch_clamped)
                        p_vertical_speed = vertical_speed
                        p_yaw = yaw_clamped
                        p_roll = roll_clamped
                        p_pitch = pitch_clamped
                    elif vertical_speed != p_vertical_speed:
                        self.drone.rc(0, vertical_speed, 0, 0)
                        p_vertical_speed = vertical_speed

                if not self.peripheral_manual_control and self.follow_face:
                    self.drone.rc(self.video_corrections[0], -self.video_corrections[1], self.video_corrections[2], self.video_corrections[3])

                if self.video_command == 'land':
                    self.land()
                    self.video_command = None
                elif self.video_command == 'left_flip':
                    self.drone.special_maneuver('left_flip')
                    self.video_command = None
                elif self.video_command == 'right_flip':
                    self.drone.special_maneuver('right_flip')
                    self.video_command = None
                    

            if self.frame is not None:
                cv2.imshow('Combined', self.frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # button 'q' to quit
                    self.follow_face= False
                    self.drone.stabilize()
                    self.quit = True
                elif key == ord('t') and not self.airborne:
                    self.takeoff()
                elif key == ord('l') and self.airborne:
                    self.land()
                elif key == ord('f') and not self.follow_face:
                    print('Following!')
                    self.follow_face= True
                elif key == ord('f') and self.follow_face:
                    print('Stopped following!')
                    self.drone.stabilize()
                    self.stabilized_since_last_iteration = True
                    self.follow_face= False
                elif key == ord('a'):
                    self.drone.rc(0, 0, -50, 0)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('d'):
                    self.drone.rc(0, 0, 50, 0)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('w'):
                    self.drone.rc(0, 0, 0, 50)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('s'):
                    self.drone.rc(0, 0, 0, -50)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('z'):
                    self.drone.rc(-50, 0, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('x'):
                    self.drone.rc(50, 0, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('c'):
                    self.drone.rc(0, 50, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('v'):
                    self.drone.rc(0, -50, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.peripheral_manual_control = True
                elif key == ord('e'):
                    self.drone.stabilize()
                    self.stabilized_since_last_iteration = True
            else:
                time.sleep(0.05)

            end = time.time()
            cps = int(1 / (end - start))
            self.previous_cpss.append(cps)
            if len(self.previous_cpss) > 120:
                self.previous_cpss.pop(0)
            self.cps = int(sum(self.previous_cpss) / len(self.previous_cpss))

        await self.peripheral.disconnect()
        self.drone.destroy()
        cv2.destroyAllWindows()

    async def video_loop(self):
        self.drone.initialize_video_feed()
        # self.w, self.h = self.drone.get_video_resolution()
        self.w, self.h = 640, 480
        self.half_w = self.w // 2
        self.half_h = self.h // 2

        desired_face_to_video_ratio = 10 # the ratio of the area of the face to the area of the video (for distance)
        previous_error = [0, 0, 0] # heading, altitude, distance (points relative to frame)
        integral_error = [0, 0, 0] # heading, altitude, distance (points relative to frame)
        acceptable_error_rate = 5 # percentage of acceptable error for x,y

        # face_cascade_classifer = cv2.CascadeClassifier(os.path.join(os.path.dirname( __file__ ), '..', '..', 'resources', 'haarcascades', 'haarcascade_frontalface_default.xml'))

        drawing_module = mediapipe.solutions.drawing_utils
        face_module = mediapipe.solutions.face_detection
        hands_module = mediapipe.solutions.hands

        faces = []
        face_id_counter = 0
        previous_face_x_error = 0
        previous_face_x_integral_error = 0

        previous_horizontal_error = 0
        previous_horizontal_integral_error = 0
        previous_vertical_error = 0
        previous_vertical_integral_error = 0

        previous_gesture = None
        previous_gesture_at = time.time()

        p_once = False

        with face_module.FaceDetection(min_detection_confidence=0.7) as face_detection:
            with hands_module.Hands(static_image_mode=False, min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=1) as hand_detection:
                while not self.quit:
                    start = time.time()

                    frame = self.drone.get_video_frame()
                    frame = cv2.resize(frame, (self.w, self.h))

                    rotational_error = 0
                    horizontal_error = 0
                    vertical_error = 0
                    distance_error = 0

                    if frame is not None:
                        t_face_s = time.time()

                        bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                        faces_result = face_detection.process(bgr)
                        life = 3 # seconds
                        frequency = 1 / self.vps if self.vps else 1 / 30

                        
                        cur_faces = []
                        if faces_result.detections:
                            for j, detection in enumerate(faces_result.detections):
                                bounding_box = detection.location_data.relative_bounding_box
                                top_left = (int(bounding_box.xmin * self.w), int(bounding_box.ymin * self.h))
                                width = int(bounding_box.width * self.w)
                                height = int(bounding_box.height * self.h)
                                bottom_right = ((top_left[0] + width), (top_left[1] + height))
                                center = (int(top_left[0] + width/2), int(top_left[1] + height/2))

                                cur_faces.append({
                                    'id': j,
                                    'center': center,
                                    'top_left': top_left,
                                    'bottom_right': bottom_right,
                                    'width': width,
                                    'height': height,
                                    'taken': False,
                                    'life': life
                                })
                        for i, face in enumerate(faces):
                            faces[i]['taken'] = False

                        if len(faces) == 0 and len(cur_faces) != 0:
                            for cur_face in cur_faces:
                                cur_face['id'] = face_id_counter
                                cur_face['taken'] = True
                                faces.append(cur_face)
                                face_id_counter += 1
                        elif len(faces) <= len(cur_faces):
                            for i, face in enumerate(faces):
                                nearest_dist = None
                                match = None
                                match_j = None
                                for j, cur_face in enumerate(cur_faces):
                                    if not cur_face['taken']:
                                        dist = math.dist(face['center'], cur_face['center'])
                                        if nearest_dist is None or dist < nearest_dist:
                                            nearest_dist = dist
                                            match = cur_face
                                            match_j = j
                                match['id'] = face['id']
                                match['taken'] = True
                                cur_faces[match_j]['taken'] = True
                                faces[i] = match

                            for j, cur_face in enumerate(cur_faces):
                                if not cur_face['taken']:
                                    cur_face['id'] = face_id_counter
                                    cur_face['taken'] = True
                                    faces.append(cur_face)
                                    face_id_counter += 1
                        elif len(faces) > len(cur_faces):
                            for j, cur_face in enumerate(cur_faces):
                                nearest_dist = None
                                match = None
                                match_i = None
                                for i, face in enumerate(faces):
                                    if not face['taken']:
                                        dist = math.dist(face['center'], cur_face['center'])
                                        if nearest_dist is None or dist < nearest_dist:
                                            nearest_dist = dist
                                            match = face
                                            match_i = i
                                if match:
                                    cur_face['id'] = match['id']
                                    cur_face['taken'] = True
                                    faces[match_i] = cur_face

                        # display faces, decrement life of non visible ones, and find first face
                        first_face = None
                        for i, face in enumerate(faces):
                            if not face['taken']:
                                if face['life'] > 0:
                                    faces[i]['life'] -= frequency
                                else:
                                    faces.pop(i)
                            is_first_face = not first_face or face['id'] < first_face['id']
                            if is_first_face:
                                first_face = face
                            cv2.rectangle(frame, face['top_left'], face['bottom_right'], (0 if is_first_face else 255, 0 if is_first_face else 255, 255), 2)
                            cv2.putText(frame, f'{face["id"]}: {int(face["life"])}s', (face['top_left'][0] + 5, face['bottom_right'][1] - 5), cv2.FONT_HERSHEY_COMPLEX, self.font_size, (255, 255, 255))


                        # track first face and center
                        if first_face is not None:
                            # get percentages of positions
                            pos_face_x = (first_face['center'][0] / self.w) * 100
                            setpoint_face_x = 50

                            # get error resulting from PID controller calculations
                            raw_face_x_error, _ = pt_helpers.pid_control(pos_face_x, setpoint_face_x, self.pids[0], previous_face_x_integral_error, previous_face_x_error, clamp_i = [-10, 10])
                            
                            # acceptable error < 10%
                            rotational_error = raw_face_x_error if (abs(raw_face_x_error) >= 5) else 0

                            # set current error as previous and add to integral for next interation
                            previous_face_x_error = raw_face_x_error
                            previous_face_x_integral_error += raw_face_x_error

                        t_face_e = time.time()

                        t_hands_s = time.time()
                        hands_result = hand_detection.process(bgr)
                        hands = []
                        finger_tip_indices = [hands_module.HandLandmark.INDEX_FINGER_TIP, hands_module.HandLandmark.MIDDLE_FINGER_TIP, hands_module.HandLandmark.RING_FINGER_TIP, hands_module.HandLandmark.PINKY_TIP]
                        if hands_result.multi_handedness:
                            for i, hand in enumerate(hands_result.multi_handedness):
                                finger_count = 0
                                fingers_bent = {'THUMB': False, 'INDEX': False, 'MIDDLE': False, 'RING': False, 'PINKY': False}

                                label = 'RIGHT' if hand.classification[0].label == 'Left' else 'LEFT' # swap for mirror
                                landmarks = hands_result.multi_hand_landmarks[i]

                                for tip_index in finger_tip_indices:
                                    finger_name = tip_index.name.split('_')[0]
                                    if(landmarks.landmark[tip_index].y > landmarks.landmark[tip_index - 2].y):
                                        fingers_bent[finger_name] = True
                                        finger_count += 1

                                palm_side = True
                                if label == 'LEFT' and landmarks.landmark[hands_module.HandLandmark.INDEX_FINGER_TIP].x > landmarks.landmark[hands_module.HandLandmark.PINKY_TIP].x:
                                    palm_side = False
                                elif label == 'RIGHT' and landmarks.landmark[hands_module.HandLandmark.INDEX_FINGER_TIP].x < landmarks.landmark[hands_module.HandLandmark.PINKY_TIP].x:
                                    palm_side = False

                                thumb_tip_x = landmarks.landmark[hands_module.HandLandmark.THUMB_TIP].x
                                thumb_mcp_x = landmarks.landmark[hands_module.HandLandmark.THUMB_TIP - 2].x
                                if label == 'RIGHT':
                                    if (palm_side and (thumb_tip_x < thumb_mcp_x)) or (not palm_side and (thumb_tip_x > thumb_mcp_x)):
                                        fingers_bent['THUMB'] = True
                                        finger_count += 1
                                else:
                                    if (palm_side and (thumb_tip_x > thumb_mcp_x)) or (not palm_side and (thumb_tip_x < thumb_mcp_x)):
                                        fingers_bent['THUMB'] = True
                                        finger_count += 1

                                bents = fingers_bent['THUMB'], fingers_bent['INDEX'], fingers_bent['MIDDLE'], fingers_bent['RING'], fingers_bent['PINKY']
                                gesture = 'unknown'
                                if bents == (False, False, False, False, False):
                                    gesture = 'open'
                                elif bents == (True, False, False, True, True):
                                    gesture = 'follow'
                                elif bents == (True, False, False, False, True):
                                    gesture = 'land'
                                elif bents == (False, True, True, True, False):
                                    gesture = 'right_flip'
                                elif bents == (False, False, True, True, True):
                                    gesture = 'left_flip'
                                elif bents == (False, False, True, True, False):
                                    gesture = 'forward'
                                elif bents == (True, False, True, True, False):
                                    gesture = 'backward'
                                elif bents == (True, True, True, True, True):
                                    gesture = 'punch'

                                wrist_x = int(landmarks.landmark[hands_module.HandLandmark.WRIST].x * 0.90 * self.w)
                                wrist_y = int(landmarks.landmark[hands_module.HandLandmark.WRIST].y * 1.10 * self.h)
                                cv2.putText(frame, gesture, (wrist_x, wrist_y), cv2.FONT_HERSHEY_COMPLEX, self.font_size, (255, 255, 255))
                                
                                hands.append({
                                    'id': i,
                                    'label': label,
                                    'palm_side': palm_side,
                                    'fingers_bent': fingers_bent,
                                    'finger_count': finger_count,
                                    'center': (int(landmarks.landmark[hands_module.HandLandmark.MIDDLE_FINGER_MCP].x * self.w), int(landmarks.landmark[hands_module.HandLandmark.MIDDLE_FINGER_MCP].y * self.h)),
                                    'gesture': gesture
                                })

                                drawing_module.draw_landmarks(frame, landmarks, hands_module.HAND_CONNECTIONS)

                        if len(hands) > 0:
                            hand = hands[0]
                            gesture = hand['gesture']

                            if gesture != previous_gesture:
                                previous_gesture = gesture
                                previous_gesture_at = time.time()
                            elif gesture == previous_gesture and time.time() - previous_gesture_at > 2: # 2 seconds
                                if gesture == 'backward':
                                    distance_error = -20
                                elif gesture == 'forward':
                                    distance_error = 20
                                elif gesture == 'land' or gesture == 'left_flip' or gesture == 'right_flip':
                                    self.video_command = gesture
                                    previous_gesture = None
                                    previous_gesture_at = time.time()
                                elif gesture == 'follow':
                                    # get percentages of positions
                                    pos_horizontal = (hand['center'][0] / self.w) * 100
                                    setpoint_horizontal = 50

                                    # get error resulting from PID controller calculations
                                    raw_horizontal_error, _ = pt_helpers.pid_control(pos_horizontal, setpoint_horizontal, self.pids[2], previous_horizontal_integral_error, previous_horizontal_error, clamp_i = [-10, 10])
                                    
                                    # acceptable error < 5%
                                    horizontal_error = raw_horizontal_error if (abs(raw_horizontal_error) >= 1) else 0

                                    # set current error as previous and add to integral for next interation
                                    previous_horizontal_error = raw_horizontal_error
                                    previous_horizontal_integral_error += raw_horizontal_error

                                    # get percentages of positions
                                    pos_vertical = (hand['center'][1] / self.w) * 100
                                    setpoint_vertical = 50

                                    # get error resulting from PID controller calculations
                                    raw_vertical_error, _ = pt_helpers.pid_control(pos_vertical, setpoint_vertical, self.pids[1], previous_vertical_integral_error, previous_vertical_error, clamp_i = [-10, 10])
                                    
                                    # acceptable error < 5%
                                    vertical_error = raw_vertical_error if (abs(raw_vertical_error) >= 1) else 0

                                    # set current error as previous and add to integral for next interation
                                    previous_vertical_error = raw_vertical_error
                                    previous_vertical_integral_error += raw_vertical_error
                                else:
                                    previous_gesture = None
                                    previous_gesture_at = time.time()
                        t_hands_e = time.time()

                        # display targets for debugging
                        rotational_error_px = int(self.half_w + ((rotational_error / 100) * self.w))
                        horizontal_error_px = int(self.half_w + ((horizontal_error / 100) * self.w))
                        vertical_error_px = int(self.half_h + ((vertical_error / 100) * self.h))

                        target_color = (0, 255, 0)
                        if distance_error < 0:
                            target_color = (0, 0, 255)
                        elif distance_error > 0:
                            target_color = (0, 255, 255)

                        # cv2.line(frame, (self.half_w, self.half_h), (horizontal_error_px, vertical_error_px), (0, 255, 255), 2)
                        cv2.line(frame, (self.half_w, self.half_h), (rotational_error_px, self.half_h), target_color, 2)
                        cv2.line(frame, (rotational_error_px, self.half_h - 10), (rotational_error_px, self.half_h + 10), target_color, 2)
                        cv2.circle(frame, (horizontal_error_px, vertical_error_px), 20, target_color, 2)
                        # cv2.circle(frame, (horizontal_error_px, vertical_error_px), z_circle_r, (255, 255, 255), 2)


                        battery_x = int(self.w * 0.03)
                        battery_y = int(self.h * 0.06)
                        overlay = frame.copy()
                        alpha = 0.5
                        cv2.rectangle(overlay, (battery_x - 10, battery_y - 20), (battery_x + 310, battery_y + 80), (0, 0, 0), -1)
                        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
                        cv2.putText(frame, f'Battery: {self.battery}, CPS: {self.cps}, VPS: {self.vps}', (battery_x, battery_y), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255, 255))
                        peripheral_status = 'Connected' if self.peripheral_connected else 'Disconnected'
                        cv2.putText(frame, f'Peripheral: {peripheral_status}', (battery_x, battery_y + 20), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255 if self.peripheral_connected else 0, 255))
                        if self.peripheral_connected:
                            yrp = str(self.yaw_biased) + ', ' + str(self.roll_biased) + ', ' + str(self.pitch_biased)
                            cv2.putText(frame, f'Yaw, roll, pitch: {yrp}', (battery_x, battery_y + 40), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255, 255))
                            btns = str(self.btn1) + ', ' + str(self.btn2) + ', ' + str(self.btn3) + ', ' + str(self.btn4)
                            cv2.putText(frame, f'Buttons: {btns}', (battery_x, battery_y + 60), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255, 255))
                            if self.peripheral_manual_control and not self.imu_safe_range:
                                cv2.putText(frame, f'IMU OUT OF RANGE!', (self.half_w - 75, self.half_h), cv2.FONT_HERSHEY_PLAIN, self.font_size * 1.5, (0, 0, 255))

                        self.frame = frame

                    self.video_corrections = [rotational_error, vertical_error, horizontal_error, distance_error]
                    end = time.time()
                    # print(f'face: {(t_face_e - t_face_s) * 1000}{(t_face_e - t_face_s) * 100 / (end - start)}, hands: {(t_hands_e - t_hands_s) * 100 / (end - start)}')

                    vps = int(1 / (end - start))
                    self.previous_vpss.append(vps)
                    if len(self.previous_vpss) > 60:
                        self.previous_vpss.pop(0)
                    self.vps = int(sum(self.previous_vpss) / len(self.previous_vpss))

    def handle_peripheral_rx(self, str):
        if str.startswith('leds') and len(str) == 10: # leds=1,1,1
            leds = str.split('=')[1].split(',')
            self.led1 = int(leds[0])
            self.led2 = int(leds[1])
            self.led3 = int(leds[2])
        elif str.startswith('btns') and len(str) == 12: # btns=1,1,1,1
            btns = str.split('=')[1].split(',')
            self.btn1 = int(btns[0])
            self.btn2 = int(btns[1])
            self.btn3 = int(btns[2])
            self.btn4 = int(btns[3])
        elif str.startswith('calb') and len(str) == 12: # calb=3,3,3,3
            calbs = str.split('=')[1].split(',')
            self.calb_system = int(calbs[0])
            self.calb_gyro = int(calbs[1])
            self.calb_accel = int(calbs[2])
            self.calb_mag = int(calbs[3])
        elif str.startswith('yrpr') and len(str) >= 10 and len(str) <= 19:
            vals = str.split('=')[1].split(',')
            self.yaw_raw = int(vals[0])
            self.roll_raw = int(vals[1])
            self.pitch_raw = int(vals[2])
        elif str.startswith('yrpb') and len(str) >= 10 and len(str) <= 19:
            vals = str.split('=')[1].split(',')
            self.yaw_biased = int(vals[0])
            self.roll_biased = int(vals[1])
            self.pitch_biased = int(vals[2])

    def handle_peripheral_disconnect(self):
        self.peripheral_connected = False

    def takeoff(self):
        print('Taking off!')
        self.drone.takeoff()
        self.drone.stabilize()
        self.stabilized_since_last_iteration = True
        self.airborne = True

    def land(self):
        print('Landing!')
        self.follow_face= False
        self.drone.land()
        self.airborne = False




def run(drone_name, peripheral_ble: bool):
    md = CombinedMode(drone_name, 'ble' if peripheral_ble else 'serial')
    md.run()

def run_debug(drone_name, peripheral_ble: bool):
    pass