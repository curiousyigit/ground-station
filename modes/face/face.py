import time
import cv2

from drones.drone import get_drone
import modes.face.helpers as face
import modes.face.debug as debug

def run(drone_name):
    print('Mode: face')

    desired_face_to_video_ratio = 10 # the ratio of the area of the face to the area of the video (for distance)
    previous_error = [0, 0, 0] # heading, altitude, distance (points relative to frame)
    integral_error = [0, 0, 0] # heading, altitude, distance (points relative to frame)
    acceptable_error_rate = 5 # percentage of acceptable error for x,y

    # initialize drone and its camera
    drone = get_drone(drone_name)
    drone.initialize()
    drone.initialize_video_feed()
    w, h = drone.get_video_resolution()
    half_w = w // 2
    half_h = h // 2

    airborne = False
    follow = False
    manual_control = False
    stabilized_since_last_iteration = True

    pids = drone.get_pids()

    frame_count = 0
    battery = drone.get_battery()
    battery_x = int(w * 0.05)
    battery_y = int(h * 0.9)

    while True:
        start = time.time()
        
        frame_count = frame_count + 1
        if frame_count >= 150:
            battery = drone.get_battery()
            frame_count = 0

        frame_time = time.time()
        # capture the video frame by frame
        frame = drone.get_video_frame()

        if frame is not None:

            cv2.putText(frame, f'Battery: {battery}', (battery_x, battery_y), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255))

            # center locator for debugging
            cv2.line(frame, (half_w, half_h - 10), (half_w, half_h +10), (0, 255, 255), 2)
            cv2.line(frame, (half_w - 10, half_h), (half_w + 10, half_h), (0, 255, 255), 2)

            follow_color = (0, 0 if follow else 255, 255)
            
            # find nearest face and get it's coordinates and area
            frame, nearest_face = face.find_nearest_face(frame, follow_color)

            
            if nearest_face:
                face_center = [nearest_face[0], nearest_face[1]]

                # get percentages of positions
                pos_x = (face_center[0] / w) * 100
                setpoint_x = 50
                previous_error_x = previous_error[0]
                previous_integral_x = integral_error[0]

                pos_y = (face_center[1] / h) * 100
                setpoint_y = 50
                previous_error_y = previous_error[1]
                previous_integral_y = integral_error[1]

                pos_z = (nearest_face[2] / w) * 100
                setpoint_z = desired_face_to_video_ratio
                previous_error_z = previous_error[2]
                previous_integral_z = integral_error[2]

                # get error resulting from PID controller calculations
                horizontal_error, _ = face.pid_control(pos_x, setpoint_x, pids[0], previous_integral_x, previous_error_x, clamp_i = [-10, 10])
                vertical_error, _ = face.pid_control(pos_y, setpoint_y, pids[1], previous_integral_y, previous_error_y, clamp_i = [-10, 10])
                z_error, _ = face.pid_control(pos_z, setpoint_z, pids[2], previous_integral_z, previous_error_z, clamp_i = [-10, 10])

                print(nearest_face[2], w, int((nearest_face[2]/w)*100), z_error)
                horizontal_error = int(horizontal_error)
                vertical_error = int(vertical_error)
                z_error = int(z_error)

                horizontal_error_px = int(half_w + ((horizontal_error / 100) * w))
                vertical_error_px = int(half_h + ((vertical_error / 100) * h))
                z_circle_r = 10 if z_error < 0 else 30

                cv2.line(frame, (half_w, half_h), (horizontal_error_px, vertical_error_px), (0, 255, 255), 2)
                cv2.circle(frame, (horizontal_error_px, vertical_error_px), 20, (0, 255, 255), 2)
                cv2.circle(frame, (horizontal_error_px, vertical_error_px), z_circle_r, (255, 255, 255), 2)

                applied_hor_cor = horizontal_error if (abs(horizontal_error) >= acceptable_error_rate) else 0
                applied_ver_cor = vertical_error if (abs(vertical_error) >= acceptable_error_rate) else 0
                applied_z_cor = z_error

                if follow and airborne and not manual_control:
                    # send corrections to drone
                    drone.rc(applied_hor_cor, -applied_ver_cor, applied_hor_cor, -applied_z_cor)
                    stabilized_since_last_iteration = False

                # set current error as previous and add to integral for next interation
                previous_error = [horizontal_error, vertical_error, z_error]
                integral_error[0] += horizontal_error
                integral_error[1] += vertical_error
                integral_error[2] += z_error


            if not manual_control and not stabilized_since_last_iteration:
                drone.stabilize()
                stabilized_since_last_iteration = True

        # display the resulting frame
        cv2.imshow('frame', frame)

        end_frame_time = time.time()
        
        manual_control = False
        # get keyboard key entry
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # button 'q' to quit
            follow = False
            drone.stabilize()
            break
        elif key == ord('t') and not airborne:
            print('Taking off!')
            drone.takeoff()
            drone.stabilize()
            stabilized_since_last_iteration = True
            airborne = True
        elif key == ord('l') and airborne:
            print('Landing!')
            follow = False
            drone.land()
            airborne = False
        elif key == ord('f') and not follow:
            print('Following!')
            follow = True
        elif key == ord('f') and follow:
            print('Stopped following!')
            drone.stabilize()
            stabilized_since_last_iteration = True
            follow = False
        elif key == ord('a'):
            drone.rc(0, 0, -50, 0)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('d'):
            drone.rc(0, 0, 50, 0)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('w'):
            drone.rc(0, 0, 0, 50)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('s'):
            drone.rc(0, 0, 0, -50)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('z'):
            drone.rc(-50, 0, 0, 0)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('x'):
            drone.rc(50, 0, 0, 0)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('c'):
            drone.rc(0, 50, 0, 0)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('v'):
            drone.rc(0, -50, 0, 0)
            stabilized_since_last_iteration = False
            manual_control = True
        elif key == ord('e'):
            drone.stabilize()
            stabilized_since_last_iteration = True

        end = time.time()
        print((end - start) * 1000, (end_frame_time - frame_time) * 1000)

    if airborne:
        print('Landing!')
        follow = False
        drone.land()
    
    # Destroy drone
    drone.destroy()
    # Destroy all the windows
    cv2.destroyAllWindows()

async def run_debug(drone_name):
    await debug.run_debug(drone_name)