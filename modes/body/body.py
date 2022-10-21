from time import sleep
import cv2

from drones.drone import get_drone
import modes.body.helpers as body

def run(drone_name):
    print('Mode: body')

    desired_body_to_video_ratio = 0.25 # the ratio of the area of the body to the area of the video (for distance)
    previous_error = [0, 0, 0] # heading, altitude, distance (points relative to frame)

    # initialize drone and its camera
    drone = get_drone(drone_name)
    drone.initialize()
    drone.initialize_video_feed()
    w, h = drone.get_video_resolution()
    half_w = int(w * 0.5)
    half_h = int(h * 0.5)

    airborne = False
    follow = False

    distance_factor = drone.get_distance_factor()
    pids = drone.get_pids()

    frame_count = 0
    battery = drone.get_battery()
    battery_x = int(w * 0.05)
    battery_y = int(h * 0.9)
    while True:
        frame_count = frame_count + 1
        if frame_count >= 150:
            battery = drone.get_battery()
            frame_count = 0

        # capture the video frame by frame
        frame = drone.get_video_frame()

        if frame is not None:
            cv2.putText(frame, f'Battery: {battery}', (battery_x, battery_y), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255))

            # center locator for debugging
            cv2.line(frame, (half_w, half_h - 10), (half_w, half_h +10), (0, 255, 255), 2)
            cv2.line(frame, (half_w - 10, half_h), (half_w + 10, half_h), (0, 255, 255), 2)
            
            # find nearest body and get it's coordinates and area
            frame, body_center, body_area = body.find_nearest_body(frame, (0, 0 if follow else 255, 255))

            if airborne or 1==1:
                if body_center:
                    if follow:
                        # get error resulting from PID controller calculations
                        error = body.calculate_tracking_error(body_center, desired_body_to_video_ratio, distance_factor, w, h, pids, previous_error)
                        heading_error = int(error[0])
                        altitude_error = int(error[1])
                        forward_backward_error = int(error[2])
                        # print(error)
                        
                        # send corrections to drone
                        drone.rc(-heading_error, altitude_error, 0, forward_backward_error)

                        # set current error as previous for next interation
                        previous_error = error
                    else:
                        drone.stabilize()
                else:
                    drone.stabilize()
                    print('Warn: No body detected!')

        # display the resulting frame
        cv2.imshow('frame', frame)
        
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
            drone.rc(0, 50, 0, 0)
            sleep(2)
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
            follow = False

    if airborne:
        print('Landing!')
        follow = False
        drone.land()
    
    # Destroy drone
    drone.destroy()
    # Destroy all the windows
    cv2.destroyAllWindows()