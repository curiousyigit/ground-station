import cv2

from drones.drone import get_drone
import modes.face.helpers as face

def run(drone_name):
    print('Mode: face')

    desired_face_to_video_ratio = 0.25 # the ratio of the area of the face to the area of the video (for distance)
    pid = [0.5, 0.5, 0] # kd, kp, ki
    previous_error = [0, 0, 0] # heading, altitude, distance (points relative to frame)

    # initialize drone and its camera
    drone = get_drone(drone_name)
    drone.initialize()
    drone.initialize_video_feed()
    w, h = drone.get_video_resolution()

    airborne = False
    follow = False

    while True:
        # capture the video frame by frame
        frame = drone.get_video_frame()

        if frame is not None:
            # find nearest face and get it's coordinates and area
            frame, face_center, face_area = face.find_nearest_face(frame, (0, 0 if follow else 255, 255))

            if airborne:
                if face_center:
                    if follow:
                        # get error resulting from PID controller calculations
                        error = face.calculate_tracking_error(face_center, desired_face_to_video_ratio, w, h, pid, previous_error)
                        heading_error = int(error[0])
                        altitude_error = int(error[1])
                        speed_error = int(error[2])

                        # send corrections to drone
                        drone.move(heading_error, altitude_error, speed_error)

                        # set current error as previous for next interation
                        previous_error = error
                    else:
                        drone.stabilize()
                else:
                    drone.stabilize()
                    print('Warn: No face detected!')

        # display the resulting frame
        cv2.imshow('frame', frame)
        
        # get keyboard key entry
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # button 'q' to quit
            break
        elif key == ord('t') and not airborne:
            print('Taking off!')
            drone.takeoff()
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