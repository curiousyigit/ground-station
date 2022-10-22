import cv2
import numpy
from djitellopy import Tello as TelloPY

from drones.drone import DroneInterface

class Tello(DroneInterface):
    def initialize(self):
        self.tello = TelloPY()
        self.tello.connect()
        self.stabilize()
        print(f'Battery: {self.get_battery()}')

    def get_battery(self):
        return self.tello.get_battery()

    def get_distance_factor(self):
        return 1000
 
    def get_pids(self):
        #[heading, altitude, forward_backward]
        #[[kp, ki, kd], [kp, ki, kd], [kp, ki, kd]]
        return [[0.15, 0, 0.03], [0.15, 0, 0.05], [0.2, 0, 0.05]]

    def initialize_video_feed(self, width=960, height=720):
        # acceptable 1280x720 & 852x480
        if [width, height] not in [[960, 720]]:
            raise Exception('Unsupported video resolution for tello!')

        self.video_resolution = [width, height]
        self.tello.streamon()

    def get_video_frame(self):
        frame = self.tello.get_frame_read().frame
        return frame

    def get_video_resolution(self):
        return self.video_resolution[0], self.video_resolution[1]

    def takeoff(self):
        self.tello.takeoff()

    def stabilize(self):
        self.rc(0, 0, 0, 0)

    def up(self, distance_cm):
        self.tello.move_up(distance_cm)

    def up(self, distance_cm):
        self.tello.move_up(distance_cm)

    def down(self, distance_cm):
        self.tello.move_down(distance_cm)

    def left(self, distance_cm):
        self.tello.move_left(distance_cm)

    def right(self, distance_cm):
        self.tello.move_right(distance_cm)
    
    def turn_left(self, degrees):
        self.tello.rotate_counter_clockwise(degrees * 10)

    def turn_right(self, degrees):
        self.tello.rotate_clockwise(degrees * 10)

    def rc(self, yaw, vertical, left_right, forward_backward):
        self.yaw_velocity = int(numpy.clip(yaw, -100, 100))
        self.up_down_velocity = int(numpy.clip(vertical, -100, 100))
        self.left_right_velocity = int(numpy.clip(left_right, -100, 100))
        self.forward_back_velocity = int(numpy.clip(forward_backward, -100, 100))
        self.tello.send_rc_control(self.left_right_velocity, self.forward_back_velocity, self.up_down_velocity, self.yaw_velocity)

    def special_maneuver(self, maneuver):
        return super().special_maneuver()

    def land(self):
        self.tello.land()

    def destroy(self):
        self.tello.streamoff()
        # self.tello.end()