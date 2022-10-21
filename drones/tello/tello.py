from djitellopy import Tello as TelloPY

from drones.drone import DroneInterface

class Tello(DroneInterface):
    def initialize(self):
        self.tello = TelloPY()
        self.tello.connect()
        self.stabilize()
        print(self.tello.get_battery())
        self.tello.streamoff()

    def initialize_video_feed(self, width=852, height=480):
        # acceptable 1280x720 & 852x480
        if [width, height] not in [[852, 480], [1280, 720]]:
            raise Exception('Unsupported video resolution for tello!')

        self.video_resolution = [width, height]
        self.tello.set_video_resolution(TelloPY.RESOLUTION_480P if height == 480 else TelloPY.RESOLUTION_720P)
        self.tello.streamon()

    def get_video_frame(self):
        frame = self.tello.get_frame_read()
        return frame

    def get_video_resolution(self):
        return self.video_resolution[0], self.video_resolution[1]

    def takeoff(self):
        self.tello.takeoff()

    def stabilize(self):
        self.move(0, 0, 0, 0)

    def move(self, yaw, vertical, left_right, forward_backward):
        self.yaw_velocity = yaw
        self.up_down_velocity = vertical
        self.left_right_velocity = left_right
        self.for_back_velocity = forward_backward
        self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity, self.yaw_velocity)

    def special_maneuver(self, maneuver):
        return super().special_maneuver()

    def land(self):
        self.tello.land()

    def destroy(self):
        self.tello.streamoff()
        self.tello.end()