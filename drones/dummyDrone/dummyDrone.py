import cv2

from drones.drone import DroneInterface

class DummyDrone(DroneInterface):
    def initialize(self):
        return super().initialize()

    def initialize_video_feed(self, width=680, height=420):
        self.video_capture = cv2.VideoCapture(0)
        self.video_resolution = [width, height]
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_video_frame(self):
        _, frame = self.video_capture.read()
        return frame

    def get_video_resolution(self):
        return self.video_resolution[0], self.video_resolution[1]

    def takeoff(self):
        return super().takeoff()

    def move(self, heading, altitude, speed):
        return super().move(heading, altitude, speed)

    def special_maneuver(self, maneuver):
        return super().special_maneuver()

    def land(self):
        return super().land()

    def destroy(self):
        self.video_capture.release()