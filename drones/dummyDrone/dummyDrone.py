import cv2

from drones.drone import DroneInterface

class DummyDrone(DroneInterface):
    def initialize(self):
        return super().initialize()

    def get_battery(self):
        return 100

    def get_pids(self):
        return [[0.5, 0.001, 0], [0.5, 0.001, 0], [0.5, 0.001, 0], [0.5, 0.001, 0]]

    def initialize_video_feed(self, width=640, height=480):
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

    def stabilize(self):
        return super().stabilize()

    def up(self, distance_cm):
        return super().up(distance_cm)

    def down(self, distance_cm):
        return super().down(distance_cm)

    def left(self, distance_cm):
        return super().left(distance_cm)

    def right(self, distance_cm):
        return super().right(distance_cm)
    
    def turn_left(self, degrees):
        return super().turn_left(degrees)

    def turn_right(self, degrees):
        return super().turn_right(degrees)

    def rc(self, yaw, vertical, left_right, forward_backward):
        print(f'rc yaw: {yaw}, vertical: {vertical}, left_right: {left_right}, forward_backward: {forward_backward}')

    def special_maneuver(self, maneuver):
        return super().special_maneuver(maneuver)

    def land(self):
        return super().land()

    def destroy(self):
        self.video_capture.release()

    async def debug(self):
        import drones.dummyDrone.debug as debug
        await debug.run_debug(self)