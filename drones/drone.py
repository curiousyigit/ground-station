from abc import ABC, abstractmethod, abstractproperty

def get_drone(drone_name):
    acceptable_drones = ['dummy', 'tello']

    if drone_name in acceptable_drones:
        print('Drone: '+drone_name)

        if drone_name == 'dummy':
            from drones.dummyDrone.dummyDrone import DummyDrone
            return DummyDrone()
        elif drone_name == 'tello':
            from drones.tello.tello import Tello
            return Tello()
            
    else:
        print('Error: Drone '+drone_name+' not supported!')
        quit()

class DroneInterface(ABC):
    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def get_battery(self):
        pass

    @abstractmethod
    def get_pids(self):
        #[heading, altitude, left_right, forward_backward]
        #[[kp, ki, kd], [kp, ki, kd], [kp, ki, kd], [kp, ki, kd]]
        pass

    @abstractmethod
    def initialize_video_feed(self, width, height):
        pass

    @abstractmethod
    def get_video_frame(self):
        pass

    @abstractmethod
    def get_video_resolution(self):
        pass

    @abstractmethod
    def takeoff(self):
        pass

    @abstractmethod
    def stabilize(self):
        pass

    @abstractmethod
    def up(self, distance_cm):
        pass

    @abstractmethod
    def down(self, distance_cm):
        pass

    @abstractmethod
    def left(self, distance_cm):
        pass

    @abstractmethod
    def right(self, distance_cm):
        pass

    @abstractmethod
    def turn_left(self, degrees):
        pass

    @abstractmethod
    def turn_right(self, degrees):
        pass

    @abstractmethod
    def rc(self, yaw, vertical, left_right, forward_backward):
        pass

    @abstractmethod
    def special_maneuver(self, maneuver):
        pass

    @abstractmethod
    def land(self):
        pass

    @abstractmethod
    def destroy(self):
        pass

    @abstractmethod
    def debug(self):
        pass