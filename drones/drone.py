from abc import ABC, abstractmethod

def get_drone(drone_name):
    acceptable_drones = ['dummy', 'tello']

    if drone_name in acceptable_drones:
        print('Drone: '+drone_name)

        if drone_name == 'dummy':
            from drones.dummyDrone.dummyDrone import DummyDrone
            return DummyDrone()
    else:
        print('Error: Drone '+drone_name+' not supported!')
        quit()

class DroneInterface(ABC):
    @abstractmethod
    def initialize(self):
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
    def move(self, heading, altitude, speed):
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