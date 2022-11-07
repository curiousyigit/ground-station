import time
import cv2
from importlib import reload

from drones.drone import get_drone
import modes.face.helpers as face

def run(drone_name):
    print('Mode: Debug')

    print('-- Debugging BLE to ESP32 Handheld Peripheral')
    
    print('---- Connection: ')