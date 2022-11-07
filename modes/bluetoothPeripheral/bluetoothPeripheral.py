import asyncio
from asyncer import asyncify
import cv2
import time

from peripherals.esp32ble.esp32ble import ESP32BLE
from drones.drone import get_drone

UART_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
UART_RX_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
UART_TX_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

class BluetoothPeripheralMode():
    peripheral_connected = False

    led1 = 0
    led2 = 0
    led3 = 0
    btn1 = 0
    btn2 = 0
    btn3 = 0
    btn4 = 0
    calb_system = 0
    calb_gyro = 0
    calb_accel = 0
    calb_mag = 0
    yaw_raw = 0
    yaw_biased = 0
    roll_raw = 0
    roll_biased = 0
    pitch_raw = 0
    pitch_biased = 0

    def __init__(self, drone_name):
        self.drone = get_drone(drone_name)

    async def run(self):
        print('Mode: Bluetooth Peripheral')

        esp32ble = ESP32BLE(UART_SERVICE_UUID, UART_RX_CHAR_UUID, UART_TX_CHAR_UUID)
        if(await esp32ble.connectToServiceUuid()):
            self.peripheral_connected = True
            async with esp32ble.getClient(self.handle_disconnect):
                await esp32ble.enable_handle_incoming(self.handle_rx)

                await esp32ble.notify(b'get_continuous')

                await asyncify(self.control_loop)()

    def control_loop(self):
        # initialize drone and its camera
        self.drone.initialize()
        self.drone.initialize_video_feed()
        w, h = self.drone.get_video_resolution()
        half_w = int(w * 0.5)
        half_h = int(h * 0.5)

        airborne = False

        frame_count = 0
        battery = self.drone.get_battery()
        battery_x = int(w * 0.05)
        battery_y = int(h * 0.05)
        while True:
            start = time.time()

            frame_count = frame_count + 1
            if frame_count >= 150:
                battery = self.drone.get_battery()
                frame_count = 0

            frame_time = time.time()
            # capture the video frame by frame
            frame = self.drone.get_video_frame()

            if frame is not None:
                cv2.putText(frame, f'Battery: {battery}', (battery_x, battery_y), cv2.FONT_HERSHEY_PLAIN, 0.75, (0, 255, 255))
                peripheral_status = 'Connected' if self.peripheral_connected else 'Disconnected'
                cv2.putText(frame, f'Bluetooth Peripheral: {peripheral_status}', (battery_x, battery_y * 2), cv2.FONT_HERSHEY_PLAIN, 0.75, (0, 255 if self.peripheral_connected else 0, 255))
                if self.peripheral_connected:
                    yrp = str(self.yaw_biased) + ', ' + str(self.roll_biased) + ', ' + str(self.pitch_biased)
                    cv2.putText(frame, f'Yaw, roll, pitch: {yrp}', (battery_x, battery_y * 3), cv2.FONT_HERSHEY_PLAIN, 0.75, (0, 255, 255))
                    btns = str(self.btn1) + ', ' + str(self.btn2) + ', ' + str(self.btn3) + ', ' + str(self.btn4)
                    cv2.putText(frame, f'Buttons: {btns}', (battery_x, battery_y * 4), cv2.FONT_HERSHEY_PLAIN, 0.75, (0, 255, 255))

                # center locator for debugging
                cv2.line(frame, (half_w, half_h - 10), (half_w, half_h +10), (0, 255, 255), 2)
                cv2.line(frame, (half_w - 10, half_h), (half_w + 10, half_h), (0, 255, 255), 2)


            # display the resulting frame
            cv2.imshow('frame', frame)

            end_frame_time = time.time()
            
            # get keyboard key entry
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # button 'q' to quit
                self.drone.stabilize()
                break
            elif key == ord('t') and not airborne:
                print('Taking off!')
                self.drone.takeoff()
                self.drone.rc(0, 50, 0, 0)
                airborne = True
            elif key == ord('l') and airborne:
                print('Landing!')
                self.drone.land()
                airborne = False

            end = time.time()
            print((end - start) * 1000, (end_frame_time - frame_time) * 1000)

        if airborne:
            print('Landing!')
            self.drone.land()
        
        # Destroy drone
        self.drone.destroy()
        # Destroy all the windows
        cv2.destroyAllWindows()

    def handle_rx(self, _, data: bytearray):
        str = data.decode('utf-8')
        if str.startswith('leds') and len(str) == 10: # leds=1,1,1
            leds = str.split('=')[1].split(',')
            self.led1 = int(leds[0])
            self.led2 = int(leds[1])
            self.led3 = int(leds[2])
        elif str.startswith('btns') and len(str) == 12: # btns=1,1,1,1
            btns = str.split('=')[1].split(',')
            self.btn1 = int(btns[0])
            self.btn2 = int(btns[1])
            self.btn3 = int(btns[2])
            self.btn4 = int(btns[3])
        elif str.startswith('calb') and len(str) == 12: # calb=3,3,3,3
            calbs = str.split('=')[1].split(',')
            self.calb_system = int(calbs[0])
            self.calb_gyro = int(calbs[1])
            self.calb_accel = int(calbs[2])
            self.calb_mag = int(calbs[3])
        elif str.startswith('yrpr') and len(str) >= 10 and len(str) <= 19:
            vals = str.split('=')[1].split(',')
            self.yaw_raw = int(vals[0])
            self.roll_raw = int(vals[1])
            self.pitch_raw = int(vals[2])
        elif str.startswith('yrpb') and len(str) >= 10 and len(str) <= 19:
            vals = str.split('=')[1].split(',')
            self.yaw_biased = int(vals[0])
            self.roll_biased = int(vals[1])
            self.pitch_biased = int(vals[2])

    def handle_disconnect(self, _):
        self.peripheral_connected = False
        print('BLE device was disconnected.')

def run(drone_name):
    c = BluetoothPeripheralMode(drone_name)
    asyncio.run(c.run())