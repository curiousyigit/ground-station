import threading
import time
import asyncio
import cv2

from drones.drone import get_drone
from peripherals.esp32_handheld.esp32_handheld import ESP32HandHeld

class CombinedMode():
    cps = None # control loop per second
    previous_cpss = []
    vps = None # video loop per second
    previous_vpss = []

    quit = False
    airborne = False
    stabilized_since_last_iteration = True
    follow_face = False

    frame = None
    w, h = 0, 0
    font_size = 1
    battery = None

    peripheral_connected = False
    peripheral_manual_control = False
    led1, led2, led3 = None, None, None
    btn1, btn2, btn3, btn4 = None, None, None, None
    calb_system, calb_gyro, calb_accel, calb_mag = None, None, None, None
    yaw_raw, yaw_biased, roll_raw, roll_biased, pitch_raw, pitch_biased = None, None, None, None, None, None

    def __init__(self, drone_name, peripheral_mode):
        print(f'Mode: Combined ({peripheral_mode})')
        self.drone = get_drone(drone_name)
        self.peripheral = ESP32HandHeld(peripheral_mode, self.handle_peripheral_rx, self.handle_peripheral_disconnect,
            port = '/dev/ttyUSB0', baud_rate = 115200,
            service_uuid = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E',
            rx_characteristic_uuid = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E',
            tx_characteristic_uuid = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
        )

    async def run(self):
        self.drone.initialize()
        self.battery = self.drone.get_battery()

        control_thread = threading.Thread(target=asyncio.run, args=(self.control_loop(),))
        control_thread.start()

        video_thread = threading.Thread(target=asyncio.run, args=(self.video_loop(),))
        video_thread.start()

        control_thread.join() # wait to finish
        video_thread.join() # wait to finish

    async def control_loop(self):
        await self.peripheral.connect()
        self.peripheral_connected = True
        print('cant get past this')

        self.pids = self.drone.get_pids()


        async def send_to_peripheral(str):
            if self.peripheral_connected:
                print('sending')
                await self.peripheral.send(str)
                print('sent')

        await send_to_peripheral('get_continuous')

        p_btn1 = self.btn1
        p_btn2 = self.btn2
        p_btn3 = self.btn3

        p_peripheral_connected = self.peripheral_connected
        await send_to_peripheral(f'led3={0 if self.peripheral_connected else 1}')

        while not self.quit:
            start = time.time()

            if self.peripheral_connected != p_peripheral_connected:
                await send_to_peripheral(f'led3={0 if self.peripheral_connected else 1}')
                p_peripheral_connected = self.peripheral_connected

            if self.btn1 != p_btn1:
                await send_to_peripheral(f'led1={self.btn1}')
                p_btn1 = self.btn1

            if self.frame is not None:
                cv2.imshow('Combined', self.frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # button 'q' to quit
                    self.follow = False
                    self.drone.stabilize()
                    self.quit = True
                elif key == ord('t') and not self.airborne:
                    print('Taking off!')
                    self.drone.takeoff()
                    self.drone.stabilize()
                    self.stabilized_since_last_iteration = True
                    self.airborne = True
                elif key == ord('l') and self.airborne:
                    print('Landing!')
                    self.follow = False
                    self.drone.land()
                    self.airborne = False
                elif key == ord('f') and not self.follow:
                    print('Following!')
                    self.follow = True
                elif key == ord('f') and self.follow:
                    print('Stopped following!')
                    self.drone.stabilize()
                    self.stabilized_since_last_iteration = True
                    self.follow = False
                elif key == ord('a'):
                    self.drone.rc(0, 0, -50, 0)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('d'):
                    self.drone.rc(0, 0, 50, 0)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('w'):
                    self.drone.rc(0, 0, 0, 50)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('s'):
                    self.drone.rc(0, 0, 0, -50)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('z'):
                    self.drone.rc(-50, 0, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('x'):
                    self.drone.rc(50, 0, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('c'):
                    self.drone.rc(0, 50, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('v'):
                    self.drone.rc(0, -50, 0, 0)
                    self.stabilized_since_last_iteration = False
                    self.manual_control = True
                elif key == ord('e'):
                    self.drone.stabilize()
                    self.stabilized_since_last_iteration = True
            else:
                time.sleep(0.05)

            end = time.time()
            cps = int(1 / (end - start))
            self.previous_cpss.append(cps)
            if len(self.previous_cpss) > 120:
                self.previous_cpss.pop(0)
            self.cps = int(sum(self.previous_cpss) / len(self.previous_cpss))

        await self.peripheral.disconnect()
        self.drone.destroy()
        cv2.destroyAllWindows()

    async def video_loop(self):
        self.drone.initialize_video_feed()
        self.w, self.h = self.drone.get_video_resolution()
        self.half_w = self.w // 2
        self.half_h = self.h // 2

        while not self.quit:
            start = time.time()

            frame = self.drone.get_video_frame()
            if frame is not None:
                battery_x = int(self.w * 0.03)
                battery_y = int(self.h * 0.06)
                overlay = frame.copy()
                alpha = 0.5
                cv2.rectangle(overlay, (battery_x - 10, battery_y - 20), (battery_x + 310, battery_y + 80), (0, 0, 0), -1)
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
                cv2.putText(frame, f'Battery: {self.battery}, CPS: {self.cps}, VPS: {self.vps}', (battery_x, battery_y), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255, 255))
                peripheral_status = 'Connected' if self.peripheral_connected else 'Disconnected'
                cv2.putText(frame, f'Peripheral: {peripheral_status}', (battery_x, battery_y + 20), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255 if self.peripheral_connected else 0, 255))
                if self.peripheral_connected:
                    yrp = str(self.yaw_biased) + ', ' + str(self.roll_biased) + ', ' + str(self.pitch_biased)
                    cv2.putText(frame, f'Yaw, roll, pitch: {yrp}', (battery_x, battery_y + 40), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255, 255))
                    btns = str(self.btn1) + ', ' + str(self.btn2) + ', ' + str(self.btn3) + ', ' + str(self.btn4)
                    cv2.putText(frame, f'Buttons: {btns}', (battery_x, battery_y + 60), cv2.FONT_HERSHEY_PLAIN, self.font_size, (0, 255, 255))

                self.frame = frame

            end = time.time()
            vps = int(1 / (end - start))
            self.previous_vpss.append(vps)
            if len(self.previous_vpss) > 60:
                self.previous_vpss.pop(0)
            self.vps = int(sum(self.previous_vpss) / len(self.previous_vpss))

    def handle_peripheral_rx(self, str):
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

    def handle_peripheral_disconnect(self):
        self.peripheral_connected = False




def run(drone_name, peripheral_ble: bool):
    md = CombinedMode(drone_name, 'ble' if peripheral_ble else 'serial')
    asyncio.get_event_loop().run_until_complete(md.run())

def run_debug(drone_name, peripheral_ble: bool):
    pass