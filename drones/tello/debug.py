import time
import asyncio

from drones.tello.tello import Tello
import resources.console_tools as ct

async def run_debug(drone: Tello):
    print('-- Debugging Drone: Tello:-')

    # initialization
    drone.initialize()
    print(f'{ct.colors.GREEN}---- Initialization: (Passed){ct.END}')

    # get battery
    start = time.time()
    battery = drone.get_battery()
    end = time.time()
    if battery is not None and (type(battery) == int or type(battery) == float):
        print(f'    {ct.colors.GREEN}---- (Passed) Battery: {battery}, response time: {int((end - start) * 1000)} ms{ct.END}')
    else:
        print(f'    {ct.colors.RED}---- (Failed) Battery: {battery}, response time: {int((end - start) * 1000)} ms{ct.END}')

    # video
    drone.initialize_video_feed()
    await asyncio.sleep(10) # give tello some time
    total_frames = 0
    max_frames = 120
    start = time.time()
    while total_frames < max_frames:
        frame = drone.get_video_frame()
        total_frames += 1
    end = time.time()
    if frame is not None:
        print(f'    {ct.colors.GREEN}---- (Passed) Video: {total_frames} connection frames in {int((end - start) * 1000)} ms, {int((total_frames / (end - start)))} connection fps{ct.END}')
    else:
        print(f'    {ct.colors.RED}---- (Failed) Video: {total_frames} connection frames in {int((end - start) * 1000)} ms, {int((total_frames / (end - start)))} connection fps{ct.END}')

    # disconnect
    start = time.time()
    drone.destroy()
    end = time.time()
    print(f'    {ct.colors.GREEN}---- (Passed) Disonnected in ~{int((end - start) * 1000)} ms{ct.END}')
