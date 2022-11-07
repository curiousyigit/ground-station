import time
import asyncio

from drones.dummyDrone.dummyDrone import DummyDrone
import resources.console_tools as ct

async def run_debug(drone: DummyDrone):
    print('-- Debugging Drone: Dummy:-')

    # initialization
    drone.initialize()
    print(f'{ct.colors.GREEN}---- Initialization: (Passed){ct.END}')

    # get battery
    start = time.time()
    battery = drone.get_battery()
    end = time.time()
    if battery is not None and (type(battery) == int or type(battery) == float):
        print(f'    {ct.colors.GREEN}---- (Passed) Battery: {battery}, response time: {int((end - start) / 1000)} ms{ct.END}')
    else:
        print(f'    {ct.colors.RED}---- (Failed) Battery: {battery}, response time: {int((end - start) / 1000)} ms{ct.END}')

    # video
    drone.initialize_video_feed()
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

    await asyncio.sleep(0)