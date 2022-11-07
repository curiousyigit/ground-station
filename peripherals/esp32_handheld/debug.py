import time
import asyncio

from peripherals.esp32_handheld.esp32_handheld import ESP32HandHeld
import resources.console_tools as ct

class ESP32HandHeldDebug():
    def __init__(self, esp32_handheld: ESP32HandHeld):
        self.ping_at = None
        self.pong_at = None
        self.continuous_times = []
        self.esp32_handheld = esp32_handheld
        esp32_handheld.handle_rx = self.handle_rx
        esp32_handheld.handle_disconnect = self.handle_disconnect

    def handle_rx(self, str):
        if str == 'pong':
            self.pong_at = time.time()
            print(f'    {ct.colors.GREEN}---- (Passed) Receive: Pong, round trip latency: ~{int((self.pong_at - self.ping_at) * 1000)} ms, one-way latency: ~{int((self.pong_at - self.ping_at) * 1000) / 2} ms{ct.END}')
        elif str.startswith('calb='):
            self.continuous_times.append(time.time())

    def handle_disconnect(self):
        pass
    

async def run_debug(esp32_handheld: ESP32HandHeld):
    mode = 'BLE' if esp32_handheld.mode == 'ble' else 'Serial'
    print(f'-- Debugging ESP32 Handheld Peripheral ({mode}):-')

    dbg = ESP32HandHeldDebug(esp32_handheld)

    # connect
    start = time.time()
    await dbg.esp32_handheld.connect()
    end = time.time()
    print(f'    {ct.colors.GREEN}---- (Passed) Connection: Connected in ~{int((end - start) * 1000)} ms{ct.END}')


    # send
    dbg.ping_at = time.time()
    await dbg.esp32_handheld.send('ping')
    print(f'    {ct.colors.GREEN}---- (Passed) Send: Ping{ct.END}')

    await asyncio.sleep(3)

    if dbg.pong_at is None:
        print(f'    {ct.colors.RED}---- (Failed) Receive: Pong not received in max time of 3 s{ct.END}')


    # continuous refresh rate
    await dbg.esp32_handheld.send('get_continuous')
    await asyncio.sleep(10)
    await dbg.esp32_handheld.send('get_stop')
    await asyncio.sleep(3)
    refresh_rate = 0
    for i, t in enumerate(dbg.continuous_times):
        if i == 0:
            continue
        else:
            refresh_rate += t - dbg.continuous_times[i - 1]
    refresh_rate /= len(dbg.continuous_times) - 1
    if len(dbg.continuous_times) < 2:
        print(f'    {ct.colors.RED}---- (Failed) Continuous One-way: Didn\'t receive enough packets{ct.END}')
    else:
        print(f'    {ct.colors.GREEN}---- (Passed) Continuous One-way: Received {len(dbg.continuous_times)} packets with an avg refresh rate of ~{int((refresh_rate) * 1000)} ms{ct.END}')

    # disconnect
    start = time.time()
    await dbg.esp32_handheld.disconnect()
    end = time.time()
    print(f'    {ct.colors.GREEN}---- (Passed) Disonnected in ~{int((end - start) * 1000)} ms{ct.END}')
