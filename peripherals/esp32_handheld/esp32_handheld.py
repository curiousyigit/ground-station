import asyncio

from peripherals.esp32_handheld import ble, serial

class ESP32HandHeld():
    def __init__(self, mode: str, handle_rx = None, handle_disconnect = None, service_uuid: str = None, rx_characteristic_uuid: str = None, tx_characteristic_uuid: str = None, port: str = None, baud_rate: int = None):
        self.mode = mode
        self.handle_rx = handle_rx
        self.handle_disconnect = handle_disconnect

        if mode == 'ble' and service_uuid and rx_characteristic_uuid and tx_characteristic_uuid:
            self.service_uuid = service_uuid
            self.rx_characteristic_uuid = rx_characteristic_uuid
            self.tx_characteristic_uuid = tx_characteristic_uuid
            self.ble_client = ble.BLE(self.service_uuid, self.rx_characteristic_uuid, self.tx_characteristic_uuid)
        elif mode == 'serial' and port and baud_rate:
            self.port = port
            self.baud_rate = baud_rate
            self.serial_client = serial.Serial(self.port, self.baud_rate)
        else:
            raise Exception(f'Unsupported mode "{mode}" or args for ESP32 Handheld Peripheral!')

    async def connect(self):
        if self.mode == 'ble':
            if(await self.ble_client.connectToServiceUuid(self.handle_disconnect)):
                await self.ble_client.enable_handle_incoming(self.handle_rx)
        else:
            self.serial_client.connect(self.handle_disconnect, self.handle_rx)

    async def disconnect(self):
        if self.mode == 'ble':
            await self.ble_client.disconnect()
        else:
            self.serial_client.disconnect()

    async def send(self, str):
        if self.mode == 'ble':
            str = bytes(str, 'utf-8')
            await self.ble_client.notify(str)
        else:
            self.serial_client.write(str)
        await asyncio.sleep(0.05)

    async def debug(self):
        from peripherals.esp32_handheld import debug
        await debug.run_debug(self)