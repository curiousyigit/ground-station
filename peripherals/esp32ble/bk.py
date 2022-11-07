import asyncio
import sys
from typing import Optional
import async_timeout

from bleak import AdvertisementDataFilter, BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class ESP32BLE():
    def __init__(self, service_uuid: str, rx_characteristic_uuid: str, tx_characteristic_uuid: str):
        self.service_uuid = service_uuid
        self.rx_characteristic_uuid = rx_characteristic_uuid
        self.tx_characteristic_uuid = tx_characteristic_uuid

    async def connectToServiceUuid(self):
        self.device = await BleakScanner.find_device_by_filter(self._match_nus_uuid)
        await self.getClient()
        return self.device

    async def connectByMacAddress(self, macAddr: str):
        self.device = await BleakScanner.find_device_by_address(macAddr)
        await self.getClient()
        return self.device

    async def getClient(self, handle_disconnect = None, handle_rx = None):
        if handle_disconnect is None:
            handle_disconnect = self._handle_disconnect
        if handle_rx is None:
            handle_rx = self._handle_rx

        print(self.device)

        # self.client = BleakClient(self.device, disconnected_callback=handle_disconnect)

        async with BleakClient(self.device, disconnected_callback=handle_disconnect) as client:

            self.client = client
            print(client)

            await client.start_notify(self.rx_characteristic_uuid, handle_rx)

            print('aquii 2')

        print('aqqui')

        return self.client

    def get_characteristic(self, characteristic_uuid: str, service_uuid: str = None):
        if service_uuid is None:
            service_uuid = self.service_uuid

        nus = self.client.services.get_service(service_uuid)
        return nus.get_characteristic(characteristic_uuid)

    async def notify(self, data):
        rx_characteristic = self.get_characteristic(self.rx_characteristic_uuid)

        if len(data) <= rx_characteristic.max_write_without_response_size:
            await self.client.write_gatt_char(rx_characteristic, data)
        else:
            raise Exception("Data too large to send over BLE!")

    def _match_nus_uuid(self, device: BLEDevice, adv: AdvertisementData):
        if self.service_uuid.lower() in [s.lower() for s in adv.service_uuids]:
            return True

        return False

    def _handle_disconnect(self, _: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()

    def _handle_rx(self, _: BleakGATTCharacteristic, data: bytearray):
        print("received:", data)
        # pass


async def uart_terminal():

    esp32ble = ESP32BLE(UART_SERVICE_UUID, UART_RX_CHAR_UUID, UART_TX_CHAR_UUID)
    if(await esp32ble.connectToServiceUuid()):
            print('connected')
            await esp32ble.notify("get_once")


        # async with await esp32ble.getClient() as client:

        #     print("Connected, start typing and press ENTER...")

        #     loop = asyncio.get_running_loop()

        #     while True:
        #         # This waits until you type a line and press ENTER.
        #         # A real terminal program might put stdin in raw mode so that things
        #         # like CTRL+C get passed to the remote device.
        #         data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

        #         # data will be empty on EOF (e.g. CTRL+D on *nix)
        #         if not data:
        #             break

        #         # some devices, like devices running MicroPython, expect Windows
        #         # line endings (uncomment line below if needed)
        #         # data = data.replace(b"\n", b"\r\n")
        #         data = data.replace(b"\n", b"")

        #         # Writing without response requires that the data can fit in a
        #         # single BLE packet. We can use the max_write_without_response_size
        #         # property to split the data into chunks that will fit.

        #         await esp32ble.notify(data)
        #         print("sent:", data)


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass