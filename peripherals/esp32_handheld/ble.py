from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

class BLE():
    def __init__(self, service_uuid: str, rx_characteristic_uuid: str, tx_characteristic_uuid: str):
        self.service_uuid = service_uuid
        self.rx_characteristic_uuid = rx_characteristic_uuid
        self.tx_characteristic_uuid = tx_characteristic_uuid

    async def connectToServiceUuid(self, handle_disconnect = None):
        self.handle_disconnect = handle_disconnect

        self.device = await BleakScanner.find_device_by_filter(self._match_nus_uuid)
        self.client = BleakClient(self.device, disconnected_callback=self._handle_disconnect)
        await self.client.connect()
        return self.device

    async def connectByMacAddress(self, macAddr: str, handle_disconnect = None):
        self.handle_disconnect = handle_disconnect

        self.device = await BleakScanner.find_device_by_address(macAddr)
        self.client = BleakClient(self.device, disconnected_callback=self._handle_disconnect)
        await self.client.connect()
        return self.device

    async def disconnect(self):
        await self.client.disconnect()

    async def enable_handle_incoming(self, handle_rx = None):
        self.handle_rx = handle_rx

        await self.client.start_notify(self.tx_characteristic_uuid, self._handle_rx)

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
            raise Exception('Data too large to send over BLE!')

    def _match_nus_uuid(self, device: BLEDevice, adv: AdvertisementData):
        if self.service_uuid.lower() in [s.lower() for s in adv.service_uuids]:
            return True

        return False

    def _handle_disconnect(self, _: BleakClient):
        if self.handle_disconnect:
            self.handle_disconnect()
        else:
            print('Device was disconnected.')

    def _handle_rx(self, _: BleakGATTCharacteristic, data: bytearray):
        if self.handle_rx:
            self.handle_rx(data.decode('utf-8').strip())
        else:
            print('Received: ', data)