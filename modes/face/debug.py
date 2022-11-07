from drones.drone import get_drone
import modes.face.helpers as face
import resources.console_tools as ct
from peripherals.esp32_handheld.esp32_handheld import ESP32HandHeld

UART_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
UART_RX_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
UART_TX_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

async def run_debug(drone_name):
    # drone
    drone = get_drone(drone_name)
    await drone.debug()

    # esp32 handheld peripheral serial mode
    serial = ESP32HandHeld('serial', port='/dev/ttyUSB0', baud_rate=115200)
    await serial.debug()

    # esp32 handheld peripheral ble mode
    ble = ESP32HandHeld('ble', service_uuid=UART_SERVICE_UUID, rx_characteristic_uuid=UART_RX_CHAR_UUID, tx_characteristic_uuid=UART_TX_CHAR_UUID)
    await ble.debug()