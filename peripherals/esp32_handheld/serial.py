import serial
import threading

class Serial():
    def __init__(self, port: str, baud_rate: int):
        self.port = port
        self.baud_rate = baud_rate

    def connect(self, handle_disconnect, handle_rx):
        self.handle_disconnect = handle_disconnect
        self.handle_rx = handle_rx
        self.client = serial.Serial(self.port, self.baud_rate, timeout=50)
        self.connected = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    def disconnect(self):
        self.connected = False
        self.client.close()
        self.handle_disconnect()

    def write(self, str):
        self.client.write(bytes(str, 'utf-8'))

    def rx_loop(self):
        while self.connected:
            if self.client.in_waiting:
                str = self.client.readline().decode('utf-8').replace('SERIAL OUT: ', '').strip()
                self.handle_rx(str)