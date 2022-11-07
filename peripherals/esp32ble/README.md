# ESP32 BLE Peripheral
## Sends
- `get_once` - Request device to send readings one time
- `get_continuous` - Request device to keep sending readings non-stop
- `get_stop` - Request device to stop sending continuously

## Receives
- `gyro_calb=3,3,3,3` - Gyroscope calibration quality (1-3). Syntax: system,gyro,accel,mag
- `gyrb=360,-180,-180` - Biased yaw,roll,pitch
- `gyrr=360,-234,129` - Raw yaw,roll,pitch
- `leds=1,0,1` - LED states, Syntax: LED1,LED2,LED3
- `btns=1,1,0,0` - Button states, Syntax: BTN1,BTN2,BTN3,BTN4