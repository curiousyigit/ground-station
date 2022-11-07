import sys
import getopt
import asyncio

acceptableModes = ['face', 'body', 'combined.ble', 'combined.serial', 'prototype.ble', 'prototype.serial']
acceptableModes.extend([mode + '.debug' for mode in acceptableModes])

try:
    arg_val_list, left_over_args = getopt.getopt(sys.argv[1:], 'd:m', ['drone =', 'mode ='])
except:
    print('Error: Wrong syntax! Example syntax: python main.py --drone=tello --mode=face, python main.py --drone=dummy --mode=hand.debug')
    quit()

options = {}

for combination in arg_val_list:
    options[combination[0].strip()] = combination[1].strip()

mode = options['--mode']
drone = options['--drone']

if mode in acceptableModes:
    if mode.startswith('face'):
        from modes.face import face
        asyncio.run(face.run_debug(drone) if mode == 'face.debug' else face.run(drone))
        quit()
    elif mode.startswith('combined.ble'):
        from modes.combined import combined
        # asyncio.run(combined.run_debug(drone, True) if mode == 'combined.ble.debug' else combined.run(drone, True))
        if mode == 'combined.ble.debug':
            combined.run_debug(drone, True)
        else:
            combined.run(drone, True)
        quit()
    elif mode.startswith('combined.serial'):
        from modes.combined import combined
        asyncio.run(combined.run_debug(drone, False) if mode == 'combined.serial.debug' else combined.run(drone, False))
        quit()
    elif mode.startswith('prototype'):
        from modes.prototype import prototype
        if mode == 'prototype.serial.debug':
            prototype.run_debug(drone, mode.startswith('prototype.ble'))
        else:
            prototype.run(drone, mode.startswith('prototype.ble'))
        quit()
    elif mode.startswith('body'):
        if mode == 'body.debug':
            from modes.body import debug
            asyncio.run(debug.run_debug(drone))
        else:
            from modes.body import body
            asyncio.run(body.run(drone))
        quit()
else:
    print('Error: Mode '+mode+' unsupported!')