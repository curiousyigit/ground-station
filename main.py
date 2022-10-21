import sys
import getopt

acceptableModes = ['face', 'hand']

try:
    arg_val_list, left_over_args = getopt.getopt(sys.argv[1:], 'd:m:', ['drone =', 'mode ='])
except:
    print('Error: Wrong syntax! Example syntax: python main.py --drone=tello --mode=face')
    quit()

options = {}

for combination in arg_val_list:
    options[combination[0].strip()] = combination[1].strip()

mode = options['--mode']
drone = options['--drone']

if mode in acceptableModes:
    if mode == 'face':
        from modes.face import face
        face.run(drone)
        quit()
else:
    print('Error: Mode '+mode+' unsupported!')