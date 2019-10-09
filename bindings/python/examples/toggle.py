import gpiod
import time

chip = gpiod.Chip("gpiochip0")

line = chip.get_line(18)

# this will segault
# line.request()
line.request("gpiochip0", type=gpiod.LINE_REQ_DIR_OUT)

# when above line is missing, the error :
    #PermissionError: [Errno 1] Operation not permitted
# is not very helpful
for i in range(3):
    line.set_value(1)
    time.sleep(1)
    line.set_value(0)
    time.sleep(1)
# input()


# We could wrap a lot of this stuff into a simpler class
