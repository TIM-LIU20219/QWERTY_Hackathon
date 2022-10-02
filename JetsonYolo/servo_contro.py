import pyfirmata
import time

def grab_and_put():
    l_grab = []
    l_loose = []
    for i in range(170):
        l_grab.append(i)
        l_loose.append(170 - i)

    board = pyfirmata.Arduino('/dev/ttyACM0')
    servo = board.get_pin('d:9:s')

    # set to 0
    servo.write(0)
    time.sleep(3)
    # increase
    for i in range(170): # half round for 180
        servo.write(l_grab[i])
        time.sleep(0.05)
        print(l_grab[i])

    time.sleep(10)

    # change status
    for i in range(170):
        servo.write(l_loose[i])
        time.sleep(0.05)
        print(l_loose[i])




if __name__ == "__main__":
    grab_and_put()
