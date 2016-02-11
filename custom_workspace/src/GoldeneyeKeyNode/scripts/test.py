#import os
#import queue
#from pynput import keyboard
#
#q = queue.Queue()
#with keyboard.Listener(
#        on_press=lambda k: q.put_nowait((k, True)),
#        on_release=lambda k: q.put_nowait((k, False))) as listener:
#        os.environ['DISPLAY'] = os.environ['REMOTE_DISPLAY']
#        controller = keyboard.Controller()
#        while True:
#            controller.touch(*q.get())

import sys, termios, tty, os
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

while True:
    print(getch())


