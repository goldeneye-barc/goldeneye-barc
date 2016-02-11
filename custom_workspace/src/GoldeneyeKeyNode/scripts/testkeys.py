#!/usr/bin/python
import sys
sys.path.append('..')
import keyboard

def print_pressed_keys(e):
    line = ','.join(str(code) for code in keyboard._pressed_events)
    print('\r' + line)

keyboard.hook(print_pressed_keys)
keyboard.wait
