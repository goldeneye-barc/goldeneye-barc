
#!/usr/bin/env python
from pynput import keyboard
import rospy
from barc.msg import ECU



combination = {'a', 'w', 's', 'd'}
current = set()
pub = rospy.Publisher('ecu_pwm', ECU, queue_size=10)
rospy.init_node('key_node')
rate = rospy.Rate(10)

def on_press(key):
    try:
        if key.char in combination:
            current.add(key.char)
        msg = move(current)
        pub.publish(msg)
        #print(msg.motor, " , servo: ", msg.servo)
        rate.sleep()

        if key == keyboard.Key.esc:
            listener.stop()
        #if key.char == 'a':
        #    print('left')
        #if key.char == 'w':
        #  print('front')
        #if key.char == 's':
        #  print('back')
        #if key.char == 'd':
        #    print('right')
    except Exception as e:
        #print(e)
        pass
    except AttributeError:
        pass

def on_release(key):
    try:
        current.remove(key.char)
        msg = move(current)
        pub.publish(msg)
        rate.sleep()
        #print(msg.motor, " , servo: ", msg.servo)
    except KeyError:
        pass
    except AttributeError:
        pass
        
def move(current):
    msg = ECU()
    msg.motor = 1500
    msg.servo = 1500
    if 'a' in current:
        msg.servo -= 300
    elif 'd' in current:
        msg.servo += 300
    
    if 'w' in current:
        msg.motor += 100
    elif 's' in current:
        msg.motor -= 200

    return msg
        


def key_node():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)

    while not rospy.is_shutdown():
        try:
            if not listener.running:
                listener.start()
        except Exception as e:
            listener.stop()
            #print(e)

#with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
#    listener.join()

if __name__=="__main__":
    try:
        key_node()
    except rospy.ROSInterruptException:
        pass

