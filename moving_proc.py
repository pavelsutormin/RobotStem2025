import RPi.GPIO as GPIO
from time import sleep
import threading

turn_pos_current = 0
turn_pos_new = 0
speed = 100
move_speed = 0
is_moving_proc_active = True

in_f1 = 24
in_b1 = 23
en_s1 = 25

in_f2 = 5
in_b2 = 6
en_s2 = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(in_f1,GPIO.OUT)
GPIO.setup(in_b1,GPIO.OUT)
GPIO.setup(en_s1,GPIO.OUT)
GPIO.output(in_f1,GPIO.LOW)
GPIO.output(in_b1,GPIO.LOW)
p1=GPIO.PWM(en_s1,1000)
p1.start(speed)

GPIO.setup(in_f2,GPIO.OUT)
GPIO.setup(in_b2,GPIO.OUT)
GPIO.setup(en_s2,GPIO.OUT)
GPIO.output(in_f2,GPIO.LOW)
GPIO.output(in_b2,GPIO.LOW)
p2=GPIO.PWM(en_s2,1000)
p2.start(speed)

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
servoFrequencyHertz = 50
servoPwm = GPIO.PWM(12, servoFrequencyHertz)
servoLeftPosition = 1.0
servoRightPosition = 2.0
servoMsPerCycle = 1000 / servoFrequencyHertz

def getDutyCyclePercentage(pos):
    rightPart = 0.5 + pos / 2.0
    leftPart = 1 - rightPart
    # print(f"leftPart={leftPart}, rightPart={rightPart}")
    position = servoLeftPosition * leftPart + servoRightPosition * rightPart
    # print(f"position={position}")
    dutyCyclePercentage = position * 100 / servoMsPerCycle
    # print(f"dutyCyclePercentage={dutyCyclePercentage}")
    return dutyCyclePercentage


def __turn__(pos):
    print("turn: " + str(pos))
    servoPwm.ChangeDutyCycle(getDutyCyclePercentage(pos))


def __forward__(dutyCyclePercentage):
    p1.ChangeDutyCycle(dutyCyclePercentage)
    p2.ChangeDutyCycle(dutyCyclePercentage)
    GPIO.output(in_f1,GPIO.HIGH)
    GPIO.output(in_b1,GPIO.LOW)
    GPIO.output(in_f2,GPIO.HIGH)
    GPIO.output(in_b2,GPIO.LOW)

def __stop__():
    GPIO.output(in_f1,GPIO.LOW)
    GPIO.output(in_b1,GPIO.LOW)
    GPIO.output(in_f2,GPIO.LOW)
    GPIO.output(in_b2,GPIO.LOW)

def __backward__(dutyCyclePercentage):
    p1.ChangeDutyCycle(dutyCyclePercentage)
    p2.ChangeDutyCycle(dutyCyclePercentage)
    GPIO.output(in_f1,GPIO.LOW)
    GPIO.output(in_b1,GPIO.HIGH)
    GPIO.output(in_f2,GPIO.LOW)
    GPIO.output(in_b2,GPIO.HIGH)

def __move__(m):
    global speed
    if m > 0:
        __forward__(m * speed)
    elif m < 0:
        __backward__(abs(m * speed))
    else:
        __stop__()

def turn_thread():
    global turn_pos_current, turn_pos_new, is_moving_proc_active
    while True:
        if turn_pos_current is not turn_pos_new and is_moving_proc_active:
            turn_pos_current = turn_pos_new
            __turn__(turn_pos_new)
            sleep(0.2)
        else:
            sleep(0.05)

def move_thread(do_pause, move_time, idle_time, idle_speed):
    global move_speed, is_moving_proc_active
    while True:
        if is_moving_proc_active:
            __move__(move_speed)
            if do_pause:
                sleep(move_time)
                __move__(idle_speed * move_speed)
                sleep(idle_time)
            else:
                sleep(0.05)
        else:
            sleep(0.05)


def turn(p):
    global turn_pos_new
    turn_pos_new = p

def move(s):
    global move_speed, speed
    if s > 0:
        print("forward: " + str(s * speed))
    elif s < 0:
        print("backward: " + str(s * speed))
    else:
        print("stop")
    move_speed = s

def stop():
    global move_speed
    print("stop")
    move_speed = 0

def change_speed(spd):
    global speed
    speed = spd

def start_moving_proc(do_pause, move_time=0.05, idle_time=0, idle_speed=0):
    servoPwm.start(getDutyCyclePercentage(0))
    t1 = threading.Thread(target=turn_thread)
    t1.start()
    t2 = threading.Thread(target=move_thread, args=(do_pause, move_time, idle_time, idle_speed))
    t2.start()

def stop_moving_proc():
    global is_moving_proc_active
    is_moving_proc_active = False
    servoPwm.stop()
    stop()
    __stop__()

def restart_moving_proc():
    global is_moving_proc_active
    servoPwm.start(getDutyCyclePercentage(turn_pos_current))
    is_moving_proc_active = True

