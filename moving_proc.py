import RPi.GPIO as GPIO
from time import sleep
import threading

turn_pos_current = 0
turn_pos_new = 0
speed = 100
move_speed = 0

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
    print("forward: " + str(dutyCyclePercentage))
    p1.ChangeDutyCycle(dutyCyclePercentage)
    p2.ChangeDutyCycle(dutyCyclePercentage)
    GPIO.output(in_f1,GPIO.HIGH)
    GPIO.output(in_b1,GPIO.LOW)
    GPIO.output(in_f2,GPIO.HIGH)
    GPIO.output(in_b2,GPIO.LOW)

def __stop__():
    print("stop")
    GPIO.output(in_f1,GPIO.LOW)
    GPIO.output(in_b1,GPIO.LOW)
    GPIO.output(in_f2,GPIO.LOW)
    GPIO.output(in_b2,GPIO.LOW)

def __backward__(dutyCyclePercentage):
    print("backward: " + str(dutyCyclePercentage))
    p1.ChangeDutyCycle(dutyCyclePercentage)
    p2.ChangeDutyCycle(dutyCyclePercentage)
    GPIO.output(in_f1,GPIO.LOW)
    GPIO.output(in_b1,GPIO.HIGH)
    GPIO.output(in_f2,GPIO.LOW)
    GPIO.output(in_b2,GPIO.HIGH)

def turn_thread():
    global turn_pos_current, turn_pos_new
    while True:
        if turn_pos_current is not turn_pos_new:
            __turn__(turn_pos_new)
            turn_pos_current = turn_pos_new
            sleep(0.2)
        else:
            sleep(0.05)

def move_thread(do_pause, stop_time, move_time):
    global turn_pos_current, turn_pos_new
    while True:
        if move_speed > 0:
            __forward__(move_speed * speed)
        elif move_speed < 0:
            __backward__(abs(move_speed * speed))
        else:
            __stop__()
        if do_pause:
            sleep(move_time)
            __stop__()
            sleep(stop_time)
        else:
            sleep(0.05)

def turn(p):
    global turn_pos_new
    turn_pos_new = p

def move(s):
    global move_speed
    move_speed = s

def stop():
    __stop__()

def change_speed(spd):
    global speed
    speed = spd

def start_moving_proc(do_pause, stop_time, move_time):
    servoPwm.start(getDutyCyclePercentage(0))
    t1 = threading.Thread(target=turn_thread)
    t1.start()
    t2 = threading.Thread(target=move_thread, args=(do_pause, stop_time, move_time))
    t2.start()
