import RPi.GPIO as GPIO
from time import sleep


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
p1 = GPIO.PWM(en_s1,1000)
p1.start(100)
GPIO.setup(in_f2,GPIO.OUT)
GPIO.setup(in_b2,GPIO.OUT)
GPIO.setup(en_s2,GPIO.OUT)
GPIO.output(in_f2,GPIO.LOW)
GPIO.output(in_b2,GPIO.LOW)
p2 = GPIO.PWM(en_s2,1000)
p2.start(100)


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
    print(f"leftPart={leftPart}, rightPart={rightPart}")
    position = servoLeftPosition * leftPart + servoRightPosition * rightPart
    print(f"position={position}")
    dutyCyclePercentage = position * 100 / servoMsPerCycle
    print(f"dutyCyclePercentage={dutyCyclePercentage}")
    return dutyCyclePercentage


def turn(pos):
    servoPwm.ChangeDutyCycle(getDutyCyclePercentage(pos))
    sleep(0.2)


def forward(dutyCyclePercentage):
    p1.ChangeDutyCycle(dutyCyclePercentage)
    p2.ChangeDutyCycle(dutyCyclePercentage)
    GPIO.output(in_f1,GPIO.HIGH)
    GPIO.output(in_b1,GPIO.LOW)
    GPIO.output(in_f2,GPIO.HIGH)
    GPIO.output(in_b2,GPIO.LOW)

def stop():
    print("stop")
    GPIO.output(in_f1,GPIO.LOW)
    GPIO.output(in_b1,GPIO.LOW)
    GPIO.output(in_f2,GPIO.LOW)
    GPIO.output(in_b2,GPIO.LOW)

def backward(dutyCyclePercentage):
    p1.ChangeDutyCycle(dutyCyclePercentage)
    p2.ChangeDutyCycle(dutyCyclePercentage)
    GPIO.output(in_f1,GPIO.LOW)
    GPIO.output(in_b1,GPIO.HIGH)
    GPIO.output(in_f2,GPIO.LOW)
    GPIO.output(in_b2,GPIO.HIGH)

if __name__ == '__main__':

    servoPwm.start(getDutyCyclePercentage(0))
    for i in range(3):
        print("left")
        turn(-1)
        sleep(1)
        print("backward")
        backward(80)
        sleep(0.5)
        stop()
        sleep(1)

        print("mid")
        turn(0)
        sleep(1)
        print("forward")
        forward(80)
        sleep(0.5)
        stop()
        sleep(1)

        print("right")
        turn(1)
        sleep(1)
        print("backward")
        backward(80)
        sleep(0.5)
        stop()
        sleep(1)

        print("mid")
        turn(0)
        sleep(1)
        print("forward")
        forward(80)
        sleep(0.5)
        stop()
        sleep(1)

    servoPwm.stop()
    GPIO.cleanup()

