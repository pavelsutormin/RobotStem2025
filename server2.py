import flask
from flask import Flask
import moving_proc

# server_ip = "10.42.0.1"
# server_port = 80

server_ip = "192.168.86.184"
server_port = 8080

app = Flask(__name__)
'''engine = pyttsx3.init()
engine.setProperty('volume', 1)
engine.setProperty('rate', 150)'''

@app.route('/')
def index_server():
    return open("index.html", "r").read()

@app.route("/f")
def forward_server():
    print("forward")
    moving_proc.turn(0)
    moving_proc.move(1)
    return "forward"

@app.route("/s")
def stop_server():
    print("stop")
    moving_proc.turn(0)
    moving_proc.stop()
    return "stop"

@app.route("/b")
def backward_server():
    print("backward")
    moving_proc.turn(0)
    moving_proc.move(-1)
    return "backward"

@app.route("/l")
def left_server():
    print("left")
    moving_proc.turn(-1)
    moving_proc.move(1)
    return "left"

@app.route("/r")
def right_server():
    print("right")
    moving_proc.turn(1)
    moving_proc.move(1)
    return "right"

@app.route("/setspeed/<spd>")
def speed_server(spd):
    moving_proc.change_speed(int(spd))
    return "speed: " + spd

@app.route("/matrix.avif")
def matrix_bkg_server():
    return open("matrix.avif", "rb")


'''@app.route("/hello/<name>")
def hello_server(name):
    with open("cmd.txt", "w") as f:
        f.write("say")
    try:
        engine.say("Hello " + name)
        engine.runAndWait()
    except RuntimeError:
        print("I am in error")
        engine.endLoop()
        engine.say("Hello " + name)
        engine.runAndWait()
    print("Hello, " + name)
    requests.get("http://127.0.0.1:5000/hello/" + name)
    with open("cmd.txt", "w") as f:
        f.write("stop")
    return "Hello, " + name
'''

@app.route("/move")
def move_server():
    moving_proc.turn(float(flask.request.args.get("x")))
    moving_proc.move(float(flask.request.args.get("y")))
    return "move"


if __name__ == '__main__':
    moving_proc.start_moving_proc(True, 0.01, 0.02, 0)
    app.run(host=server_ip, port=server_port, debug=True)
