import threading

import cv2
import time
import moving_proc
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from gpiozero import CPUTemperature
import numpy as np
import flask
from flask import Flask


default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
default_labels = 'coco_labels.txt'
top_k = 3
threshold = 0.3 # 0.45

interpreter = make_interpreter(default_model)
interpreter.allocate_tensors()
labels = read_label_file(default_labels)
inference_size = input_size(interpreter)
print(inference_size)

cpu = CPUTemperature()


def getObjects(img, interpreter, labels, inference_size, threshold, top_k, needed_labels):
    cv2_im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
    run_inference(interpreter, cv2_im_rgb.tobytes())
    objs = get_objects(interpreter, threshold)[:top_k]
    height, width, channels = img.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]

    object_info_map = {}
    for needed_label in needed_labels:
        object_info = []
        for obj in objs:
            label = labels.get(obj.id, obj.id)
            if label != needed_label:
                continue
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, label)

            img = cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
            img = cv2.putText(img, label, (x0, y0+30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            object_info.append((bbox, label, percent))
        object_info_map.update({needed_label: object_info})
    return img, object_info_map


def center(channel, objInfo, width):
    # print("Channel: " + channel + "\n------------------")
    if len(objInfo) == 0:
        return None
    best_index = -1
    best_score = 0
    for i in range(len(objInfo)):
        score = objInfo[i][2]
        if score > best_score:
            best_score = score
            best_index = i
    bbox = objInfo[best_index][0]
    x0 = int(bbox.xmin)
    x1 = int(bbox.xmax)
    midx = (x0 + x1) / 2
    return midx / (width / 2) - 1


# Below determines the size of the live feed window that will be displayed on the Raspberry Pi OS
def loop():
    cap = cv2.VideoCapture(0)
    print('Is opened:', cap.isOpened())
    print('Width:', cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print('Height:', cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.set(3, 640)  # 2560)
    cap.set(4, 240)  # 960)

    # Below is the never ending loop that determines what will happen when an object is identified.
    last_process_time = time.time()
    is_moving = False
    while cap.isOpened():
        success, img = cap.read()
        next_process_time = time.time()
        if not success:
            return
        since_last_process = next_process_time - last_process_time
        if since_last_process < 0.1:
            continue
        last_process_time = next_process_time
        # print(type(img))
        # print(img.shape)
        img1 = img[0:240, 0:320]
        img2 = img[0:240, 320:640]

        needed_labels = ["clock", "stop sign"]
        # Below provides a huge amount of control. the 0.45 number is the threshold number, the 0.2 number is the nms number)
        img1, object_info_map1 = getObjects(img1, interpreter, labels, inference_size, threshold, top_k, needed_labels)
        img2, object_info_map2 = getObjects(img2, interpreter, labels, inference_size, threshold, top_k, needed_labels)
        c1 = None
        c2 = None
        object_type = None
        if len(object_info_map1["clock"]) > 0 and len(object_info_map2["clock"]) > 0:
            c1 = center("Left", object_info_map1["clock"], 320)
            c2 = center("Right", object_info_map2["clock"], 320)
            object_type = "clock"
        elif len(object_info_map1["stop sign"]) > 0 and len(object_info_map2["stop sign"]) > 0:
            c1 = center("Left", object_info_map1["stop sign"], 320)
            c2 = center("Right", object_info_map2["stop sign"], 320)
            object_type = "stop sign"

        if c1 is not None and c2 is not None:
            # We can move
            is_moving = True
            c = (c1 + c2) / 2
            print(f"Centers: {c1} + {c2} -> {c}")
            turn_dir = max(min(c * 2, 1), -1)
            move_dir = 1
            if object_type == "stop sign":
                turn_dir = -turn_dir
                move_dir = -move_dir
                print(f"Move backward!")
            else:
                print(f"Move forward!")
            moving_proc.turn(turn_dir)
            moving_proc.move(move_dir)
        else:
            # We should stop
            if is_moving:
                print("Stop!")
            moving_proc.stop()
            is_moving = False

        width_m = 800 - (img1.shape[1] + img2.shape[1])
        img_m = np.zeros((img1.shape[0], width_m, 3), dtype=np.uint8)
        img_top = np.concatenate((img1, img_m, img2), axis=1)

        print(cpu.temperature)

        temp = round(cpu.temperature, 1)

        footer = np.zeros((150, img_top.shape[1], 3), dtype=np.uint8)
        cv2.putText(footer, "Temp: " + str(temp), (5, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), 2)

        img_full = np.concatenate((img_top, footer), axis=0)
        cv2.imshow("Output", img_full)

        cv2.waitKey(1)

def run_loop():
    while True:
        loop()
        print("Something happened. Process is restarting...")

server_ip = "192.168.86.184"
server_port = 8080
app = Flask(__name__)

@app.route('/')
def index_server():
    return open("index_ai.html", "r").read()

@app.route("/stop")
def stop_server():
    moving_proc.stop_moving_proc()
    return "stop"

@app.route("/restart")
def restart_server():
    moving_proc.restart_moving_proc()
    return "restart"

def serve():
    app.run(host=server_ip, port=server_port, debug=True)

if __name__ == "__main__":
    moving_proc.start_moving_proc(True, 0.01, 0.02, 0)
    # threading.Thread(target=run_loop).start()
    # serve()
    run_loop()
