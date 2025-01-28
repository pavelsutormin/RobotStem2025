import cv2
import time
import moving_proc
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
import numpy as np

# cpu = CPUTemperature()

# This is to pull the information about what each object is called
#classNames = []
#classFile = "coco.names"
#default_model_dir = '../all_models'
default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
default_labels = 'coco_labels.txt'
top_k = 3
threshold = 0.3 # 0.45

#with open(classFile, "rt") as f:
#    classNames = f.read().rstrip("\n").split("\n")

# This is to pull the information about what each object should look like
#configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
#weightsPath = "frozen_inference_graph.pb"

interpreter = make_interpreter(default_model)
interpreter.allocate_tensors()
labels = read_label_file(default_labels)
inference_size = input_size(interpreter)
print(inference_size)

# This is some set up values to get good results
#net = cv2.dnn_DetectionModel(weightsPath, configPath)
#net.setInputSize(320, 320)
#net.setInputScale(1.0 / 127.5)
#net.setInputMean((127.5, 127.5, 127.5))
#net.setInputSwapRB(True)


def getObjects(img, interpreter, labels, inference_size, threshold, top_k):
    cv2_im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
    run_inference(interpreter, cv2_im_rgb.tobytes())
    objs = get_objects(interpreter, threshold)[:top_k]
    #return append_objs_to_img(img, inference_size, objs, labels)
    height, width, channels = img.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]

    objectInfo = []
    for obj in objs:
        label = labels.get(obj.id, obj.id)
        if label != "person":  # "person":
            continue
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, label)

        img = cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        img = cv2.putText(img, label, (x0, y0+30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        objectInfo.append((bbox, label, percent))
    return img, objectInfo


def center(channel, objInfo, width):
    # print("Channel: " + channel + "\n------------------")
    # for obj in objInfo:
    #     print(f" - Bounding Box: {obj[0]}, Class Name: {obj[1]}, Confidence: {obj[2]}")
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
        # t = 50 #cpu.temperature
        # print(f"Temperature: {t}")
        # if t > 70:
        #     break

        success, img = cap.read()
        next_process_time = time.time()
        if not success:
            return
        since_last_process = next_process_time - last_process_time
        if since_last_process < 0.3:
            if since_last_process >= 0.2 and is_moving:
                moving_proc.stop()
                is_moving = False
            continue
        last_process_time = next_process_time
        # print(type(img))
        # print(img.shape)
        img1 = img[0:240, 0:320]
        img2 = img[0:240, 320:640]

        # Below provides a huge amount of controll. the 0.45 number is the threshold number, the 0.2 number is the nms number)
        img1, objectInfo1 = getObjects(img1, interpreter, labels, inference_size, threshold, top_k)
        img2, objectInfo2 = getObjects(img2, interpreter, labels, inference_size, threshold, top_k)
        c1 = center("Left", objectInfo1, 320)
        c2 = center("Right", objectInfo2, 320)

        if c1 is not None and c2 is not None:
            # We can move
            is_moving = True
            # rotation = get_rotation_force((c1 + c2) / 2) * -100
            '''
            c = 0
            if c1 > 0 and c2 > 0:
                c = max(c1, c2)
            elif c1 < 0 and c2 < 0:
                c = min(c1, c2)
            else:
            '''
            c = (c1 + c2) / 2
            print(f"Centers: {c1} + {c2} -> {c}")

            #rotation = get_rotation_force(c) * -130
            #if c > 0.5:
            #    print(f"Rotate right! Motors: 100%, -100%")
            #    move(100, -100)
            if c > 0.4:
                print(f"Move right!")
                #move(100, rotation)
                moving_proc.turn(1)
            #elif c < -0.5:
            #    print(f"Move left! Motors: -100%, 100%")
            #    move(-100, 100)
            elif c < -0.4:
                print(f"Move left!")
                #move(rotation, 100)
                moving_proc.turn(-1)
            else:
                print("Move forward!")
                # move(100, 100)
                moving_proc.turn(0)
            moving_proc.move(1)
        else:
            # We should stop
            if is_moving:
                print("Stop!")
            moving_proc.stop()
            is_moving = False

        #print(f" - c1: {c1}, c2: {c2}")
        cv2.imshow("Output 1", np.concatenate((img1, img2), axis=1))
        #cv2.imshow("Output 2", img2)
        cv2.waitKey(1)
        # sleep(0.25)
        # moving_proc.stop()
        # sleep(0.25)


if __name__ == "__main__":
    moving_proc.start_moving_proc(False, 0, 0)
    while True:
        loop()
        print("Something happened. Process is restarting...")
