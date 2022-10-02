from turtle import width
import cv2
import numpy as np
from elements.yolo import OBJ_DETECTION
import time
import socket


def operate(state, label, order, target):

    if order == "grab" and state == "relax" and label == target:
        print (f"{order} {target}") # replace by grab()
        # time.sleep(10)
        state = "grab"
    elif order == "relax" and state == "grab":
        print ("relaxing") # replace by relax()
        # time.sleep(10)
        state = "relax"
    return state

def object_detection(order, target):

    Object_classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
                    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
                    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
                    'hair drier', 'toothbrush' ]


    # parameters
    capture_width=1280
    capture_height=720

    center_thre = [100, 100]
    box_thre = [200, 200]
    state = "relax" # default relax, can switch to hold


    Object_colors = list(np.random.rand(80,3)*255)
    Object_detector = OBJ_DETECTION('weights/yolov5s.pt', Object_classes)

    def gstreamer_pipeline(
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=24,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )


    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=2))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

    if cap.isOpened():
        print("Opened")
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            # print("window set")
            ret, frame = cap.read()
            if ret:
                # detection process
                #print("got frame")
                objs = Object_detector.detect(frame)

                for obj in objs:

                    # acquire object
                    label = obj['label']
                    score = obj['score']
                    [(xmin,ymin),(xmax,ymax)] = obj['bbox']

                    if label == target or order == "relax":

                        # functionality for grabing
                        object_center = [float((xmin+xmax)/2), float((ymin+ymax)/2)]
                        screen_center = [float(capture_width/2), float(capture_height/2)]
                        box_size = [float(xmax - xmin), float(ymax - ymin)]
                        if (np.abs(object_center[0] - screen_center[0]) < center_thre[0] and np.abs(object_center[1]- screen_center[1]) < center_thre[1]):
                            if (box_size[0] > box_thre[0] and box_size[1] > box_thre[1]):
                                print(f"grab {label}")
                                # print(f"{label}: {object_center}; {box_size}")
                                # state = operate(state, label, order, target)


                    # plotting
                    color = Object_colors[Object_classes.index(label)]
                    frame = cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2) 
                    frame = cv2.putText(frame, f'{label} ({str(score)})', (xmin,ymin), cv2.FONT_HERSHEY_SIMPLEX , 0.75, color, 1, cv2.LINE_AA)

            cv2.imshow("CSI Camera", frame)
            keyCode = cv2.waitKey(30)
            if keyCode == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")




if __name__ == "__main__":
    order = "grab"
    target = "apple"
    object_detection(order, target)
