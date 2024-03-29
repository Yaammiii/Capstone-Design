import numpy as np
import cv2
from ultralytics import YOLO, utils
import random
import os



def object_detect(model, frame, detection_colors, class_list):

    # Predict on image
    detect_params = model.predict(source=[frame], conf=0.70, save=False, verbose=False)

    # Convert tensor array to numpy
    DP = detect_params[0].cpu().numpy()

    # temporary dict to save detected objects in current loop
    data_dict = {}

    if len(detect_params[0]) != 0:
        for i in range(len(detect_params[0])):
            boxes = detect_params[0].boxes
            box = boxes[i]  # returns one box
            clsID = box.cls.cpu().numpy()[0]
            conf = box.conf.cpu().numpy()[0]
            bb = box.xyxy.cpu().numpy()[0]

            current_class = class_list[int(clsID)]
            # print(current_class)

            cv2.rectangle(
                frame,
                (int(bb[0]), int(bb[1])),
                (int(bb[2]), int(bb[3])),
                detection_colors[int(clsID)],
                3,
                )
            data_dict[current_class] = is_triggering_dist(
                                                    current_class,
                                                    bb[3]-bb[1])
            # if Trafficlight is green, it doesn't need to be triggered
            # decision of light is depends on the brightness of upper/lower half side
            if current_class.lower() == 'trafficlight':
                if data_dict[current_class] == True:
                    croppped = frame[int(bb[1]):int(bb[3]), int(bb[0]):int(bb[2])]
                    crop_upper = frame[int(bb[1]):int((bb[1]+bb[3])/2), int(bb[0]):int(bb[2])]
                    crop_lower = frame[int((bb[1]+bb[3])/2):int(bb[3]), int(bb[0]):int(bb[2])]
                    hsvFrame_u = cv2.cvtColor(crop_upper, cv2.COLOR_BGR2GRAY)
                    hsvFrame_l = cv2.cvtColor(crop_lower, cv2.COLOR_BGR2GRAY)

                    bright_px_u = np.sum(hsvFrame_u >= 240)
                    bright_px_l = np.sum(hsvFrame_l >= 240)

                    if bright_px_u>bright_px_l:
                        data_dict[current_class] = True
                        print('red')
                    else:
                        data_dict[current_class] = False
                        print('green')

            # Display class name and confidence
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                frame,
                current_class
                + " "
                + str(round(conf*100, 2))
                + "%",
                (int(bb[0]), int(bb[1]) - 10),
                font,
                0.7,
                (255, 255, 255),
                2,
            )

    return data_dict, frame
        
def is_triggering_dist(object, height):
    dist = 1/(height) * 10000
    if object.lower() == 'trafficlight' and dist <380:
        # print(dist)
        print(f'Trafficlight in Range: {dist}')
        return True
    
    elif object.lower() == 'crosswalk' and dist <300:
        # print(dist)
        print(f'Crosswalk in Range: {dist}')
        return True

    elif object.lower() == 'stop' and dist <260:
        # print(dist)
        print(f'Stop in Range: {dist}')
        return True

    elif object.lower() == 'speedlimit' and dist < 240:
        # print(dist)
        print('Speedlimit in Range')
        return True
    
    elif object == 'person' and dist <200:
        return True
    
    else:
        return False
        
    

def main():
    model = YOLO(r"/home/hanwool/Desktop/haksulje/run/Final/weights/0.990.pt")
    class_list = list(model.names.values())

    # Generate random colors for class list
    detection_colors = []
    for i in range(len(class_list)):
        r = random.randint(0,255)
        g = random.randint(0,255)
        b = random.randint(0,255)
        detection_colors.append((b,g,r))
    url = 'http://192.168.0.202:81/stream'
    capture3 = cv2.VideoCapture(url)
    # capture3.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # capture3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        ret, frame = capture3.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        # print(frame.shape)


        # Predict on image
        detect_params = model.predict(source=[frame], conf=0.70, save=False, verbose=False)

        # Convert tensor array to numpy
        DP = detect_params[0].cpu().numpy()

        # temporary dict to save detected objects in current loop
        data_dict = {}

        if len(detect_params[0]) != 0:
            for i in range(len(detect_params[0])):
                boxes = detect_params[0].boxes
                box = boxes[i]  # returns one box
                clsID = box.cls.cpu().numpy()[0]
                conf = box.conf.cpu().numpy()[0]
                bb = box.xyxy.cpu().numpy()[0]

                current_class = class_list[int(clsID)]
                # print(current_class)

                cv2.rectangle(
                    frame,
                    (int(bb[0]), int(bb[1])),
                    (int(bb[2]), int(bb[3])),
                    detection_colors[int(clsID)],
                    3,
                    )
                data_dict[current_class] = is_triggering_dist(
                                                        current_class,
                                                        bb[3]-bb[1])
                # if Trafficlight is green, it doesn't need to be triggered
                # decision of light is depends on the brightness of upper/lower half side
                if current_class.lower() == 'trafficlight':
                    if data_dict[current_class] == True:
                        croppped = frame[int(bb[1]):int(bb[3]), int(bb[0]):int(bb[2])]
                        crop_upper = frame[int(bb[1]):int((bb[1]+bb[3])/2), int(bb[0]):int(bb[2])]
                        crop_lower = frame[int((bb[1]+bb[3])/2):int(bb[3]), int(bb[0]):int(bb[2])]
                        hsvFrame_u = cv2.cvtColor(crop_upper, cv2.COLOR_BGR2GRAY)
                        hsvFrame_l = cv2.cvtColor(crop_lower, cv2.COLOR_BGR2GRAY)

                        bright_px_u = np.sum(hsvFrame_u >= 240)
                        bright_px_l = np.sum(hsvFrame_l >= 240)

                        if bright_px_u>bright_px_l:
                            data_dict[current_class] = True
                            print('Red')
                        else:
                            data_dict[current_class] = False
                            print('Green')

                # Display class name and confidence
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(
                    frame,
                    current_class
                    + " "
                    + str(round(conf*100, 2))
                    + "%",
                    (int(bb[0]), int(bb[1]) - 10),
                    font,
                    0.7,
                    (255, 255, 255),
                    2,
                )
        cv2.imshow('ObjectDetection', frame)
        # print('proessed:',frame.shape)
        cv2.waitKey(1)



if __name__ == "__main__":
    main()
