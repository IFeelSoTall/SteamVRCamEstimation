import sys, os
#sys.path.insert(1, os.getcwd()+'/PYTHONPATH/opencv/build/python')
PyPATH = os.getcwd()
f = open("log.txt", "w")
f.write("check1");
f.close()
sys.path.insert(1, PyPATH+'/opencv/build/python')
f = open("log.txt", "a")
f.write("check2");
f.close()
print(os.getcwd())
f = open("log.txt", "a")
f.write("check3");
f.close()
#sys.path.insert(1, os.getcwd()+'/PYTHONPATH')
sys.path.insert(1, PyPATH)
f = open("log.txt", "a")
f.write("check4");
f.close()


import time
import socket

import json
import struct
import cv2
import numpy as np

from modules.parse_poses import parse_poses
from modules.inference_engine_pytorch import InferenceEnginePyTorch
f = open("log.txt", "a")
f.write("check5");
f.close()

def pose3d():
    stride = 8
    #f.write("check8");
    net = InferenceEnginePyTorch( PyPATH + '\model\human-pose-estimation-3d.pth', 'GPU')

    with open( PyPATH + '\parameters\extrinsics.json', 'r') as f:
        extrinsics = json.load(f)
    R = np.array(extrinsics['R'], dtype=np.float32)
    t = np.array(extrinsics['t'], dtype=np.float32)

    cap = cv2.VideoCapture(1)

    if not(cap.isOpened):
        print("Webcam not recognized")

    base_height = 256

    while True:
        ret, frame = cap.read()
        if (ret == False):
            continue;
        #time.sleep(1)
        #print(ret)
        input_scale = base_height / frame.shape[0]
        scaled_img = cv2.resize(frame, dsize=None, fx=input_scale, fy=input_scale)
        #scaled_img = scaled_img[:, 0:scaled_img.shape[1] - (scaled_img.shape[1] % stride)]  # better to pad, but cut out for demo
        fx = np.float32(0.8 * frame.shape[1])

        inference_result = net.infer(scaled_img)
        poses_3d, poses_2d = parse_poses(inference_result, input_scale, stride, fx)
        if len(poses_2d):
            poses_3d_copy = poses_2d.copy()
            x = poses_3d_copy[:, 0::3]
            y = poses_3d_copy[:, 1::3]
            #0 - GRUD
            #1 - nose
            #2 - nothing
            #3 - levoe plecho
            #4 - levi lokot
            #5 - levi zapastie
            #6 - levoe taz
            #7 - levi koleno
            #8 - levi stopa
            #9 - pravoe plecho
            #10 - pravoe lokot
            #11 - pravoe zapastie
            #12 - pravoe taz
            #13 - pravoe koleno
            #14 - pravoe stopa
            frame = cv2.circle(frame, (x[0][7], y[0][7]), 10,(255, 0, 0))
            print("x")
            print(x[0][7])
            print("y")
            print(y[0][7])
            #frame
            #size = len(x[0])
            #i = 0
            #for a in x[0]:
                #frame = cv2.circle(frame, (x[0][i], y[0][i]), 10,(255, 0, 0))
                #i=i+1
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            #print(poses_3d_copy)
            #print("x")
            #print(x)
            #print("y")
            #print(y)
        if len(poses_3d):
            poses_3d_copy = poses_3d.copy()
            #poses_3d_1 = poses_3d
            #poses_3d_2 = poses_3d
            x = poses_3d_copy[:, 0::4]
            y = poses_3d_copy[:, 1::4]
            z = poses_3d_copy[:, 2::4]
            #print(x)
            #print("x")
            #print(y)
            #print("y")
            #print(z)
            #print("z")
            #cv2.imshow("Live", frame);
            #poses_3d[:, 0::4], poses_3d[:, 1::4], poses_3d[:, 2::4] = z, -x, -y
            #poses_3d = poses_3d.reshape(poses_3d.shape[0], 19, -1)[:, :, 0:3]
        
            
            #poses_3d_copy[:, 0::4], poses_3d_copy[:, 1::4], poses_3d_copy[:, 2::4] = z, -x, -y
            #poses_3d_copy = poses_3d_copy.reshape(poses_3d_copy.shape[0], 19, -1)[:, :, 0:3]
            #ba = poses_3d_copy[0,0]
            #ba = np.hstack((ba,poses_3d_copy[0,8]))
            #ba = np.hstack((ba,poses_3d_copy[0,14]))
            #print(poses_3d)
            #try:
            #s.send(ba)
            #except:
                #print("fuck")
                #s.close()
                #cap.release()
                #cv2.destroyAllWindows()
                #return
            


if __name__ == '__main__':
    f = open("log.txt", "a")
    f.write("check6");
    f.close()
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.connect(('localhost', 50000))
    f = open("log.txt", "a")
    f.write("check7");
    f.close()
    pose3d()

    s.close()
