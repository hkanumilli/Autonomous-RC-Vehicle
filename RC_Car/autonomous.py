import cv2
import sys
import threading
import socketserver
import numpy as np
import picamera
import time, io

from picamera.array import PiRGBArray


from model import NeuralNetwork
from rc_driver_helper import *

from ultrasonic import measure_distance

class Autonomy(): 

    h1 = 5.5  # cm
    h2 = 5.5

    # load trained neural network
    nn = NeuralNetwork()
    nn.load_model("saved_model/nn_model.xml")

    obj_detection = ObjectDetection()
    rc_car = RCControl() 

    # cascade classifiers
    stop_cascade = cv2.CascadeClassifier("cascade_xml/stop_sign.xml")
    light_cascade = cv2.CascadeClassifier("cascade_xml/traffic_light.xml")

    d_to_camera = DistanceToCamera()
    # hard coded thresholds for stopping, sensor 30cm, other two 25cm
    d_sensor_thresh = 12
    d_stop_light_thresh = 25
    d_stop_sign = d_stop_light_thresh
    d_light = d_stop_light_thresh

    stop_start = 0  # start time when stop at the stop sign
    stop_finish = 0
    stop_time = 0
    drive_time_after_stop = 0

    def drive(self):
        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (320, 240)      # pi camera resolution
                camera.framerate = 15               # 15 frames/sec
                time.sleep(2)                       # give 2 secs for camera to initilize
                start = time.time()
                stream = io.BytesIO()

                sensor_data = None
                stop_flag = False
                stop_sign_active = True
                
                # send jpeg format video stream
                for foo in camera.capture_continuous(stream, 'jpeg', use_video_port = True):
                    
                    if time.time() - start > 600:
                        break
                    
                    stream.seek(0)
                    data = stream.read()

                    gray = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)

                    # lower half of the image
                    height, width = gray.shape
                    roi = gray[int(height/2):height, :]

                    # # object detection
                    v_param1 = self.obj_detection.detect(self.stop_cascade, gray, image)
                    v_param2 = self.obj_detection.detect(self.light_cascade, gray, image)

                    # distance measurement
                    if v_param1 > 0 or v_param2 > 0:
                        d1 = self.d_to_camera.calculate(v_param1, self.h1, 300, image)
                        d2 = self.d_to_camera.calculate(v_param2, self.h2, 100, image)
                        self.d_stop_sign = d1
                        self.d_light = d2

                    cv2.imshow('image', image)
                    cv2.imshow('mlp_image', roi)
                    cv2.waitKey(1)

                    # reshape image
                    image_array = roi.reshape(1, int(height/2) * width).astype(np.float64)
                    prediction = self.nn.predict(image_array)

                    sensor_data = measure_distance()

                    # stop conditions
                    if sensor_data < self.d_sensor_thresh:
                        print("Stop, obstacle in front")
                        self.rc_car.stop()
                        sensor_data = None

                    elif 0 < self.d_stop_sign < self.d_stop_light_thresh and stop_sign_active:
                        print("Stop sign ahead")
                        self.rc_car.stop()

                        # stop for 5 seconds
                        if stop_flag is False:
                            self.stop_start = cv2.getTickCount()
                            stop_flag = True
                        self.stop_finish = cv2.getTickCount()

                        self.stop_time = (self.stop_finish - self.stop_start) / cv2.getTickFrequency()
                        print("Stop time: %.2fs" % self.stop_time)

                        # 5 seconds later, continue driving
                        if self.stop_time > 5:
                            print("Waited for 5 seconds")
                            stop_flag = False
                            stop_sign_active = False

                    elif 0 < self.d_light < self.d_stop_light_thresh:
                        # print("Traffic light ahead")
                        if self.obj_detection.red_light:
                            print("Red light")
                            self.rc_car.stop()
                        elif self.obj_detection.green_light:
                            print("Green light")
                            pass
                        elif self.obj_detection.yellow_light:
                            print("Yellow light flashing")
                            pass

                        self.d_light = self.d_stop_light_thresh
                        self.obj_detection.red_light = False
                        self.obj_detection.green_light = False
                        self.obj_detection.yellow_light = False

                    else:
                        self.rc_car.steer(prediction)
                        self.stop_start = cv2.getTickCount()
                        self.d_stop_sign = self.d_stop_light_thresh

                        if stop_sign_active is False:
                            self.drive_time_after_stop = (self.stop_start - self.stop_finish) / cv2.getTickFrequency()
                            if self.drive_time_after_stop > 5:
                                stop_sign_active = True

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("car stopped")
                        self.rc_car.stop()
                        break

                    stream.seek(0)
                    stream.truncate()
        except Exception as e: 
            print('Failed: {}'.format(e))

        finally: 
            cv2.destroyAllWindows()
            sys.exit()



if __name__ == '__main__': 
    car = Autonomy()
    car.drive()
