import cv2
import threading
import time
import argparse
from queue import LifoQueue,Queue
import logging
from typing import List

class Sensor():
    def get(self):
        raise NotImplementedError("Subclasses must implement metod get()")

class SensorX(Sensor):
    def __init__(self, delay: float):
        self._delay = delay
        self._data = 0
    
    def get(self):
        time.sleep(self._delay)
        self._data += 1
        return self._data
    

class SensorCam():
    def __init__(self, sys_name: str, razr: str):
        r1 = razr.split("x")
        self.camInit = True
        self._razrX = int(r1[0])
        self._razrY = int(r1[1])
        print("Razr x:"+ str(self._razrX) +" y:" + str(self._razrY))
        self._camName = sys_name
        self._Camera = cv2.VideoCapture(sys_name, cv2.CAP_V4L2)
        if (self._Camera.isOpened() == False):
            logging.error("Cam not found")
            self.camInit = False
        self._Camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._razrX)
        self._Camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._razrY)
    def __del__(self):
        self._Camera.release()
    def get(self):
        global flag
        f, data = self._Camera.read()
        if(not(f)):
            logging.error("Frame not get")
            self.camInit = False
        return data


class ImageWindow():
    def __init__(self,fps:int = 15,height:int = 480):
        self._sensor_data = [0,0,0]
        self.frame = None
        self.fps = fps
        self._lock = threading.Lock()
        self._height = height
    def show(self,cam_queue:Queue,queues:List[Queue]):
        try:
            
            for i in range(3):
                if queues[i].full():
                    self._sensor_data[i] = queues[i].get()
            if cam_queue.full():
                self.frame=cam_queue.get()

            cv2.putText(self.frame,"Sensor1: "+str(self._sensor_data[0])+"  Sensor2: "+str(self._sensor_data[1])+"  Sensor3: "+str(self._sensor_data[2]), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0 ,0, 0) , 2)
            cv2.imshow('camera and data',self.frame)
        except Exception as e:
            logging.error("We have an error at show(): %s",str(e))

    def __del__(self):
        cv2.destroyAllWindows()



def sensor_worker(sensor:SensorX,queue:LifoQueue):
    global flag
    lock = threading.Lock()
    
    while flag:
        a = sensor.get()
        
        if queue.full():
            queue.get()
            queue.put(a)
        else:
            queue.put(a)
    logging.info("Sensor disable")

def camera_worker(queue:Queue, hw, camid):
    global flag
    global camflag
    global bigerror
    cam = SensorCam(camid,hw)
    if not (cam.camInit):
        bigerror = True
    queue.put(cam.get())
    logging.info("Cam init")
    camflag = True
    while flag and not(bigerror):
        a = cam.get()
        if not (cam.camInit):
            bigerror = True
        if queue.full():
            queue.get()
            queue.put(a)
        else:
            queue.put(a)
        
    cam.__del__()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camIndex",type=str,default= "/dev/video0")
    parser.add_argument("--hw", type=str, default = "1920x1080")
    parser.add_argument("--fps", type=int, default=60)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, filename="./log/log_file.log",filemode="w")

    r1 = args.hw.split("x")
    height1 = int(r1[1])
    flag = True
    camflag = False
    bigerror = False

    sensors = [SensorX(i) for i in [0.01,0.1,1]]
    logging.info("Sensor init")
    sensor_queues = [LifoQueue(maxsize=1) for _ in range(3)]
    logging.info("Sensor queue init")
    cam_queue = Queue(maxsize=1)
    sensor_workers = [threading.Thread(target=sensor_worker,args=(sensors[i],sensor_queues[i])) for i in range(3)]
    cam_worker = threading.Thread(target=camera_worker,args=(cam_queue,args.hw,args.camIndex))
    time.sleep(1)
    window_imager = ImageWindow(fps = args.fps,height=height1)
    for i in range(3):
        sensor_workers[i].start()
    cam_worker.start()
    while not(camflag):
        logging.info("Cam Wait")
        time.sleep(1)
    while True:
        window_imager.show(cam_queue,sensor_queues)
        if cv2.waitKey(1) & 0xFF == ord('q') or bigerror:
            window_imager.__del__()
            
            flag=False
            cam_worker.join()
            for sensor_workerr in sensor_workers:
                sensor_workerr.join()
            break

        
        time.sleep(1/window_imager.fps)