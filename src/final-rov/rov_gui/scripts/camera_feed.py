from concurrent.futures import ThreadPoolExecutor
from threading import Event

import cv2

# List of IPS
# IPS = ["rtsp://192.168.1.122:8554/test", "rtsp://192.168.1.123/test"]
# rtsp://192.168.1.122:8554/test", "rtsp://192.168.1.123:8554/test", 
# IPS = ["rtsp://10.42.0.221:8554/unicast","rtsp://192.168.111.226:8554/test"]
IPS = ["rtsp://192.168.1.103:8554/unicast", "rtsp://192.168.1.103:8555/unicast", "rtsp://192.168.1.104:8554/unicast"]


# Initiate Joystick
def displayCamera(IP, event, name):

    pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink"
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    while not event.is_set(): 
        
        _,img = cap.read()
        
        cv2.imshow(name, img)
        
        if(cv2.waitKey(1) == ord('q')):
            break
        
    cap.release()
    cv2.destroyAllWindows()

with ThreadPoolExecutor(max_workers=len(IPS)) as EXEC:
    
    events = [Event() for _ in range(len(IPS))]
    record_event = [Event() for _ in range(len(IPS))]
    names = [str(i) for i in range(3)]
    futures = [EXEC.submit(displayCamera, ip, event, name) for ip, event, name in zip(IPS, events, names)]
    input("Press Enter to stop cameras...")
    
    # Set the stop event for each camera
    for event in events:
        event.set()
    