import cv2
IP = "rtsp://192.168.1.100:8554/unicast"
pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink"
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while 1: 
    
    _,img = cap.read()        
    cv2.imshow("Points", img)
        
    if(cv2.waitKey(1) == ord("q")):
        break
    
cap.release
cv2.destroyAllWindows()

