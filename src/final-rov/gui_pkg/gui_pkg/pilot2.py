import cv2

def displayCamera(IP, name, size, position, screen):
    pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
    
    x, y, w, h = screen
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, int(w*size[0]), int(h*size[1]))
    cv2.moveWindow(name, int(x+w*position[0]), int(y+h*position[1]))
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    while 1: 
        
        _,img = cap.read()
        
        cv2.imshow(name, img)
        
        if(cv2.waitKey(1) == ord('q')):
            break
        
    cap.release()
    cv2.destroyAllWindows()

