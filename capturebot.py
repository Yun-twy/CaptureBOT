import serial
import time
import cv2
import threading
from flask import Response, Flask

#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit

import signal

import datetime

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(i2c_bus=1, addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)


atexit.register(turnOffMotors)

################################# DC motor test!
myMotor = mh.getMotor(1)
myMotor2 = mh.getMotor(2)
myMotor3 = mh.getMotor(3)
myMotor4 = mh.getMotor(4)

# set the speed to start, from 0 (off) to 255 (max speed)
myMotor.setSpeed(550) #150
myMotor2.setSpeed(550)
myMotor3.setSpeed(550)
myMotor4.setSpeed(550)

myMotor.run(Adafruit_MotorHAT.RELEASE)
myMotor2.run(Adafruit_MotorHAT.RELEASE)
myMotor3.run(Adafruit_MotorHAT.RELEASE)
myMotor4.run(Adafruit_MotorHAT.RELEASE)

global is_running
is_running = True

global length
length = 250
# Global variables to share data between threads
face_x, face_y = 480, 270
global video_frame
video_frame = None
global thread_lock
thread_lock = threading.Lock()

# GStreamer Pipeline to access the Raspberry Pi camera
GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

# Create the Flask object for the application
app = Flask(__name__)


# Your existing gstreamer_pipeline function
def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=True"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def send_serial_data():
    global face_x, face_y
    ser = serial.Serial('/dev/ttyACM0', 9600)

    while is_running:
        # Access shared variables with a lock to prevent data races
        ser.write('{0},{1}\n'.format(face_x, face_y).encode())
        time.sleep(1)


def face_detect(frame):
    global face_x, face_y, length
    face_cascade = cv2.CascadeClassifier(
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
    )
    smile_cascade = cv2.CascadeClassifier(
        "/usr/share/opencv4/haarcascades/haarcascade_smile.xml"
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    if len(faces) > 0:
        x, y, w, h = faces[0]
        length = w
        face_x_temp = x + w/2
        face_y_temp = y + h/2
        if 0 < face_x_temp < 960 and 0 < face_y_temp < 540:
            #print("center X : ", face_x_temp, " center Y : ", face_y_temp, "lenght : ", length)
            face_x, face_y = face_x_temp, face_y_temp
        else:
            face_x, face_y = 480, 270

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Capture the face region
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]

        # Classify the facial expression
        smiles = smile_cascade.detectMultiScale(roi_gray, 1.8, 20)
        for (sx, sy, sw, sh) in smiles:
            #cv2.rectangle(roi_color, (sx, sy), (sx+sw, sy+sh), (0, 255, 0), 2)

            # Check if the expression is "smile"
            if sw > 0 and sh > 0:
                smile_ratio = float(sw) / float(sh)
                if 0.2 <= smile_ratio:
                    # Save the face region image to a folder on the computer
                    now = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                    path = "/home/yun/Documents/ws/MEMBOT/smile_imgs/" + now + ".jpg"
                    cv2.imwrite(path, roi_color)
                    #cv2.imwrite("/home/yun/Documents/ws/MEMBOT/smile_imgs/smile_face.jpg", roi_color)
                    print("SMILE :)", now)

    return frame


def motor_control():
    global length, face_x
    while is_running:
        if length > 300:
            #print("FORWARD")
            myMotor.run(Adafruit_MotorHAT.FORWARD)
            myMotor2.run(Adafruit_MotorHAT.FORWARD)
            myMotor3.run(Adafruit_MotorHAT.FORWARD)
            myMotor4.run(Adafruit_MotorHAT.FORWARD)
        elif length < 200:
            #print("BACKWARD")
            myMotor.run(Adafruit_MotorHAT.BACKWARD)
            myMotor2.run(Adafruit_MotorHAT.BACKWARD)
            myMotor3.run(Adafruit_MotorHAT.BACKWARD)
            myMotor4.run(Adafruit_MotorHAT.BACKWARD)

        else:
            #print("KEEP POSITION")
            myMotor.run(Adafruit_MotorHAT.RELEASE)
            myMotor2.run(Adafruit_MotorHAT.RELEASE)
            myMotor3.run(Adafruit_MotorHAT.RELEASE)
            myMotor4.run(Adafruit_MotorHAT.RELEASE)
        if face_x <240:
            myMotor.run(Adafruit_MotorHAT.BACKWARD)
            myMotor2.run(Adafruit_MotorHAT.FORWARD)
            myMotor3.run(Adafruit_MotorHAT.BACKWARD)
            myMotor4.run(Adafruit_MotorHAT.FORWARD)
        elif face_x > 720:
            myMotor.run(Adafruit_MotorHAT.FORWARD)
            myMotor2.run(Adafruit_MotorHAT.BACKWARD)
            myMotor3.run(Adafruit_MotorHAT.FORWARD)
            myMotor4.run(Adafruit_MotorHAT.BACKWARD)


def captureFrames():
    global video_frame, thread_lock, video_capture

    # Video capturing from OpenCV
    video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

    while is_running and video_capture.isOpened():
        return_key, frame = video_capture.read()
        if not return_key:
            break

        # Apply face detection on the captured frame
        frame = face_detect(frame)

        # Create a copy of the frame and store it in the global variable,
        # with thread safe access
        with thread_lock:
            video_frame = frame.copy()

        #key = cv2.waitKey(30) & 0xff
        key = cv2.waitKey(1)
        if key == ord("q"):
            print("key input")

    video_capture.release()

def turn_off():
    global video_capture
    while is_running:
        key = cv2.waitKey(10)
        if key == ord("q"):
            print("QUIT")
            break
    video_capture.release()
    

def encodeFrame():
    global thread_lock
    while is_running:
        # Acquire thread_lock to access the global video_frame object
        with thread_lock:
            global video_frame
            if video_frame is None:
                continue
            return_key, encoded_image = cv2.imencode(".jpg", video_frame)
            if not return_key:
                continue

        # Output image as a byte array
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encoded_image) + b'\r\n')

@app.route("/")
def streamFrames():
    return Response(encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")

def signal_handler(sig, frame):
    global is_running
    print("Signal received: shutting down...")
    is_running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

if __name__ == "__main__":
    face_detect_thread = threading.Thread(target=face_detect)
    serial_thread = threading.Thread(target=send_serial_data)
    process_thread = threading.Thread(target=captureFrames)
    flask_thread = threading.Thread(target=app.run, kwargs={'host': '192.168.0.124', 'port': 8000, 'threaded': True})
    motor_thread = threading.Thread(target=motor_control)
    quit_thread = threading.Thread(target=turn_off)

    face_detect_thread.start()
    serial_thread.start()
    process_thread.start()
    flask_thread.start()
    motor_thread.start()
    quit_thread.start()

    face_detect_thread.join()
    serial_thread.join()
    process_thread.join()
    flask_thread.join()
    motor_thread.join()
    quit_thread.join()

