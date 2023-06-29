import cv2
import time
import datetime


# 비디오 캡처 객체 생성
cap = cv2.VideoCapture(0)  # 0은 디폴트 웹캠을 의미합니다. 다른 번호를 사용하여 다른 카메라를 선택할 수 있습니다.

face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
smile_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_smile.xml")

while True:
    # 프레임 읽기
    ret, frame = cap.read()

    # 프레임이 제대로 읽혔는지 확인
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        # if len(faces) > 0:
        #     x, y, w, h = faces[0]
        #     length = w
        #     face_x_temp = x + w/2
        #     face_y_temp = y + h/2
        #     if 0 < face_x_temp < 960 and 0 < face_y_temp < 540:
        #         #print("center X : ", face_x_temp, " center Y : ", face_y_temp, "lenght : ", length)
        #         face_x, face_y = face_x_temp, face_y_temp
        #     else:
        #         face_x, face_y = 480, 270

        if len(faces) > 0:
            x, y, w, h = faces[0]
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            #cv2.imshow('face', frame)

            # Capture the face region
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = frame[y:y+h, x:x+w]

            # Classify the facial expression
            smiles = smile_cascade.detectMultiScale(roi_gray, 1.8, 20)
            if len(smiles)>0:
                for (sx, sy, sw, sh) in smiles:
                    cv2.rectangle(roi_color, (sx, sy), (sx+sw, sy+sh), (0, 255, 0), 2)
            cv2.imshow('smile', roi_color)

            # Check if the expression is "smile"
            # if sw > 0 and sh > 0:
            #     smile_ratio = float(sw) / float(sh)
            #     if 0.2 <= smile_ratio:
            #         # Save the face region image to a folder on the computer
            #         now = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
            #         path = "/home/yun/Desktop/" + now + ".jpg"
            #         cv2.imwrite(path, roi_color)
            #         #cv2.imwrite("/home/yun/Documents/ws/MEMBOT/smile_imgs/smile_face.jpg", roi_color)
            #         print("SMILE :)", now)

            #cv2.imshow('smile', frame)

    

    # 프레임 표시
    

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) == ord('q'):
        break

# 비디오 캡처 객체와 창 해제
cap.release()
cv2.destroyAllWindows()
