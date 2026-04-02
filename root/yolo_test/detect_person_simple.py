import cv2

cap = cv2.VideoCapture(0)

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# 获取分辨率（更稳）
width = int(cap.get(3))
height = int(cap.get(4))

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 10.0, (width, height))

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败")
        break

    boxes, _ = hog.detectMultiScale(frame,
                                    winStride=(8,8),
                                    padding=(8,8),
                                    scale=1.05)

    for (x, y, w, h) in boxes:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(frame, "person", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    out.write(frame)

cap.release()
out.release()
