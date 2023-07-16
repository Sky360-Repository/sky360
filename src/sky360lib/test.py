import cv2
import pysky360

#algorithm = pysky360.WeightedMovingVariance()
algorithm = pysky360.Vibe()

video_file = "Dahua-20220901-184734.mp4"
capture = cv2.VideoCapture(video_file)

parameters = algorithm.getParameters()
threshold = parameters.getThreshold()
print("threshold: " + str(threshold))

while not capture.isOpened():
    capture = cv2.VideoCapture(video_file)
    cv2.waitKey(1000)
    print("Wait for the header")

while True:
    flag, frame = capture.read()

    if flag:
        frame = cv2.resize(frame, (1024, 1024))
        cv2.imshow('video', frame)
        greyFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
        pos_frame = capture.get(1)
        img_output = algorithm.apply(greyFrame)
        
        cv2.imshow('img_output', img_output)

    else:
        cv2.waitKey(1000)
        break
    keyPressed = cv2.waitKey(10) & 0xFF
    if keyPressed == 27:
        break
    elif keyPressed == ord('+'):
        threshold += 5
        parameters.setThreshold(threshold)
        print("threshold: " + str(threshold))
    elif keyPressed == ord('-'):
        threshold -= 5
        parameters.setThreshold(threshold)
        print("threshold: " + str(threshold))


cv2.destroyAllWindows()