import numpy as np
import cv2
import random as rng

cap = cv2.VideoCapture(2)

# check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

# read until video is completed
while(cap.isOpened()):
  # capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
    # convert the image to HSV because easier to represent color in
    # HSV as opposed to in BGR 
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of orange traffic cone color in HSV
    lower_orange1 = np.array([0, 135, 135])
    lower_orange2 = np.array([15, 255, 255])
    upper_orange1 = np.array([159, 135, 80])
    upper_orange2 = np.array([179, 255, 255])

    # threshold the HSV image to get only bright orange colors
    imgThreshLow = cv2.inRange(hsv_img, lower_orange1, lower_orange2)
    imgThreshHigh = cv2.inRange(hsv_img, upper_orange1, upper_orange2)

    # Bitwise-OR low and high threshes
    threshed_img = cv2.bitwise_or(imgThreshLow, imgThreshHigh)

    # smooth the image with erosion, dialation, and smooth gaussian
    # first create a kernel with standard size of 5x5 pixels
    kernel = np.ones((5,5),np.uint8)

    # get rid of small artifacts by eroding first and then dialating 
    threshed_img_smooth = cv2.erode(threshed_img, kernel, iterations = 3)
    threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations = 2)

    # account for cones with reflective tape by dialating first to bridge the gap between one orange edge
    # and another and then erode to bring the traffic cone back to standard size
    smoothed_img = cv2.dilate(threshed_img_smooth, kernel, iterations = 11)
    smoothed_img = cv2.erode(smoothed_img, kernel, iterations = 7)

    cv2.imshow('smooth', smoothed_img)

    # detect all edges witin the image
    edges_img = cv2.Canny(smoothed_img, 100, 200)
    cv2.imshow('frame2', edges_img)
    contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # set parameters for writing text and drawing lines
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 2
    fontColor = (0, 0, 255)
    lineType = 2

    # analyze each contour and deterime if it is a triangle
    for cnt in contours:
        boundingRect = cv2.boundingRect(cnt)
        approx = cv2.approxPolyDP(cnt, 0.06 * cv2.arcLength(cnt, True), True)
        # if the contour is a triangle, draw a bounding box around it and tag a traffic_cone label to it
        if len(approx) == 3:
            x, y, w, h = cv2.boundingRect(approx)
            rect = (x, y, w, h)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
            bottomLeftCornerOfText = (x, y)
            cv2.putText(frame,'traffic_cone', 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)

    # display the resulting frame
    cv2.imshow('Frame',frame)
 
    # press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # break the loop
  else: 
    break
 
# when everything done, release the video capture object
cap.release()
 
# closes all the frames
cv2.destroyAllWindows()