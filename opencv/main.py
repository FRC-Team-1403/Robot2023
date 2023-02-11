
import cv2
import time
import numpy as np

resize_size = 60
label_txt = np.empty((1, 1)).astype('float32')
feature_txt = np.empty((1, resize_size ** 2)).astype('float32')
knn = cv2.ml.KNearest_create()
feature_txt = np.loadtxt("feature.txt", np.float32)
print(feature_txt.shape)
label_txt = np.loadtxt("label.txt", np.float32).reshape((feature_txt.shape[0], 1))
knn.train(feature_txt, cv2.ml.ROW_SAMPLE, label_txt)

vid = cv2.VideoCapture(0, cv2.CAP_V4L2)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)


H_low = 9
H_high = 52
S_low = 100
S_high = 255
V_low = 165
V_high = 255

debug_msg = False
debug_pic = False

if(debug_msg):
    print(np.array([[ord('c')]], dtype=np.float32).shape)
    print(label_txt.shape)


def label_img(frame):
    cv2.imshow("looks good?", frame)
    key = chr(cv2.waitKey(0)).lower()
    cv2.destroyWindow("looks good?")
    if (key == 'u' or key == 's' or key == 'h'):
        arr = frame.reshape(1, resize_size ** 2)
        return np.array([[ord(key)]], dtype=np.float32), arr
    else:
        return ()


# trackbar callback fucntion to update HSV value
def callback(x):
    global H_low, H_high, S_high, S_low, V_low, V_high
    # assign trackbar position value to H,S,V High and low variable
    H_low = cv2.getTrackbarPos('low H', 'controls')
    H_high = cv2.getTrackbarPos('high H', 'controls')
    S_low = cv2.getTrackbarPos('low S', 'controls')
    S_high = cv2.getTrackbarPos('high S', 'controls')
    V_low = cv2.getTrackbarPos('low V', 'controls')
    V_high = cv2.getTrackbarPos('high V', 'controls')


recal = True

# create a seperate window named 'controls' for trackbar

if (recal):
    cv2.namedWindow('controls', 2)
    cv2.resizeWindow("controls", 550, 10)

    # create trackbars for high,low H,S,V
    cv2.createTrackbar('low H', 'controls', H_low, 255, callback)
    cv2.createTrackbar('high H', 'controls', H_high, 255, callback)

    cv2.createTrackbar('low S', 'controls', S_low, 255, callback)
    cv2.createTrackbar('high S', 'controls', S_high, 255, callback)

    cv2.createTrackbar('low V', 'controls', V_low, 255, callback)
    cv2.createTrackbar('high V', 'controls', V_high, 255, callback)

min_cone_area = 1000

# cone_vert_mask = cv2.imread("cone_upward.png")
# cone_vert_mask = cv2.cvtColor(cone_vert_mask, cv2.COLOR_BGR2GRAY)
# cone_vert_mask = cv2.resize(cone_vert_mask, (80, 160))



while True:
    start = time.process_time()
    _, frame = vid.read()

    #frame = cv2.bilateralFilter(frame, 3, 21, 31)
    frame = cv2.GaussianBlur(frame, (3,3), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hsv_low = np.array([H_low, S_low, V_low])
    hsv_high = np.array([H_high, S_high, V_high])
    
    new_img = cv2.bitwise_and(frame, frame, mask=cv2.inRange(hsv, hsv_low, hsv_high))

    new_img_gray = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(new_img_gray.shape[:2], dtype=new_img_gray.dtype)
    # new_img_gray2 = cv2.adaptiveThreshold(new_img_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    # new_img_gray = cv2.Canny(new_img_gray, 100,200)
    (contours2, _) = cv2.findContours(new_img_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = []

    for cont in contours2:
        x, y, w, h = cv2.boundingRect(cont)
        if (w / h >= 0.5 and w / h <= 1.8 and w*h >= min_cone_area):
            contours.append(cont)

    if (len(contours) > 0):
        cont = max(contours, key=cv2.contourArea)
        cv2.drawContours(frame, [cont], -1, 255, -1)
        x, y, w, h = cv2.boundingRect(cont)
        if(debug_pic):
            cone = frame[y: y + h, x: x + w]

        hull = cv2.convexHull(cont)

        cv2.drawContours(mask, [cont], -1, 255, -1)
        mask = mask[y: y + h, x: x + w]
        # hull2 = cv2.convexHull(cont, returnPoints = False)
        # if(len(hull) > 0):
        # defects = cv2.convexityDefects(cont, hull2)
        # for i in range(defects.shape[0]):
        # s, e, f, d = defects[i, 0]
        # start = tuple(cont[s][0])
        # end = tuple(cont[e][0])
        # far = tuple(cont[f][0])
        # cv2.line(frame, start, end, [0, 255, 0], 2)
        # cv2.circle(frame, far, 5, [0, 0, 255], -1)

        cv2.drawContours(frame, [hull], -1, (0, 255, 0))

        # print(w / h)

        mask = cv2.resize(mask, (resize_size, resize_size))

        resized_mask = mask.copy()

        resized_mask = resized_mask.reshape((1, resize_size ** 2)).astype(np.float32)

        _, result, nears, dists = knn.findNearest(resized_mask, 5)

        if (dists[0][0] >= 1000000):
            if(debug_msg):
                print("result: " + str(int(result)))
                print("score: " + str(dists[0][0]))

        if (int(result) == 117):
            cv2.putText(frame, 'Upward Cone', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)
        if (int(result) == 115):
            cv2.putText(frame, 'Sideways Cone', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)
        if (int(result) == 104):
            cv2.putText(frame, 'Head on Cone', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

        if(debug_msg):
            print(mask.shape)

    cv2.putText(frame, str(int(1 / (time.process_time() - start))) + " FPS" , (50,200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)
    cv2.imshow('frame', frame)

    # cv2.imshow('wow', new_img_gray2)
    if (len(contours) > 0 and cv2.contourArea(cont) >= min_cone_area and debug_pic):
        cv2.imshow('cone', cone)
        cv2.imshow('contour', mask)
        # if (mask.shape[0] == cone_vert_mask.shape[0] and mask.shape[1] == cone_vert_mask.shape[1]):
        # anded_img = cv2.bitwise_and(mask, mask, mask=cone_vert_mask)

        # cv2.imshow('anded_img', anded_img)

    key_pressed = cv2.waitKey(1) & 0xFF

    if key_pressed == ord('q'):
        break

    if (key_pressed == ord(' ') and len(contours) > 0 and cv2.contourArea(cont) >= min_cone_area):
        output = label_img(mask)
        if (len(output) > 1):
            label_txt = np.append(label_txt, output[0], axis=0)
            feature_txt = np.append(feature_txt, output[1], axis=0)
            knn.train(feature_txt, cv2.ml.ROW_SAMPLE, label_txt)

cv2.destroyAllWindows()

print("write? (y/n)")
if str(input())[0] == 'y':
    np.savetxt("label.txt", label_txt)
    np.savetxt("feature.txt", feature_txt)
    print("data has been written")

vid.release()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
