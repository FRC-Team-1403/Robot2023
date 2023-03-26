from keras import regularizers
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Conv2D, Flatten, Dropout, MaxPooling2D
import pandas as pd
import cv2
import matplotlib.pyplot as plt

vid = cv2.VideoCapture(2, cv2.CAP_V4L2)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)

train = pd.read_csv('training.csv')

newid = [str(i) for i in train['Id']]
images = [cv2.imread('Training/' + j) for j in newid]
resized = [cv2.resize(i, (400, 300)) for i in images]


cla = Sequential()

cla.add(Conv2D(16, (3, 3)), input_shape = resized[0].shape, activation = 'relu')

cla.add(MaxPooling2D(pool_size = (2, 2)))

cla.add(Conv2D(32, (3, 3)))
cla.add(MaxPooling2D(pool_size = (2, 2)))

cla.add(Conv2D(64, (3, 3)))
cla.add(MaxPooling2D(pool_size = (2, 2)))

cla.add(Flatten())

cla.add(Dense(units = 128, activation = 'relu', kernel_regularizer=regularizers.l2(0.05)))

cla.add(Dense(units = 1, activation = 'sigmoid', kernel_regularizer=regularizers.l2(0.01)))

cla.compile(optimizer = 'adam', loss = 'binary_crossentropy', metrics = ['accuracy'])
cla.fit(images, train.Category.values, epochs=120, batch_size=32, validation_split=0.1)

while True:

    _, frame = vid.read()

    result = cla.predict([frame])

    result = np.round(result).astype('int64')



    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break


vid.release()

cv2.destroyAllWindows()

