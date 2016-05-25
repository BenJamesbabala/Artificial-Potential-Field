import cv2

cap = cv2.VideoCapture(1)
_, im = cap.read()

cv2.imwrite('sample1.jpg', im)