import cv2
import numpy as np
from picamera2 import Picamera2

picam = Picamera2(1)
config = picam.create_preview_configuration(main={"size": (640, 480)})
picam.configure(config)
picam.start()

def capture_image():
    # Capture the image into a variable (numpy array)
    image_data = picam.capture_array()
    # cv2.imshow("Captured Image", image_data)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return image_data

def empty(a):
  pass

def stackImages(scale,imgArray):
  rows = len(imgArray)
  cols = len(imgArray[0])
  rowsAvailable = isinstance(imgArray[0], list)
  width = imgArray[0][0].shape[1]
  height = imgArray[0][0].shape[0]
  if rowsAvailable:
      for x in range ( 0, rows):
          for y in range(0, cols):
              if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                  imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
              else:
                  imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
              if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
      imageBlank = np.zeros((height, width, 3), np.uint8)
      hor = [imageBlank]*rows
      hor_con = [imageBlank]*rows
      for x in range(0, rows):
          hor[x] = np.hstack(imgArray[x])
      ver = np.vstack(hor)
  else:
      for x in range(0, rows):
          if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
              imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
          else:
              imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
          if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
      hor= np.hstack(imgArray)
      ver = hor
  return ver

#IMAGE PATH PASTE HERE
path = capture_image()
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",19,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",110,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",240,255,empty)
cv2.createTrackbar("Val Min","TrackBars",153,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)

MORPH_KERNEL_SIZE = 5       # Size of structuring element (5x5 is a good start)
MORPH_OPEN_ITERATIONS = 2   # Number of opening operations (removes small noise)
MORPH_CLOSE_ITERATIONS = 3  # Number of closing operations (fills small gaps)

def clean_mask(mask):
    # Create structuring element (kernel) for morphological operations
    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    
    # Opening: Erosion followed by Dilation
    # Removes small white noise/spots while preserving larger shapes
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=MORPH_OPEN_ITERATIONS)
    
    # Closing: Dilation followed by Erosion
    # Fills small black holes inside white regions and connects nearby regions
    cleaned_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=MORPH_CLOSE_ITERATIONS)
    
    return cleaned_mask


while True:
  img = capture_image()
  img = cv2.GaussianBlur(img, (5, 5), 0)
  imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

  h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
  h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
  s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
  s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
  v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
  v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
  print(h_min,h_max,s_min,s_max,v_min,v_max)
  lower = np.array([h_min,s_min,v_min])
  upper = np.array([h_max,s_max,v_max])
  mask = cv2.inRange(imgHSV,lower,upper)
  mask = clean_mask(mask)
  imgResult = cv2.bitwise_and(img,img,mask=mask)


  cv2.imshow("Original",img)
  cv2.imshow("HSV",imgHSV)
  cv2.imshow("Mask", mask)
  cv2.imshow("Result", imgResult)

  #imgStack = stackImages(0.6,([img,imgHSV],[mask,imgResult]))
  #cv2.imshow("Stacked Images", imgStack)

  cv2.waitKey(1)