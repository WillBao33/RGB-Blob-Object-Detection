# Standard imports
import cv2
import numpy as np
import serial
ser = serial.Serial('COM4', baudrate=9600, timeout=1)
# Read image
cam = cv2.VideoCapture(1)
font = cv2.FONT_HERSHEY_COMPLEX
sent_coordinates = 53
while(1):
        ret, frame = cam.read()

        if not ret:
            break

        #canvas = frame.copy()

        #red lego starts here
        lower_red = np.array([83, 0, 211])
        upper_red = np.array([179, 255, 255])

        red_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(red_hsv, lower_red, upper_red)
        red = cv2.bitwise_and(frame, frame, mask=red_mask)


        # Setup SimpleBlobDetector parameters.
        red_params = cv2.SimpleBlobDetector_Params()

        # Filter by Area.
        red_params.filterByArea = True
        red_params.minArea = 100
        red_params.maxArea = 500

        # Filter by Circularity
        red_params.filterByCircularity = False
        # params.minCircularity = 0.1

        # Filter by Convexity
        red_params.filterByConvexity = False
        # params.minConvexity = 0.1

        # Filter by Inertia
        red_params.filterByInertia = False
        # params.minInertiaRatio = 0.1

        # Set up the detector with default parameters.
        red_detector = cv2.SimpleBlobDetector_create(red_params)

        reverse_mask = 255-red_mask
        blur = cv2.GaussianBlur(reverse_mask, (9, 9), 0)

        # Detect blobs.
        keypoints = red_detector.detect(blur)

        # M = cv2.moments(cam)
        # cX = int(M["m10"]/M["m00"])
        # cY = int(M["m01"]/M["m00"])
        # cv2.circle(frame,(cX,cY),5,(255,255,255),-1)
        # cv2.putText(frame,"centroid",(cX-25,cY-25),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),2)
        #
        # print(cX)
        # print(cY)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints)>0:
            x = keypoints[0].pt[0]  # i is the index of the blob you want to get the position
            y = keypoints[0].pt[1]
            cv2.putText(im_with_keypoints,str(int(x)),(int(x),int(y)),font,1,(0,0,0))
            cv2.putText(im_with_keypoints,str(int(y)),(int(x),int(y+50)),font,1,(0,0,0))
            #print(int(x),int(y))
            # To send pixy coordinates of the yellow block

            # if bool(sent_coordinates) == 53:
            #
            #     ser.write('x'.encode())
            #     ser.write(str(int(x)).encode())
            #     ser.write('y'.encode())
            #     ser.write(str(int(y)).encode())
            #     sent_coordinate = 54
            #
            # # while 1:  # Takes in all read values till the last character
            # print(ser.read_until())

            # ser.write(str(int(x)).encode())
            # value = ser.readline()
            # print(value)
            # y_coordi =ser.write(int(y))


        # blue lego starts here

        lower_blue = np.array([0, 25, 135])
        upper_blue = np.array([179, 255, 255])

        blue_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(blue_hsv, lower_blue, upper_blue)
        blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

        # Setup SimpleBlobDetector parameters.
        blue_params = cv2.SimpleBlobDetector_Params()

        # Filter by Area.
        blue_params.filterByArea = True
        blue_params.minArea = 50
        blue_params.maxArea = 100

        # Filter by Circularity
        blue_params.filterByCircularity = False
        # params.minCircularity = 0.1

        # Filter by Convexity
        blue_params.filterByConvexity = False
        # params.minConvexity = 0.1

        # Filter by Inertia
        blue_params.filterByInertia = False
        # params.minInertiaRatio = 0.1

        # Set up the detector with default parameters.
        blue_detector = cv2.SimpleBlobDetector_create(blue_params)

        blue_reverse_mask = 255-blue_mask
        blue_blur = cv2.GaussianBlur(blue_reverse_mask, (9, 9), 0)

        # Detect blobs.
        blue_keypoints = blue_detector.detect(blue_blur)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, blue_keypoints, np.array([]), (0, 0, 0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(blue_keypoints)>0:
            x = blue_keypoints[0].pt[0]  # i is the index of the blob you want to get the position
            y = blue_keypoints[0].pt[1]
            cv2.putText(im_with_keypoints,str(int(x)),(int(x),int(y)),font,1,(0,0,0))
            cv2.putText(im_with_keypoints,str(int(y)),(int(x),int(y+50)),font,1,(0,0,0))

        lower_robo = np.array([0, 40, 179])
        upper_robo = np.array([179, 255, 255])

        robo_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        robo_mask = cv2.inRange(robo_hsv, lower_robo, upper_robo)
        robo = cv2.bitwise_and(frame, frame, mask=robo_mask)

        # Setup SimpleBlobDetector parameters.
        robo_params = cv2.SimpleBlobDetector_Params()

        # Filter by Area.
        robo_params.filterByArea = True
        robo_params.minArea = 200
        robo_params.maxArea = 500

        # Filter by Circularity
        robo_params.filterByCircularity = False
        # params.minCircularity = 0.1

        # Filter by Convexity
        robo_params.filterByConvexity = False
        # params.minConvexity = 0.1

        # Filter by Inertia
        robo_params.filterByInertia = False
        # params.minInertiaRatio = 0.1

        # Set up the detector with default parameters.
        robo_detector = cv2.SimpleBlobDetector_create(robo_params)

        robo_reverse_mask = 255-robo_mask
        robo_blur = cv2.GaussianBlur(robo_reverse_mask, (9, 9), 0)

        # Detect blobs.
        robo_keypoints = robo_detector.detect(robo_blur)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, robo_keypoints, np.array([]), (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(robo_keypoints)>0:
            x = robo_keypoints[0].pt[0]  # i is the index of the blob you want to get the position
            y = robo_keypoints[0].pt[1]
            cv2.putText(im_with_keypoints,str(int(x)),(int(x),int(y)),font,1,(0,0,0))
            cv2.putText(im_with_keypoints,str(int(y)),(int(x),int(y+50)),font,1,(0,0,0))


        #
        # cv2.imshow('blue',blue)
        cv2.imshow('result',im_with_keypoints)
        # cv2.imshow('blue_mask',blue_mask)
        # result = cv2.bitwise_and(im_with_keypoints, blue_with_keypoints, mask=red_mask)
        # cv2.imshow('result',result)

        if cv2.waitKey(1) == 27:
                break
cam.release()
cv2.destroyAllWindows()