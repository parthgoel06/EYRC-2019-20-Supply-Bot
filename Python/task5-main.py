''' 
* Team Id : 8401
* Author List : Parth Goel, Shivika Singh
* Filename: task5-main.py
* Theme: Supply Bot
* Functions: calc_angle, assign_node, process, main
* Global Variables: None
'''

######################
## Essential libraries
######################
import cv2
import numpy as np
import os
import math
import csv
import copy
import time
import cv2.aruco as aruco
from aruco_lib import *
import serial

''' 
* Function Name: calc_angle

* Input: centre_x, centre_y (coordinates of arena centre); x1, y1 (coordinates of a point 1); x2, y2 (coordinates of a point 2)

* Output: Returns angle between two points after calculating it

* Logic: This function takes 6 parameters as mentioned above and calculates angles between two points on the circle by 
         calculating slope of those points from the centre of the circle and applying general co-ordinate geometry formulas. 
         Also this function always find the minimum angle.

* Example Call: calc_angle(centre_x,centre_y,point1_x,point1_y,point2_x,point2_y)
'''
def calc_angle(centre_x,centre_y,x1,y1,x2,y2):
    m1 = (int(centre_y) - int(y1))/(int(centre_x) - int(x1))    #m1: slope of line from point 1 to centre of cirlce
    m2 = (int(centre_y) - int(y2))/(int(centre_x) - int(x2))    #m2: slope of line from point 2 to centre of cirlce
    tan_angle = (m2 - m1)/(1 + m1*m2)
    angle = abs(math.degrees(math.atan(tan_angle)))
    distance_1_2 = math.sqrt((int(x1) - int(x2))**2 + (int(y1) - int(y2))**2) #linear distance between point 1 and 2
    
    #this code always calculates inner or minimum angle 
    default_distance = math.sqrt((int(centre_y) - int(y1))**2 + (int(centre_x) - int(x1))**2)*math.sqrt(2)  
    if distance_1_2 > default_distance:
        angle = 180 - angle
    angle = float("{0:.2f}".format(angle))

    return angle

''' 
* Function Name: assign_node

* Input: angle (angle between bot and detected node circle)

* Output: Returns node number

* Logic: This function takes angle as a parameter and returns the node the bot has to visit. 
'''

def assign_node(angle):
    if 0<angle<10:
        return 1
    if 35<angle<55:
        return 2
    if 70<angle<85:
        return 3
    if 110<=angle<=125:
        return 4
    if 140<=angle<=160:
        return 5
    if 175<=angle<=190:
        return 6
    if 215<=angle<=235:
        return 7
    if 270<=angle<=290:
        return 8
    if 295<=angle<=310:
        return 9
    if 350<angle<360:
        return 1

''' 
* Function Name: process

* Input: ip_image (image as an input)

* Output: op_image(processed output image),3 angle variables, situation(coin orientation), aruco_x, aruco_y(x and y coordinates of aruco id)

* Logic: This function takes an image as an input and does image processing on it. Firstly, this function puts a black circular mask 
         on the image to hide outer boundaries of the image so that the detection becomes easier and faulty detections becomes less. 
         Then this function takes the original image and performs thresholding on it twice (one for green color and one for red). Then we get two seperate images,
         with green and red thresholding and we take these two images to perform hough circle detection on those images. After detecting green and red circles 
         we detect the aruco id of the bot and then find the angles between the bot and all the coins(red and green circles).
'''
def process(ip_image):
    ###########################
    ## Your Code goes here
    ###########################
    try:
        #code for orange color masking to detect centre co-ordinates of the flex
        blur = cv2.blur(ip_image,(5,5))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower_range = np.array([150,100,50])
        upper_range = np.array([255,255,244])
        mask_orange = cv2.inRange(hsv, lower_range, upper_range) #mask_orange: thresholding the image to show only orange color
        cv2.imwrite('mask_O.jpg',mask_orange)
        mask_orange = cv2.imread('mask_O.jpg')
        mask_gray = cv2.cvtColor(mask_orange, cv2.COLOR_BGR2GRAY)
        circle_centre = cv2.HoughCircles(mask_gray, cv2.HOUGH_GRADIENT, 3.3, 10, param1 = 200,      
                    param2 = 100, minRadius = 35, maxRadius = 45)
        if circle_centre is not None:
            circle_centre = np.round(circle_centre[0, :]).astype("int")
            for (x, y, r) in circle_centre:
                cv2.circle(mask_orange, (x, y), r, (255, 0, 255), 2)
        centre_x,centre_y,radius = circle_centre[0][0],circle_centre[0][1],circle_centre[0][2]
        
        #code for putting a black circular mask on the image with centre as the centre of flex
        img = ip_image.copy()
        cv2.circle(img, (centre_x,centre_y), 325, (0,0,0), thickness=200, lineType=8, shift=0) 
        
        #code for red color thresholding on the image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_range = np.array([169,125,100])
        upper_range = np.array([250,250,244])
        mask_red = cv2.inRange(hsv, lower_range, upper_range)
        cv2.circle(mask_red, (centre_x,centre_y), radius, (0,0,0), thickness=80, lineType=8, shift=0)
        cv2.imwrite('r.jpg',mask_red)
        
        #code for green color thresholding on the image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_range = np.array([10,40,150])
        upper_range = np.array([90,255,255])
        mask_green = cv2.inRange(hsv, lower_range, upper_range)
        cv2.imwrite('g.jpg',mask_green)

        #code for hough circle detection on green and red thresholded images and displaying the detected circles on the original images 
        red = cv2.imread('r.jpg')
        green = cv2.imread('g.jpg')
        grayG = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        circleG = cv2.HoughCircles(grayG, cv2.HOUGH_GRADIENT, 1.5, 10, param1 = 32, 
                    param2 = 15, minRadius = 4, maxRadius = 6)
        circleR = cv2.HoughCircles(grayR, cv2.HOUGH_GRADIENT, 2, 10, param1 = 24, 
                    param2 = 11, minRadius = 4, maxRadius = 6)
        if circleG is not None:
            circleG = np.round(circleG[0, :]).astype("int")
            for (x, y, r) in circleG:
                cv2.circle(ip_image, (x, y), r, (255, 0, 0), 2)
        if circleR is not None:
            circleR = np.round(circleR[0, :]).astype("int")
            for (x, y, r) in circleR:
                cv2.circle(ip_image, (x, y), r, (255, 0, 0), 2)

        #code for detecting and marking aruco id             
        ls = detect_Aruco(ip_image)
        id_dict = calculate_Robot_State(ip_image,ls)
        id_list = id_dict[25]
        aruco_x,aruco_y = id_list[1],id_list[2]

        #code to remove extra images
        os.remove('r.jpg')
        os.remove('g.jpg')
        os.remove('mask_O.jpg')

        #code for calculating angles between aruco id(bot) and detected coins
        if len(circleG)>1:                                                                      #CONDITION if 2 green coins and 1 red coin
            circleG1_x, circleG1_y = circleG[0][0], circleG[0][1]
            circleR_x, circleR_y = circleR[0][0], circleR[0][1]
            circleG2_x, circleG2_y = circleG[1][0], circleG[1][1]
            angle_a_g1 = calc_angle(centre_x,centre_y,aruco_x,aruco_y,circleG1_x,circleG1_y)
            angle_a_g2 = calc_angle(centre_x,centre_y,aruco_x,aruco_y,circleG2_x,circleG2_y)
            angle_a_R = calc_angle(centre_x,centre_y,aruco_x,aruco_y,circleR_x,circleR_y)
            if circleG1_x > centre_x:
                angle_a_g1 = 360 - angle_a_g1 
            if circleG2_x > centre_x:
                angle_a_g2 = 360 - angle_a_g2
            if circleR_x > centre_x:
                angle_a_R = 360 - angle_a_R
            
            op_image = ip_image
            situation = 0                                                                       #situation: to tell which situation has occurred to the main function
            
            return op_image,angle_a_R,angle_a_g1,angle_a_g2,situation,aruco_x,aruco_y                           #situation = 0 means 2 green coins and 1 red coins
        
        else:                                                                                    #CONDITION if 1 green coin and 2 red coins
            circleG_x, circleG_y = circleG[0][0], circleG[0][1]
            circleR1_x, circleR2_y = circleR[0][0], circleR[0][1]
            circleR2_x, circleR2_y = circleR[1][0], circleR[1][1]
            angle_a_g = calc_angle(centre_x,centre_y,aruco_x,aruco_y,circleG_x,circleG_y)
            angle_a_R1 = calc_angle(centre_x,centre_y,aruco_x,aruco_y,circleR1_x,circleR1_y)
            angle_a_R2 = calc_angle(centre_x,centre_y,aruco_x,aruco_y,circleR2_x,circleR2_y)
            if circleG_x > centre_x:
                angle_a_g = 360 - angle_a_g 
            if circleR2_x > centre_x:
                angle_a_R2 = 360 - angle_a_R2
            if circleR1_x > centre_x:
                angle_a_R1 = 360 - angle_a_R1

            op_image = ip_image
            situation = 1                                                                        #situation = 1 means 1 green coins and 2 red coins
            
            return op_image,angle_a_g,angle_a_R1,angle_a_R2,situation,aruco_x,aruco_y
    except:
        pass

    
####################################################################
## The main program which provides read in input of one image at a
## time to process function 
####################################################################
def main():
    ser = serial.Serial('COM9', 9600, timeout=1)
    
    #Some flag variables to be used later in the code
    flag1 = 0
    flag2 = 0
    flag3 = 0
    
    i = 1
    
    ## reading in video 
    cap = cv2.VideoCapture(1) #if you have a webcam on your system, then change 0 to 1 
    
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    try:
        img,angle1,angle2,angle3,situation,capital_x,capital_y = process(frame)          ##retrieving image, angles, coin configuration and capital co-ordinates 
                                                                                         #situation from process function
        ##In this piece of code we are assigning node values to the coins and 
        #printing the sequence of bot traversal on python console 
        if situation == 0:                                                               #situation = 0 means 2 green coins and 1 red coin
            angle_a_R = angle1
            angle_a_g1 = angle2
            angle_a_g2 =  angle3    
            print(angle_a_R)
            print(angle_a_g1)
            print(angle_a_g2)

            node_R = assign_node(angle_a_R)                                              #node_R: node at which red coin is placed
            node_g1 = assign_node(angle_a_g1)                                            #node_g1: node at which 1st green coin is placed
            node_g2 = assign_node(angle_a_g2)                                            #node_g2: node at which 2nd green coin is placed
            print(node_R)
            print(node_g1)
            print(node_g2)

            if ((node_R > node_g1) and (node_R < node_g2)) or ((node_R < node_g1) and (node_R > node_g2)):      #condition if red coin is in between 2 green coins
                print(f"1)Red Coin on Node-{node_R}")                                                           #bot first goes to red coin
                if node_g1 > node_g2:                                                                           #condition to check which is the next nearest green coin
                    print(f"2)Green Coin on Node-{node_g2}")                         
                    print(f"3)Green Coin on Node-{node_g1}") 
                else:
                    print(f"2)Green Coin on Node-{node_g1}")
                    print(f"3)Green Coin on Node-{node_g2}")
            
            else:                                                                                        #condition if red coin is NOT in between 2 green coins
                print(f"1)Red Coin on Node-{node_R}") 
                if node_g1 < node_g2:                                                                    #condition to check which is the next nearest green coin                                                  
                    print(f"2)Green Coin on Node-{node_g2}")                         
                    print(f"3)Green Coin on Node-{node_g1}") 
                else:
                    print(f"2)Green Coin on Node-{node_g1}")
                    print(f"3)Green Coin on Node-{node_g2}")        
            
        
        elif situation == 1:                                                    #situation = 1 means 1 green coin and 2 red coins
            angle_a_g = angle1
            angle_a_R1 = angle2
            angle_a_R2 =  angle3      

            node_g = assign_node(angle_a_g)                                     #node_g: node at which green coin is placed
            node_R1 = assign_node(angle_a_R1)                                   #node_R1: node at which 1st red coin is placed
            node_R2 = assign_node(angle_a_R2)                                   #node_R2: node at which 2nd red coin is placed
            
            if node_R1 < node_R2:                                               #condition to check which red coin is nearer  
                print(f"1)Red Coin on Node-{node_R2}")
                print(f"2)Red Coin on Node-{node_R1}")
            else:
                print(f"1)Red Coin on Node-{node_R1}")
                print(f"2)Red Coin on Node-{node_R2}")
            
            print(f"3)Green Coin on Node-{node_g}")
    except:
        pass

    while(ret):
        ret, frame = cap.read()
        cv2.waitKey(int(1000/fps));
        ## calling the algorithm function
        try:
            op_image,angle1,angle2,angle3,situation,bot_x,bot_y = process(frame)   
            ## display to see if the frame is correct
            cv2.imshow("window1", op_image)
            
            ##Now the code checks the  coin configuration i.e. if there are '2 green and 1 red' or else '2 red or 1 green coin', 
            #and sends striking signals in respective situations by calculating angle between the bot and all the coins
            
            if situation == 0:                                      #SITUATION: 1 red coin and 2 green coin
                angle_aruco_red   = angle1
                angle_aruco_green1 = angle2
                angle_aruco_green2 = angle3

                if angle_aruco_red==0:                              #checks condition if angle between aruco id and red coin is zero
                    if flag1 == 1:                                  #if red coin has already been hit i.e flag1=1 ,it doesn't send strike signal again for red coin
                        continue
                    ser.write(bytes('2','utf-8'))                   #Sends strike signal to bot
                    flag1 = 1                                       #Sets flag1 = 1 to indicate that red coin has been hit


                elif angle_aruco_green1==0 and flag1 == 1:          #checks condition that 'angle b/w aruco and green1 coin is zero' AND 'red coin has been hit'
                    if flag2 == 1:                                  #if green1 coin has already been hit i.e flag2=1 ,it doesn't send strike signal again for green1 coin
                        continue
                    ser.write(bytes('2','utf-8'))                   #Sends strike signal to bot
                    flag2 = 1                                       #Sets flag2 = 1 to indicate that grren1 coin has been hit


                elif angle_aruco_green2==0 and flag1 == 1:          #Checks condition 'angle b/w aruco and green2 is zero' AND 'red coin has been hit'       
                    if flag3 == 1:                                  #if green2 coin has already been hit i.e flag3=1 ,it doesn't send strike signal again for green2 coin
                        continue
                    ser.write(bytes('2','utf-8'))                   #Sends strike signal to bot
                    flag3 = 1                                       #Sets flag3 = 1 to indicate that green2 coin has been hit

                ##Stops the bot at the capital after striking all 
                #coins i.e all flags are 1 and bot coordinates are 
                #same as that of capital coordinates
                if flag1 == 1 and flag2 == 1 and flag3 == 1 and capital_x == bot_x and capital_y == bot_y: 
                    ser.write(bytes('1','utf-8'))                  


            else:                                                   #SITUATION : 2 red coins 1 green coin
                angle_aruco_green   = angle1
                angle_aruco_red1 = angle2
                angle_aruco_red2 = angle3

                if angle_aruco_red1==0:                             #checks condition if angle between aruco id and red1 coin is zero
                    if flag1 == 1:                                  #if red1 coin has already been hit i.e flag1=1 ,it doesn't send strike signal again for red1 coin           
                        continue
                    ser.write(bytes('2','utf-8'))                   #Sends strike signal to bot
                    flag1 = 1                                       #Sets flag1 = 1 to indicate that red1 coin has been hit

                elif angle_aruco_red2==0:                           #checks condition if angle between aruco id and red1 coin2 is zero
                    if flag2 == 1:                                  #if red1 coin2 has already been hit i.e flag1=1 ,it doesn't send strike signal again for red2 coin
                        continue
                    ser.write(bytes('2','utf-8'))                   #Sends strike signal to bot
                    flag2 = 1                                       #Sets flag2 = 1 to indicate that red2 coin has been hit

                elif angle_aruco_green == 0 and flag1 == 1 and flag2 == 1:  #checks condition if 'angle b/w aruco id and green coin is zero' AND 'both red coins are hit'
                    if flag3 == 1:                                  #if green coin has already been hit i.e flag1=1 ,it doesn't send strike signal again for green coin
                        continue
                    ser.write(bytes('2','utf-8'))                   #Sends strike signal to bot
                    flag3 = 1                                       #Sets flag3 = 1 to indicate that green coin has been hit
        except:
            cv2.imshow("window1", frame)
    cap.release()
    cv2.destroyAllWindows()
    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
