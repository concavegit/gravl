import sys
import rosbag
import cv2
import time
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge, CvBridgeError

# if you have bag name as a command line argument.
if len(sys.argv) > 1:
    # make the bridge
    bridge = CvBridge()
    # get the data from the specified bag file
    bagname = sys.argv[1]
    bag = rosbag.Bag(bagname)
    # iterate through the messages and display them
    for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']):
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        min_green = (25,10,10)
        max_green = (95,255,255)
        green_mask = cv2.inRange(cv_image, min_green, max_green)
        green_mask = 255 - green_mask
        cv_image = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
        
        min_white = (0,0,100)
        max_white = (180, 100, 255)
        white_mask = cv2.inRange(cv_image, min_white, max_white)
        white_mask = 255 - white_mask
        #cv_image = cv2.GaussianBlur(cv_image, (25,25), 0)
        #cv_image = cv2.Canny(cv_image,15,25) 
        cv2.imshow("Image raw", cv_image)
        cv2.waitKey(10)
    print(green_mask)
    # close the bag
    bag.close()
# you done fucked up.
else:
    print("Please specify a bag file as an argument.")


