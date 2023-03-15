import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        raw_img = cv_image.copy() 
        mask_image, bird_image = self.detection(raw_img)

        # Convert an OpenCV image into a ROS image message
        out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
        out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

        # Publish image message in ROS
        self.pub_image.publish(out_img_msg)
        self.pub_bird.publish(out_bird_msg)


    def gradient_thresh(self, img, thresh_min=20, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        ## TODO

        # Convert image to gray
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian filter
        img_blurred = cv2.GaussianBlur(img_gray, (9, 9), 0)

        # Apply Sobel filter
        img_sobel_x = cv2.Sobel(img_blurred, cv2.CV_8U, 1, 0, ksize = 1)
        img_sobel_y = cv2.Sobel(img_blurred, cv2.CV_8U, 0, 1, ksize = 1)

        # Combine images
        img_sum = cv2.addWeighted(img_sobel_x, 0.5, img_sobel_y, 0.5, 0)

        # Apply thresholds
        binary_output = cv2.inRange(img_sum, thresh_min, thresh_max)

        # Normalize pixel values
        binary_output = (binary_output / 255).astype(np.uint8)

        ####

        return binary_output


    def color_thresh(self, img, thresh=(150, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image 

        ## TODO

        # Convert image into HSL color space
        img_hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # Split HSV channels
        img_hls_h, img_hls_l, img_hls_s = cv2.split(img_hls)

        # Apply thresholds
        binary_output = cv2.inRange(img_hls_l, thresh[0], thresh[1])

        # Normalize pixel values
        binary_output = (binary_output / 255).astype(np.uint8)

        ####

        return binary_output


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter 
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want. 

        ## TODO

        # Get gradient-threhold image
        SobelOutput = self.gradient_thresh(img)

        # Get color-threhold image
        ColorOutput = self.color_thresh(img)

        ####

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)

        return binaryImage


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO

        # Get image dimensions
        img_height = img.shape[0]
        img_width = img.shape[1]

        # Define source points
        source_points = np.array([
            [440 , 250			 ], # Upper left
            [150 , img_height - 1], # Lower left
            [740 , 250			 ], # Upper right
            [1000, img_height - 1]  # Lower right
        ], dtype = "float32")

        # Define destination points
        destination_points = np.array([
            [0            , 0             ], # Upper left
            [0            , img_height - 1], # Lower left
            [img_width - 1, 0             ], # Upper right
            [img_width - 1, img_height - 1]  # Lower right
        ], dtype = "float32")

        # Get perspective transformation matrices
        M = cv2.getPerspectiveTransform(source_points, destination_points)
        Minv = np.linalg.inv(M)

        # Get Warped image
        warped_img = cv2.warpPerspective(np.float32(img), M, (img_width, img_height))

        # Apply thresholds
        warped_img = cv2.inRange(warped_img, 1, 255)

        # Normalize pixel values
        warped_img = (warped_img / 255).astype(np.uint8)

        ####

        return warped_img, M, Minv


    def detection(self, img):

        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']
        
        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)
                left_fit = ret['left_fit']
                right_fit = ret['right_fit']
                nonzerox = ret['nonzerox']
                nonzeroy = ret['nonzeroy']
                left_lane_inds = ret['left_lane_inds']
                right_lane_inds = ret['right_lane_inds']

                left_fit = self.left_line.add_fit(left_fit)
                right_fit = self.right_line.add_fit(right_fit)

                self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)
                left_fit = ret['left_fit']
                right_fit = ret['right_fit']
                nonzerox = ret['nonzerox']
                nonzeroy = ret['nonzeroy']
                left_lane_inds = ret['left_lane_inds']
                right_lane_inds = ret['right_lane_inds']

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
            combine_fit_img = final_viz(img, left_fit, right_fit, Minv)


            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
