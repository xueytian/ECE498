import cv2
import numpy as np
import studentVision
import line_fit

test_student_vision = 1

detector = studentVision.lanenet_detector()
img_input = cv2.imread("test.png")

img_output_gradient = detector.gradient_thresh(img_input)
img_output_color = detector.color_thresh(img_input)
img_output_combined = detector.combinedBinaryImage(img_input)
img_output_warped, M, Minv = detector.perspective_transform(img_output_combined)

ret = line_fit.line_fit(img_output_warped)
left_fit = ret['left_fit']
right_fit = ret['right_fit']
nonzerox = ret['nonzerox']
nonzeroy = ret['nonzeroy']
img_output_fit = ret['out_img']
left_lane_inds = ret['left_lane_inds']
right_lane_inds = ret['right_lane_inds']

if (test_student_vision == 1):
	cv2.imshow('img_input', img_input)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imshow('img_output_gradient', (img_output_gradient * 255).astype(np.uint8))
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imshow('img_output_color', (img_output_color * 255).astype(np.uint8))
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imshow('img_output_combined', (img_output_combined * 255).astype(np.uint8))
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imshow('img_output_warped', (img_output_warped * 255).astype(np.uint8))
	cv2.waitKey(0)
	cv2.destroyAllWindows()
else:
	cv2.imshow('img_output_fit', img_output_fit)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
