import cv2
import imutils
import numpy as np
import pdb
import rospy

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template, image_pub=None):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	bounding_box = ((0,0),(0,0))

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert image to hsv

	mask = cv2.inRange(hsv,(1, 130, 20),(30, 255, 255)) # apply mask to keep orange color

	# erosion & dilation
	kernel = np.ones((6,6), np.uint8)
	just_erosion = cv2.erode(mask, kernel, iterations=3)
	erosion_dilation = cv2.dilate(just_erosion, kernel, iterations=4)
	# erosion_dilation = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=3) # check whether it works better than ero, dil or not

    # image_print(erosion_dilation)

	# TODO CONTOURS (necessary?)
	_, contours, hierarchy = cv2.findContours(erosion_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        coord_to_h = {}
        coord_to_s = {}
        coord_to_v = {}

        # sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        def color(cnt):
                cnt_mask = np.zeros(hsv.shape[:2], np.uint8)
                cv2.drawContours(cnt_mask,[cnt],0,255,-1)
                means = cv2.mean(hsv, mask=np.logical_and(cnt_mask > 0, mask > 0).astype(np.uint8))
                h,s,v, _  = means
                rect = cv2.boundingRect(cnt)
                # print 'HSV: ', means
                # print "Rect: " , rect
                coord_to_h[tuple(rect[0:2])] = h
                coord_to_s[tuple(rect[0:2])] = s
                coord_to_v[tuple(rect[0:2])] = v
                good_s = min(2*220 - s, s) # = s up to 220, then decreasing
                good_v = min(2*180 - v, v) # = v up to 180, then decreasing
                # print "calculated s, v: " , good_s, good_v
                # if s > 240 or v > 220:
                #         # too bright, could be a reflection
                #         return 0 # this didn't work
                # return good_h + good_s + good_v
                return good_s + good_v        # Saturation + Value makes cone look bright
        sorted_contours = sorted(contours, key=lambda x: color(x), reverse=True)

	if len(contours) == 0:
		return bounding_box

	rects = [cv2.boundingRect(contour) for contour in sorted_contours]

	ratio_rects = [((float(h)/float(w)),(x,y,w,h)) for x,y,w,h in rects]
	# cone_ratio = 2 # dilation and erosion usually turn cone skinnier
	# def closeness_to_cone(ratio_rect):
	# 	ratio, rect = ratio_rect
	# 	percent_of_cone_ratio = abs(ratio - cone_ratio) / cone_ratio
        #         if ratio < 1:
        #                 return 0 # reject squat rectangles
	# 	return percent_of_cone_ratio
        # most_cone_like_ratio, most_cone_like_rect = max(ratio_rects, key=closeness_to_cone)
        for x,y,w,h in rects:
                pass
                # print "width:", w
                # print "height:", h
        min_cone_like_ratio, max_cone_like_ratio = 0.8, 5
        cone_like_rects = [rect for ratio, rect in ratio_rects if min_cone_like_ratio < ratio < max_cone_like_ratio]
        if cone_like_rects:
                best_cone_like_rect = cone_like_rects[0]
        else:
                best_cone_like_ratio, best_cone_like_rect = ratio_rects[0] # just biggest rect if none are conelike

	# bounding rectangle
	# x,y,w,h = cv2.boundingRect(erosion_dilation)
	# x, y, w, h = most_cone_like_rect
        x, y, w, h = best_cone_like_rect
	bounding_box = ((x,y),(x+w,y+h))

	for rect in rects:
                x,y,w,h = rect
	        box = ((x,y),(x+w,y+h))
                cv2.rectangle(img, box[0], box[1], (255,0,0), 2)
	for rect in cone_like_rects:
                x,y,w,h = rect
	        box = ((x,y),(x+w,y+h))
                cv2.rectangle(img, box[0], box[1], (0,0,255), 2)

        best_h, best_s, best_v = coord_to_h[bounding_box[0]], coord_to_s[bounding_box[0]], coord_to_v[bounding_box[0]]
        bgr = cv2.cvtColor(np.array([[[best_h, best_s, best_v]]]).astype(np.uint8), cv2.COLOR_HSV2BGR)
        cv2.rectangle(img, bounding_box[0], bounding_box[1], tuple(int(x) for x in bgr[0][0]), 2)

        if image_pub is not None:
                image_pub(img)

        # print "BEST: ", best_h, best_s, best_v
        rospy.loginfo("good h: %s", coord_to_h[bounding_box[0]])
        rospy.loginfo("good s: %s", coord_to_s[bounding_box[0]])
        rospy.loginfo("good v: %s", coord_to_v[bounding_box[0]])

        # import random
        # if random.random() < 0.2:
	#         image_print(img)

        # image_print(img)
	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

def lookahead(img,top = 2, bottom = 5):
	'''
	restricts view of image
	creates a black bar of height = h/top (default = h/2) on top
	black bar of height = h/bottom (default = h/5) on bottom
	'''
	h = len(img)
        w = len(img[0])
	black = [0, 0, 0]

        crop = img.copy()

        for i in range(h/top):
                crop[i] = [black for j in range(w)]
        for i in range(h-h/bottom,h):
                crop[i] = [black for j in range(w)]

        return crop

# img = cv2.imread('test_images_cone/test1.jpg')
# crop = lookahead(img)
# image_print(crop)
