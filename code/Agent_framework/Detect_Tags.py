import cv2
import numpy as np


class DetectTags:
    def __init__(self):
        super(DetectTags, self).__init__()
        self._tk_label_input = 0
        self.is_moving = False
        self.img1 = None  # trainImage
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.index_params = dict(algorithm=0, trees=5)
        self.search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        self.img2 = cv2.imread('Selection/15.png', 0)   # <= the image viewed by the robot
        self.match_found = False
        self.image = ''

    def check_image(self):
        image_list = list(range(1, 15))
        i = 0
        while self.match_found is False and i < len(image_list):

            self.match(image_list[i])
            i += 1
        if self.match_found is True:
            print('image is', image_list[i-1])
            return image_list[i-1]
        else:
            print('image not found')
            return -1

    def match(self, image_number):
        self.img1 = cv2.imread('Selection/' + str(image_number) + '.png', 0)

        self.kp1, self.des1 = self.sift.detectAndCompute(self.img1, None)
        self.match_found = False
        min_match_count = 35

        # Feature Recognition
        kp2, des2 = self.sift.detectAndCompute(self.img2, None)
        matches = self.flann.knnMatch(self.des1, des2, k=2)

        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
                if len(good) > min_match_count:
                    src_pts = np.float32([self.kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                    m, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    h, w = self.img1.shape
                    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, m)
                    self.img2 = cv2.polylines(self.img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
                    self.match_found = True
        print(str(image_number) + '  matches : ', len(good))

