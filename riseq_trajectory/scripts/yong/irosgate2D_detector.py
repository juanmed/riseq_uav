import numpy as np
import cv2


class IROSGateDetector:
    def __init__(self):
        self.window_width = 1.4  # m
        self.window_height = 1.4  # m

        # This params must be initialized with the best performing values
        self.canny_lt = 75
        self.canny_ht = 200
        self.canny_k = 5
        self.epsilon = 0.1  # should be between 0.02 to 0.15
        self.gauss_k = 5  # should be 3 or 5, but no more

        # For how to decide HSV color boundaries, look
        # https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
        green = [([60 - 20, 100, 40], [60 + 20, 255, 255])]
        blue = [([120 - 20, 100, 40], [120 + 20, 255, 255])]
        self.HSVboundaries = green
        # self.HSVboundaries = [([160, 100, 40], [180, 255, 255]), #red
        #                      ([0, 100, 40], [30, 255, 255])]
        # ([25, 146, 190], [62, 174, 250]),
        # ([103, 86, 65], [145, 133, 128])]

    def detect(self, img):
        # blur, filter colors, dilate and erode to define edges, and find contours
        blur = cv2.GaussianBlur(img, (self.gauss_k, self.gauss_k), 0)
        mask = self.filterColors(blur)
        #mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=1)
        inf, cnts, hrch = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def filterColors(self, img):
        """
        Get an RGB image, transfor to HSV, and filter only specific colors
        defined by HSV boundaries
        Args:
            img: image to transfor to from RGB to HSV
        Returns:
            mask: A 2-dimension boolean np.array whose elements are true
                  if the image pixel value in the same position of element is
                  inside the boundaries specified
        """

        # store here the final mask
        mask = np.zeros(img.shape[:2], dtype=bool)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        for (lower, upper) in self.HSVboundaries:
            # create NumPy arrays from the HSVboundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            # find the colors within the specified HSVboundaries and apply
            # the mask
            mask = cv2.inRange(img, lower, upper) + mask

        return mask


    if __name__ == "__main__":
        rospy.subscriber("")
