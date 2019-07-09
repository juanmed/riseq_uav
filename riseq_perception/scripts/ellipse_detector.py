import cv2
import argparse
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.widgets import Slider


def build_arg_parser():
    """
    Build command line argument parser
    Return:
        Dictionary containing arguments and their values
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required = True, help = "Path to the image")
    ap = ap.parse_args()
    return ap   

class Ellipse():
    def __init__(self,a=1,b=1,cx=0,cy=0,theta=0):
        """
        Create an ellipse object
        Args:
            a --  major radius
            b -- minor radius
            cx -- x coordinate of ellipse center
            cy -- y coordinate of ellipse center
            theta -- angle by which the ellipse is rotated
        """
        self.a = a
        self.b = b
        self.cx = cx
        self.cy = cy
        self.theta = theta

    def get_ellipse_point(self,alpha):
        """
        Get x,y values corresponding to angle alpha
        Args:
            theta -- angle for which the x,y coordinated will be computed
        Returns:
            x,y values of the point corresponding to the angle alpha
        See: https://math.stackexchange.com/questions/2645689/what-is-the-parametric-equation-of-a-rotated-ellipse-given-the-angle-of-rotatio
        """
        x = self.a*np.cos(alpha)*np.cos(self.theta) - self.b*np.sin(alpha)*np.sin(self.theta) + self.cx
        y = self.a*np.cos(alpha)*np.sin(self.theta) + self.b*np.sin(alpha)*np.cos(self.theta) + self.cy
        return x,y

class Memory():

    def __init__(self, max_size = 800):

        self.max_size = max_size
        self.img = 0

        self.fig = plt.figure(figsize=(20,10))
        self.ax = self.fig.add_axes([0.01, 0.25,0.45, 0.45])
        self.ax2 = self.fig.add_axes([0.50, 0.25, 0.45, 0.45])

        #create sliders
        axcolor = 'lightgoldenrodyellow'
        axcannylt = self.fig.add_axes([0.1, 0.05, 0.8, 0.01], facecolor= axcolor)
        axcannyht =self.fig.add_axes([0.1, 0.07, 0.8, 0.01], facecolor= axcolor)
        axiter = self.fig.add_axes([0.1, 0.09, 0.8, 0.01], facecolor= axcolor)
        axcannyk = self.fig.add_axes([0.1, 0.11, 0.8, 0.01], facecolor= axcolor)
        #axminRadius = self.fig.add_axes([0.1, 0.13, 0.8, 0.01], facecolor= axcolor)
        #axmaxRadius = self.fig.add_axes([0.1, 0.15, 0.8, 0.01], facecolor= axcolor)

        self.canny_lt_slider = Slider(axcannylt, 'canny_lt', 50, 500.0, valinit=100)
        self.canny_ht_slider = Slider(axcannyht, 'canny_ht', 50, 500.0, valinit=200)
        self.iter_slider = Slider(axiter, 'iter', 0.0, 10, valinit=0, valstep=1)
        self.canny_k_slider = Slider(axcannyk, 'canny K', 0.0, 30, valinit = 3, valstep=2)
        #self.minRadius_slider = Slider(axminRadius, '*', 0.0,100,valinit=50)
        #self.maxRadius_slider = Slider(axmaxRadius, '*',0.0,100,valinit=70)

        self.canny_lt_slider.on_changed(self.update)
        self.canny_ht_slider.on_changed(self.update)
        self.iter_slider.on_changed(self.update)
        self.canny_k_slider.on_changed(self.update)
        #self.minRadius_slider.on_changed(self.update)
        #self.maxRadius_slider.on_changed(self.update)

        self.centroid_threshold = 10

        #Define corners of 3D Model's bounding cube
        self.corners3D = np.zeros((3,5))
        tube_length = 1 # m
        tube_radius = 0.2 # m
        self.corners3D[0] = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.corners3D[1] = np.array([0.0, tube_radius, 0.0, -tube_radius, 0.0])
        self.corners3D[2] = np.array([0.0, 0.0, tube_radius, 0.0, -tube_radius])
        #self.corners3D[3] = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
        self.corners3D = np.transpose(self.corners3D)
        #print(self.corners3D)

        # camera intrinsic matrix
        self.K = np.zeros((3,3), dtype='float64')
        self.K[0, 0], self.K[0, 2] = 320, 320
        self.K[1, 1], self.K[1, 2] = 320, 240
        self.K[2, 2] = 1.     

        self.distCoeffs = np.zeros((8, 1), dtype='float32')  

        self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1]]).reshape(-1,3) 

    def update(self, val):
        """
        Read slider values and update algorithms' parameters for ellipse 3D pose
        estimation. The argument is ignored.
        """
        lt = int(self.canny_lt_slider.val)
        ht = int(self.canny_ht_slider.val)
        k = int(self.canny_k_slider.val)
        iterations = int(self.iter_slider.val)


        gray, self.img = self.preprocess(self.img.copy())
        edges = cv2.Canny(self.img.copy(), lt, ht, k)
        dilate = cv2.dilate(edges.copy(), None, iterations = iterations)
        erode = cv2.erode(dilate.copy(), None, iterations = iterations)
        cnts = cv2.findContours(erode.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        img = self.img.copy()
        
        for i, c in enumerate(cnts[1]):
            if c.shape[0] > 5:

                e = cv2.fitEllipse(c)
                # threshold by size 
                ex = int(e[0][0])
                ey = int(e[0][1])
                width = e[1][0]
                height = e[1][1]

                if( (width > 20 ) and (height > 20) ):

                    empty = np.zeros_like(gray)
                    #print("{}. Center: {}".format(i, e[0]))
                    #print("    >> Is Convex? : {}".format(cv2.isContourConvex(c)))
                    edge_arcLength = cv2.arcLength(c,False)
                    edge = c.reshape(-1,2)
                    #print("    >> Edge Length: {}".format(edge_arcLength))
                    #print("    >> Edge Contour shape: {}, type: {}, reshape: {}".format(c.shape, type(c), c.reshape(-1,2).shape))
                    empty = cv2.ellipse(empty, e, (255,0,0), 2)
                    ellipse_edges = cv2.Canny(empty, lt, ht, k)
                    ellipse_cnts = cv2.findContours(ellipse_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    empty2 = np.zeros_like(gray)
                    empty2 = cv2.drawContours(empty2, ellipse_cnts[1], -1, (255,0,0), 1)
                    if (len(ellipse_cnts[1]) == 1):
                        ellipse_edge = ellipse_cnts[1][0]
                        ellipse_edge_arclength = cv2.arcLength(ellipse_edge, True)
                        ellipse_edge = ellipse_edge.reshape(-1,2)
                        #print("    ** Ellipse edges shape: {}, arcLength: {}".format(ellipse_edge.shape, ellipse_edge_arclength))
                    else:
                        continue
                        #ellipse_edge_arclength = cv2.arcLength(ellipse_cnts[1][0], True)
                        #print(" ***** ERROR  ERROR   FOUND MORE THAN 1 EDGE, SHOULD FOUND ONLY 1 EDGE ****")


                    coverage = edge_arcLength / ellipse_edge_arclength
                    if ( (coverage > 0.8) and (coverage < 1.1)):
                        pass
                        #cv2.drawContours(img, [c], -1, (255,0,0), 1)
                        #cv2.ellipse(img, e, (0,255,0), 1)
                        #cv2.putText(img, str(i), (int(e[0][0]),int(e[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1) 
                    else:
                        continue

                    # compute centroids
                    edge_moments = cv2.moments(c)
                    #ellipse_moments = cv2.moments(ellipse_edge)

                    edgex = int(edge_moments["m10"] / edge_moments["m00"])
                    edgey = int(edge_moments["m01"] / edge_moments["m00"])


                    edge_centroid = np.array([edgex, edgey])
                    ellipse_centroid = np.array([ex, ey])
                    centroids_distance =  np.linalg.norm(edge_centroid-ellipse_centroid)

                    if(centroids_distance < self.centroid_threshold):
                        print("   >> {}. Valid ellipse found, coverage: {}".format(i, coverage))
                        print("   >>     Edge length: {}, ellipse length: {}".format(edge.shape, ellipse_edge.shape))
                        print("   >>     Centroids distance: {}".format(centroids_distance))
                        cv2.drawContours(img, [c], -1, (255,0,0), 1)
                        cv2.ellipse(img, e, (0,255,0), 1)
                        cv2.putText(img, str(i), (int(e[0][0]),int(e[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1) 
                        cv2.circle(img, (edgex, edgey), 3, (255, 0, 0), -1)
                        cv2.circle(img, (ex, ey), 3, (0, 255, 0), -1)                    
                    else:
                        continue                        


                    # get 2d points correspondences
                    el = Ellipse(a=width/2, b=height/2, cx=ex, cy=ey, theta = e[2]*np.pi/180.0)
                    corners2D = np.zeros((5,2), dtype='float32')
                    corners2D[0][0] = ex
                    corners2D[0][1] = ey
                    for i ,angle in enumerate([0.0, np.pi/2, np.pi, 3*np.pi/2]):
                        x,y = el.get_ellipse_point(angle)
                        corners2D[i+1][0] = x
                        corners2D[i+1][1] = y
                        cv2.circle(img, (int(x), int(y)), 4, (0, 0, 255), -1) 

                    #print(corners2D)

                    R,t,rvecs = self.getPose(self.corners3D, corners2D)
                    print("Rotation: {}\ntranslation: {}".format(R,t))

                    imgpts, jac = cv2.projectPoints(self.axis, rvecs, t, self.K, self.distCoeffs)
                    img = self.draw(img,(ex,ey),imgpts)

                    #empty = cv2.dilate(empty, None, iterations = 1)
                    #empty = cv2.erode(empty, None, iterations = 1)

                    
                    #ellipse = np.transpose(np.nonzero(empty))
                    #ellipse = np.unique(ellipse, axis = 0).reshape((-1,1,2))
                    #print("    >> Ellipse shape: {}, arcLength: {}".format(ellipse.shape, cv2.arcLength(ellipse, True)))
                    #polyapprox = cv2.approxPolyDP(c, 0.001*cv2.arcLength(c,False), False )
                    #print("    >> PolyApprox shape: {}, arcLength: {}".format(polyapprox.shape, cv2.arcLength(polyapprox, False)))
                    #convHull = cv2.convexHull(c)
                    #print("    >> ConvexHull shape: {}, arcLength: {}".format(convHull.shape, cv2.arcLength(convHull, False)))
                    #cv2.drawContours(empty, [convHull], -1, (255,0,0), 1)

        self.ax.imshow(edges)#
        self.ax2.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        print(" >> Image updated")
        return img

    def preprocess(self, img):
        """
        Resize, blur and convert input image to grayscale
        Args:
            img (np.array): input image to preprocess with 3 channels
        Returns:
            gray (np.array): grayscale image, 1 channel
            img (np.array): resized and possibly blured img, 3 channels
        """
        scale = self.max_size / float(max(img.shape))
        img = cv2.resize(img, None, fx=scale, fy = scale)
        #proc = cv2.bilateralFilter(img,9,75,75) 
        #proc = cv2.GaussianBlur(img.copy(), (5,5),0) 
        #proc = cv2.medianBlur(img.copy(), 5)   
        proc = img

        if(proc.shape[2] == 3):
            gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
        else:
            gray = proc

        return gray, img

    def getPose(self, points_3D, points_2D):
        """
        Estimate ellipse 3D pose based on 2D points and 3D points correspondence
        Args:
            points_3D (np.array): 3D points of the object 
            points_2D (np.array): 2D points in the image plane
        Returns:
            R (np.array): 3x3 rotation matrix following the opencv camera convention
            t (np.array): 3x1 translation vector
        """
        assert points_2D.shape[0] == points_2D.shape[0], 'points 3D and points 2D must have same number of vertices'

        points_2D = np.ascontiguousarray(points_2D[:,:2]).reshape((-1,1,2))

        _, R_exp, t = cv2.solvePnP(points_3D, points_2D, self.K, self.distCoeffs)
        R, _ = cv2.Rodrigues(R_exp)
        return R, t, R_exp

    def draw(self, img, corner, imgpts):
        """
        Draw over the image a coordinate frame with origin in corner 
        and axis extending to the points in imgpts.
        Args:
            img (np.array): image over which to draw the coordinate frame
            corner (tuple): tuple of coordinates of the center, (x,y)
            imgpts (np.array): coordinates of the other end of each of the three
                               axis

        Taken from: https://docs.opencv.org/master/d7/d53/tutorial_py_pose.html
        """
        #corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (255,0,0), 2)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 2)
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0,0,255), 2)
        return img


# Canny parameters
lt = 50
ht = 250
k = 3
iterations = 0

#Define corners of 3D Model's bounding cube
corners3D = np.zeros((3,5))
tube_length = 1 # m
tube_radius = 0.2 # m
corners3D[0] = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
corners3D[1] = np.array([0.0, tube_radius, 0.0, -tube_radius, 0.0])
corners3D[2] = np.array([0.0, 0.0, tube_radius, 0.0, -tube_radius])
#self.corners3D[3] = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
corners3D = np.transpose(corners3D)

mem = Memory()


def detect_ellipse(img, max_size):


        gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
        img  = cv2.GaussianBlur(img, (5,5),0) 
        edges = cv2.Canny(img, lt, ht, k)
        dilate = cv2.dilate(edges, None, iterations = iterations)
        erode = cv2.erode(dilate, None, iterations = iterations)
        cnts = cv2.findContours(erode, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        
        detected_ellipses = []
        for i, c in enumerate(cnts[1]):
            if c.shape[0] > 5:

                e = cv2.fitEllipse(c)
                # threshold by size 
                ex = int(e[0][0])
                ey = int(e[0][1])
                width = e[1][0]
                height = e[1][1]

                # filter ellipse by size
                if((width > 20 ) and (height > 20)):           

                    edge_arcLength = cv2.arcLength(c,False)
                    edge = c.reshape(-1,2)


                    empty = np.zeros_like(gray)
                    empty = cv2.ellipse(empty, e, (255,0,0), 2)
                    ellipse_edges = cv2.Canny(empty, lt, ht, k)
                    ellipse_cnts = cv2.findContours(ellipse_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if (len(ellipse_cnts[1]) == 1):
                        ellipse_edge = ellipse_cnts[1][0]
                        ellipse_edge_arclength = cv2.arcLength(ellipse_edge, True)
                        ellipse_edge = ellipse_edge.reshape(-1,2)
                        #print("    ** Ellipse edges shape: {}, arcLength: {}".format(ellipse_edge.shape, ellipse_edge_arclength))
                    else:
                        # only 1 countours should be found, if not, continue
                        continue 

                    # filter by arclength
                    try:
                        coverage = edge_arcLength / ellipse_edge_arclength
                    except ZeroDivisionError:
                        continue

                    if ( (coverage > 0.8) and (coverage < 1.1)):
                        pass
                    else:
                        continue         

                    # compute centroids
                    edge_moments = cv2.moments(c)
                    try:
                        edgex = int(edge_moments["m10"] / edge_moments["m00"])
                        edgey = int(edge_moments["m01"] / edge_moments["m00"])
                    except ZeroDivisionError:
                        continue

                    edge_centroid = np.array([edgex, edgey])
                    ellipse_centroid = np.array([ex, ey])
                    centroids_distance =  np.linalg.norm(edge_centroid-ellipse_centroid)

                    # filter by centroid distance
                    centroid_threshold = 10
                    if(centroids_distance < centroid_threshold):
                        pass                  
                    else:
                        continue                        


                    # get 2d points correspondences
                    el = Ellipse(a=width/2, b=height/2, cx=ex, cy=ey, theta = e[2]*np.pi/180.0)
                    corners2D = np.zeros((5,2), dtype='float32')
                    corners2D[0][0] = ex
                    corners2D[0][1] = ey
                    for i ,angle in enumerate([0.0, np.pi/2, np.pi, 3*np.pi/2]):
                        x,y = el.get_ellipse_point(angle)
                        corners2D[i+1][0] = x
                        corners2D[i+1][1] = y
                        cv2.circle(img, (int(x), int(y)), 4, (0, 0, 255), -1) 

                    
                    # get Rotation and translation
                    R,t,rvecs = mem.getPose(corners3D, corners2D)
                    #print("Rotation: {}\ntranslation: {}".format(R,t))          

                    #imgpts, jac = cv2.projectPoints(self.axis, rvecs, t, self.K, self.distCoeffs)
                    #img = self.draw(img,(ex,ey),imgpts)

                    detected_ellipses.append((R,t,e))

        return detected_ellipses


def main(args):
    """
    Load image, set parameter sliders, show result
    """
    image = cv2.imread(args.image)
    mem = Memory()
    mem.img = image
    mem.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)

