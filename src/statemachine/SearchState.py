import vision, math, rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

ZED_FOV_H = math.pi/2

class SearchState:
    def __init__(self, confidence_thres):
        self.confidence_thres = confidence_thres
        self.notfound = 0
        self.bridge = CvBridge()

    def attach(self):
        self.im_sub = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.imageCallback)

    def detach(self):
        self.im_sub.unregister()
        
    def imageCallback(self, data):
        try:
          image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
        cv2.imshow("IM", image)
        cv2.waitKey(1)
        x, y, conf, r = vision.detect(image)
        if conf > self.confidence_thres:
            ori, angle = vision.calculateOrientation(x, image.shape[1], ZED_FOV_H)
            dist = vision.calcDist(2*r, image.shape[1], ZED_FOV_H)
            self.notfound = 0
            # Angle heading
            self.foundCallback(ori, angle, dist)
        else:
            self.notfound += 1
            self.notfoundCallback(self.notfound)

    def notfoundCallback(self, count):
        return

    def foundCallback(self, orientation, angle, dist):
        print("Found object at "),
        print(orientation)
