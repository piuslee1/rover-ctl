import vision, math, rospy
from sensor_msgs.msg import Image

ZED_FOV_H = math.pi/2

class SearchState:
    def __init__(self, confidence_thres):
        self.confidence_thres = confidence_thres

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
        x, y = vision.detect(image)
        ori = vision.calculateOrientation(x, image.shape[1], ZED_FOV_H)
        self.foundCallback(ori)

    def foundCallback(self, orientation):
        print("Found object at "),
        print(orientation)
