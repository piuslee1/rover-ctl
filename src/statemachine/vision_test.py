import glob
import vision
import cv2

for imagePath in glob.glob("test_images/*.JPEG"):
    image = cv2.imread(imagePath)
    x, y, conf, r = vision.detect(image)
    vision.draw(image, x, y, r)

