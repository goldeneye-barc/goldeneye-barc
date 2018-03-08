import cv2
import numpy as np

def bounding_box(img):
    # find contours
    ret = img.astype('uint8')
    (_, contours, _) = cv2.findContours(ret, cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE)

    # print table of contours and sizes
    max_contour = max(contours, key=lambda c: len(c)) if len(contours) > 0 else None
    if max_contour is None: return ret, [(0,0), (0,0)]
    x, y, w, h = cv2.boundingRect(max_contour)
    # draw a rectangle to visualize the bounding rect
    cv2.rectangle(ret, (x, y), (x+w, y+h), 255, 2)
    # get the min area rect
    rect = cv2.minAreaRect(max_contour)
    box = cv2.boxPoints(rect)
    # convert all coordinates floating point values to int
    box = np.int0(box)
    # draw a red 'nghien' rectangle
    cv2.drawContours(ret, [box], 0, 255)
    return ret, [(x, y), (x+w, y+h)]

def frame_to_bounding_box(img, IMAGE_MODE='rgb'):
    size = img.shape
    img = img.reshape((size[0], size[1], 3))
    if IMAGE_MODE=='bgr':
        b, g, r = cv2.split(img)
    else:
        r, g, b = cv2.split(img)
    red = np.zeros((size[0], size[1], 3))
    red[:,:,2] = r / 255
    red_min, green_max, blue_max = 80, 50, 50
    bw = np.logical_and(np.logical_and(r > red_min, g < green_max), b < blue_max)
    bw_b, coords = bounding_box(bw)
    x1, y1 = coords[0]
    x2, y2 = coords[1]
    fin = img
    cv2.rectangle(fin, (x1, y1), (x2, y2), 255, 2)
    return fin, [x1, y1, x2, y2]

if __name__ == "__main__":
    img = cv2.imread('test_img.png', cv2.IMREAD_COLOR)
    fin = frame_to_bounding_box(img, 'bgr')
    cv2.imshow('final', fin)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

