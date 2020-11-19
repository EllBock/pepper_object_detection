import cv2

def stitch(images):
    stitcher = cv2.createStitcher()
    #stitcher = cv2.Stitcher.create()
    return stitcher.stitch(images)