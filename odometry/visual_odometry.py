import numpy as np
from numpy import linalg as la
import cv2 as cv

class PinholeCamera:
    def __init__(
        self, width, height, fx, fy, cx, cy, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0
    ):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.distortion = abs(k1) > 0.0000001
        self.d = [k1, k2, p1, p2, k3]

    def get_intrinsic_matrix(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])

class ImageFrameInput:
    def __init__(self, images_path, index_start = 0):
        self.__cap = None
        self.__last_frame = None
        self.__has_next_frame = True
        self.__index_frame = index_start
        self.__images_path = images_path

    def release_cap(self):
        return True

    def is_capture_opened(self):
        return True

    def getFrames(self):        
        if self.__has_next_frame:            
            # If it is "None" means that it is the first captured frame
            image_1_path = str(self.__index_frame).zfill(12) + ".png"
            image_2_path = str(self.__index_frame + 1).zfill(12) + ".png"
            
            image_1_path = self.__images_path + "/" + image_1_path
            image_2_path  = self.__images_path + "/" + image_2_path

            if self.__last_frame is None:                
                frame1 = cv.imread(image_1_path)
                if self.is_capture_opened():
                    frame2 = cv.imread(image_2_path)
                    self.__last_frame = frame2.copy()
                else:
                    return None
            else:
                frame1 = self.__last_frame.copy()
                frame2 = cv.imread(image_2_path)
                if frame2 is not None:
                    self.__last_frame = frame2.copy()
            #Increase counter to next the frame        
            self.__index_frame += 1
        else:
            return None

        return frame1, frame2

class VideoFrameInput:
    def __init__(self, video_file):
        self.__cap = cv.VideoCapture(video_file)
        self.__last_frame = None

    def release_cap(self):
        self.__cap.release()

    def is_capture_opened(self):
        return self.__cap.isOpened()

    def getFrames(self):
        if self.__cap.isOpened():
            # If it is "None" means that it is the first captured frame
            if self.__last_frame is None:
                ret, frame1 = self.__cap.read()
                if self.__cap.isOpened():
                    ret, frame2 = self.__cap.read()
                    self.__last_frame = frame2.copy()
                else:
                    return None
            else:
                if self.__cap.isOpened():
                    frame1 = self.__last_frame.copy()
                    has_more_frames, frame2 = self.__cap.read()
                    if has_more_frames:
                        self.__last_frame = frame2.copy()
        else:
            return None

        return frame1, frame2

class VisualOdometry:

    def __init__(self, pinhole_camera, scale = 1):
        self.cam_params = pinhole_camera        
        self.scale = scale
        self.transf_last = np.identity(3, dtype=np.float32)
        self.transf_accum = np.identity(3, dtype=np.float32)
        self.transf_trajectory = []

    def normalized(self, a, axis = -1, order = 2):
        l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
        l2[l2==0] = 1
        return a / np.expand_dims(l2, axis)

    def findPoseTransform(self, frame1_gray, frame2_gray):

        #FIRST PASS
        warp_matrix = np.eye(2, 3, dtype=np.float32)
        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 1e-5)
        # Get homography
                
        (cc, warp_matrix) = cv.findTransformECC(
            frame2_gray, frame1_gray, warp_matrix, cv.MOTION_EUCLIDEAN, criteria
        )
        
        size = frame1_gray.shape
        im_aligned = cv.warpAffine(
                frame2_gray,
                warp_matrix,
                (size[1], size[0]),
                flags=cv.INTER_LINEAR + cv.WARP_INVERSE_MAP,
                borderMode=cv.BORDER_CONSTANT,
                borderValue=0
            )
        
        # diff_img = cv.absdiff(im_aligned, frame2_gray)
        # _, mask = cv.threshold(diff_img, 70, 255, cv.THRESH_BINARY_INV)
        # cv.imshow("diff_abs", mask)        

        # (cc, warp_matrix) = cv.findTransformECC(
        #     frame2_gray, frame1_gray, warp_matrix, cv.MOTION_EUCLIDEAN, criteria, inputMask=mask
        # )

        # Homogeneous coordinate version of homography
        last_row = np.array([0, 0, 1], dtype=np.float32)
        warp_matrix_ext = np.vstack((warp_matrix, last_row))
        # Getting 4 matrices decompositions
        _, Rs, Ts, Ns = cv.decomposeHomographyMat(
            warp_matrix_ext, self.cam_params.get_intrinsic_matrix()
        )
        # Select the best matrix
        self.transf_last = self.getFeasibleTranformation(Ns, Rs, Ts)
        self.transf_trajectory.append(self.transf_last)
        self.transf_accum = self.transf_accum.dot(la.inv(self.transf_last))
        return self.transf_last, warp_matrix

    def getFeasibleTranformation(self, normalPlanes, rotations, translations):
        transf = self.transf_last        
        cam_normal = np.array([0, 0, 1], dtype=np.float32)

        for i, normal in enumerate(normalPlanes):                
            # Validate if matrix is useful
            if np.transpose(normal).dot(cam_normal) == 1:                
                r = rotations[i]
                t = translations[i]
                
                angle = np.arccos((np.trace(r) - 1) / 2)
                deg = np.rad2deg(angle)

                if abs(deg) > 2:
                    transf = np.array(
                        [
                            [r[0][0], r[0][1], t[0] * self.scale],
                            [r[1][0], r[1][1], t[1] * self.scale],
                            [0, 0, 1],
                        ],
                        dtype=np.float32,
                    )
                else:                    
                    transf = np.array(                    
                        [
                            [1, 0, t[0] * self.scale],
                            [0, 1, t[1] * self.scale],
                            [0, 0, 1],
                        ],
                        dtype=np.float32,
                    )

        return transf
