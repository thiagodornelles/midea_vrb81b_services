import numpy as np
from numpy import linalg as la
import cv2 as cv
import visual_odometry as vo

def draw_trajectory(position, trajectory_image):
    # TRAJECTORY DRAWING
    x, y = position[0], position[1]
    dims = trajectory_image.shape
    draw_x, draw_y = int(x) + int(dims[1]/2), int(y) + int(dims[0]/2)
    cv.circle(
        trajectory_image,
        (draw_x, draw_y),
        2,
        (0, 255, 0),
        -1,
    )
    cv.rectangle(trajectory_image, (0, 0), (1200, 60), (0, 0, 0), -1)
    text = "Coordinates: x=%2fm y=%2fm" % (x, y)
    cv.putText(trajectory_image, text, (20, 40), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 1, 8)
    cv.imshow("Trajectory", trajectory_image)
    # TRAJECTORY DRAWING


def main():
    f = 2714.3
    f = np.sqrt(f)
    f = np.sqrt(f)
    f = np.sqrt(f)

    # size = [320, 240]
    size = [160, 120]
    cam_params = vo.PinholeCamera(size[0], size[1], f, f, size[0]/2, size[1]/2)
    vis_odo = vo.VisualOdometry(cam_params, 10)
    # capture = vo.VideoFrameInput("output2.mp4")    
    capture = vo.ImageFrameInput("/Users/thiago/Desktop/curve")
    trajectory_image = np.zeros((1200, 1200, 3), dtype=np.uint8)

    i = 0
    vector = np.array([0, 0, 1]).astype(float)
    transf_acum = np.identity(3).astype(float)

    while capture.is_capture_opened():
        i += 1
        frame1, frame2 = capture.getFrames()
        if frame1 is not None and frame2 is not None:
            frame1 = cv.resize(frame1, (size[0], size[1]))
            frame2 = cv.resize(frame2, (size[0], size[1]))            
        else:
            break
        if i > 0:
            frame1_gray = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
            frame2_gray = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)

            cv.imshow("Frame 1", frame1_gray)
            cv.imshow("Frame 2", frame1_gray)

            transf, warp_matrix = vis_odo.findPoseTransform(frame2_gray, frame1_gray)
            _, Rs, Ts, Ns = cv.decomposeHomographyMat(
                        transf, cam_params.get_intrinsic_matrix()
                    )        
            # Select the best matrix
            transf_last = vis_odo.getFeasibleTranformation(Ns, Rs, Ts)
            
            cv.imshow("diff", cv.absdiff(frame2_gray, frame1_gray))

            position = vis_odo.transf_accum.dot(np.array([0, 0, 1], dtype=np.float32))
            draw_trajectory(position, trajectory_image)

        if cv.waitKey(1000) & 0xFF == ord("q"):
            break
    
    print("#### FIM ####")    
    if cv.waitKey(1) & 0xFF == ord("q"):
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()
