from calibrate import calibrate
from camutils import reconstruct, Camera
import pickle

def main():

    #perform camera calibration
    calibimgfiles = './calib_jpg_u/*.jpg'
    resultfile = 'calibration.pickle'
    calibrate(calibimgfiles, resultfile)
    

    #decode and reconstruct pts3 from scans
    imprefixL = "./couple/grab_0_u/frame_C0_"
    imprefixR = "./couple/grab_0_u/frame_C1_"
    imprefixLC = "./couple/grab_0_u/color_C0_"
    imprefixRC = "./couple/grab_0_u/color_C1_"

    threshold = 0.02
    thresholdC = 0.01

    #load camera parameters
    print("loading camera parameters")
    fid = open('./calibration.pickle','rb')
    calib = pickle.load(fid)

    camL = Camera()
    camR = Camera()
    
    fid.close

    pts2L,pts2R,pts3 = reconstruct(imprefixL,imprefixR,threshold,camR,camL, imprefixLC, imprefixRC, thresholdC)
    print(pts3)


if __name__ == '__main__':
    main()