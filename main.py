from os import write
from matplotlib import pyplot as plt
from calibrate import calibrate
from camutils import reconstruct, Camera, calibratePose, makerotation, generateMesh
import pickle
import numpy as np
from cv2 import findChessboardCorners
from meshutils import writeply

def performCameraCalibration():
    '''
    #perform camera calibration
    calibimgfiles = './calib_jpg_u/*.jpg'
    resultfile = 'calibration.pickle'
    calibrate(calibimgfiles, resultfile)
    '''
    #load intrinsic camera parameters
    print("loading intrinsic camera parameters")
    fid = open('./calibration.pickle','rb')
    calib = pickle.load(fid)
    fid.close()
    f = (calib['fx']+calib['fy'])/2
    c = np.array([[calib['cx']],[calib['cy']]])
    t = np.array([[0,0,0]]).T
    R = makerotation(0,0,0)

    #create cameras with intrinsic params
    camL = Camera(f,c,R,t)
    camR = Camera(f,c,R,t)

    #board images
    imgL = plt.imread('./calib_jpg_u/frame_C0_01.jpg')
    imgR = plt.imread('./calib_jpg_u/frame_C1_01.jpg')

    #find board corners
    _, boardL = findChessboardCorners(imgL, (8, 6), None)
    _, boardR = findChessboardCorners(imgR, (8, 6), None)
    #get pts2L,pts2R
    pts2L = boardL.squeeze().T
    pts2R = boardR.squeeze().T
    
    #board in 3d
    pts3 = np.zeros((3,6*8))
    yy,xx = np.meshgrid(np.arange(8),np.arange(6)) 
    pts3[0,:] = 2.8*xx.reshape(1,-1)
    pts3[1,:] = 2.8*yy.reshape(1,-1)
    params = [0,0,0,0,0,-2]
    #return cameras with extrinsic params
    return calibratePose(pts3, pts2L, camL, params), calibratePose(pts3, pts2R, camR, params)

def generatePoints(camL, camR, imprefix):
    imprefixL = imprefix+"/frame_C0_"
    imprefixR = imprefix+"/frame_C1_"
    imprefixLC = imprefix+"/color_C0_"
    imprefixRC = imprefix+"/color_C1_"
    threshold = 0.02
    thresholdC = 0.01
    pts2L,pts2R,pts3, colors = reconstruct(imprefixL,imprefixR,threshold,camL,camR, imprefixLC, imprefixRC, thresholdC)
    data = {
        "pts2L": pts2L,
        "pts2R": pts2R,
        "pts3": pts3,
        "colors": colors
    }
    with open("./points.pickle", "wb") as f:
        pickle.dump(data, f)    
    print("done generating points")
    return data

    
def visualizePoints(pts3, camL, camR, outFileName):
    lookL = np.hstack((camL.t,camL.t+camL.R @ np.array([[0,0,100]]).T))
    lookR = np.hstack((camR.t,camR.t+camR.R @ np.array([[0,0,100]]).T))
    #3d
    fig = plt.figure(figsize=(15,10))
    ax = fig.add_subplot(221, projection='3d')
    ax.plot(pts3[0,:],pts3[1,:],pts3[2,:],'.')
    ax.plot(camL.t[0],camL.t[1],camL.t[2],'bo')
    ax.plot(camR.t[0],camR.t[1],camR.t[2],'ro')
    ax.plot(lookL[0,:],lookL[1,:],lookL[2,:],'b')
    ax.plot(lookR[0,:],lookR[1,:],lookR[2,:],'r')
    plt.title('scene 3D view')
    plt.show()
    plt.savefig(outFileName)

def loadPoints(filename):
    data = {}
    with open(filename, "rb") as f:
        data = pickle.load(f)
    return data


def generatePlyFiles(directory):
    camL, camR = performCameraCalibration()
    for i in range(7):
        if i == 5:
            continue
        print(f"Scanning directory {i}")
        folder = f"grab_{i}_u"
        points = generatePoints(camL, camR, f"./{directory}/{folder}")
        pts3, triangles, colors = generateMesh(pts3=points["pts3"],
                        pts2L = points["pts2L"],
                        pts2R = points["pts2R"],
                        colors = points["colors"])
        writeply(pts3, colors, triangles, f"./{directory}_output/scan{i}.ply")

def main():
    generatePlyFiles("couple")


if __name__ == '__main__':
    main()