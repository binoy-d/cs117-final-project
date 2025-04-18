import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

def makerotation(rx,ry,rz):
    """
    Generate a rotation matrix    

    Parameters
    ----------
    rx,ry,rz : floats
        Amount to rotate around x, y and z axes in degrees

    Returns
    -------
    R : 2D numpy.array (dtype=float)
        Rotation matrix of shape (3,3)
    """
    rx = np.pi*rx/180.0
    ry = np.pi*ry/180.0
    rz = np.pi*rz/180.0

    Rx = np.array([[1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]])
    Ry = np.array([[np.cos(ry),0,-np.sin(ry)],[0,1,0],[np.sin(ry),0,np.cos(ry)]])
    Rz = np.array([[np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]])
    R = (Rz @ Ry @ Rx)
    
    return R 

class Camera:
    """
    A simple data structure describing camera parameters 
    
    The parameters describing the camera
    cam.f : float   --- camera focal length (in units of pixels)
    cam.c : 2x1 vector  --- offset of principle point
    cam.R : 3x3 matrix --- camera rotation
    cam.t : 3x1 vector --- camera translation 

    
    """    
    def __init__(self,f,c,R,t):
        self.f = f
        self.c = c
        self.R = R
        self.t = t

    def __str__(self):
        return f'Camera : \n f={self.f} \n c={self.c.T} \n R={self.R} \n t = {self.t.T}'
    
    def project(self,pts3):
        """
        Project the given 3D points in world coordinates into the specified camera    

        Parameters
        ----------
        pts3 : 2D numpy.array (dtype=float)
            Coordinates of N points stored in a array of shape (3,N)

        Returns
        -------
        pts2 : 2D numpy.array (dtype=float)
            Image coordinates of N points stored in an array of shape (2,N)

        """
        assert(pts3.shape[0]==3)

        # get point location relative to camera
        pcam = self.R.transpose() @ (pts3 - self.t)
         
        # project
        p = self.f * (pcam / pcam[2,:])
        
        # offset principal point
        pts2 = p[0:2,:] + self.c
        
        assert(pts2.shape[1]==pts3.shape[1])
        assert(pts2.shape[0]==2)
    
        return pts2
 
    def update_extrinsics(self,params):
        """
        Given a vector of extrinsic parameters, update the camera
        to use the provided parameters.
  
        Parameters
        ----------
        params : 1D numpy.array (dtype=float)
            Camera parameters we are optimizing over stored in a vector
            params[0:2] are the rotation angles, params[2:5] are the translation

        """
        self.R = makerotation(params[0],params[1],params[2])
        self.t = np.array([[params[3]],[params[4]],[params[5]]])


def triangulate(pts2L,camL,pts2R,camR):
    """
    Triangulate the set of points seen at location pts2L / pts2R in the
    corresponding pair of cameras. Return the 3D coordinates relative
    to the global coordinate system


    Parameters
    ----------
    pts2L : 2D numpy.array (dtype=float)
        Coordinates of N points stored in a array of shape (2,N) seen from camL camera

    pts2R : 2D numpy.array (dtype=float)
        Coordinates of N points stored in a array of shape (2,N) seen from camR camera

    camL : Camera
        The first "left" camera view

    camR : Camera
        The second "right" camera view

    Returns
    -------
    pts3 : 2D numpy.array (dtype=float)
        (3,N) array containing 3D coordinates of the points in global coordinates

    """

    npts = pts2L.shape[1]

    qL = (pts2L - camL.c) / camL.f
    qL = np.vstack((qL,np.ones((1,npts))))

    qR = (pts2R - camR.c) / camR.f
    qR = np.vstack((qR,np.ones((1,npts))))
    
    R = camL.R.T @ camR.R
    t = camL.R.T @ (camR.t-camL.t)

    xL = np.zeros((3,npts))
    xR = np.zeros((3,npts))

    for i in range(npts):
        A = np.vstack((qL[:,i],-R @ qR[:,i])).T
        z,_,_,_ = np.linalg.lstsq(A,t,rcond=None)
        xL[:,i] = z[0]*qL[:,i]
        xR[:,i] = z[1]*qR[:,i]
 
    pts3L = camL.R @ xL + camL.t
    pts3R = camR.R @ xR + camR.t
    pts3 = 0.5*(pts3L+pts3R)

    return pts3


def residuals(pts3,pts2,cam,params):
    """
    Compute the difference between the projection of 3D points by the camera
    with the given parameters and the observed 2D locations

    Parameters
    ----------
    pts3 : 2D numpy.array (dtype=float)
        Coordinates of N points stored in a array of shape (3,N)

    pts2 : 2D numpy.array (dtype=float)
        Coordinates of N points stored in a array of shape (2,N)

    params : 1D numpy.array (dtype=float)
        Camera parameters we are optimizing over stored in a vector

    Returns
    -------
    residual : 1D numpy.array (dtype=float)
        Vector of residual 2D projection errors of size 2*N
        
    """

    cam.update_extrinsics(params)
    residual = pts2 - cam.project(pts3)
    
    return residual.flatten()

def calibratePose(pts3,pts2,cam_init,params_init):
    """
    Calibrate the provided camera by updating R,t so that pts3 projects
    as close as possible to pts2

    Parameters
    ----------
    pts3 : 2D numpy.array (dtype=float)
        Coordinates of N points stored in a array of shape (3,N)

    pts2 : 2D numpy.array (dtype=float)
        Coordinates of N points stored in a array of shape (2,N)

    cam : Camera
        Initial estimate of camera

    Returns
    -------
    cam_opt : Camera
        Refined estimate of camera with updated R,t parameters
        
    """

    # define our error function
    efun = lambda params: residuals(pts3,pts2,cam_init,params)        
    popt,_ = scipy.optimize.leastsq(efun,params_init)
    cam_init.update_extrinsics(popt)

    return cam_init



def decode(imprefix,start,threshold, imprefixC, thresholdC):
    """
    Decode 10bit gray code pattern with the given difference
    threshold.  We assume the images come in consective pairs
    with filenames of the form <prefix><start>.png - <prefix><start+20>.png
    (e.g. a start offset of 20 would yield image20.png, image01.png... image39.png)

    Parameters
    ----------
    imprefix : str
      prefix of where to find the images (assumed to be .png)

    start : int
      image offset.  

    threshold : float

    Returns
    -------
    code : 2D numpy.array (dtype=float)
        
    mask : 2D numpy.array (dtype=float)
    
    """
    nbits = 10
    
    imgs = list()
    imgs_inv = list()
    print('loading',end='')
    for i in range(start,start+2*nbits,2):
        fname0 = '%s%2.2d.png' % (imprefix,i)
        fname1 = '%s%2.2d.png' % (imprefix,i+1)
        print('(',i,i+1,')',end='')
        img = plt.imread(fname0)
        img_inv = plt.imread(fname1)
        if (img.dtype == np.uint8):
            img = img.astype(float) / 256
            img_inv = img_inv.astype(float) / 256
        if (len(img.shape)>2):
            img = np.mean(img,axis=2)
            img_inv = np.mean(img_inv,axis=2)
        imgs.append(img)
        imgs_inv.append(img_inv)
        
    (h,w) = imgs[0].shape
    print('\n')
    
    gcd = np.zeros((h,w,nbits))
    mask = np.ones((h,w))
    for i in range(nbits):
        gcd[:,:,i] = imgs[i]>imgs_inv[i]
        mask = mask * (np.abs(imgs[i]-imgs_inv[i])>threshold)
        
    bcd = np.zeros((h,w,nbits))
    bcd[:,:,0] = gcd[:,:,0]
    for i in range(1,nbits):
        bcd[:,:,i] = np.logical_xor(bcd[:,:,i-1],gcd[:,:,i])
        
    code = np.zeros((h,w))
    for i in range(nbits):
        code = code + np.power(2,(nbits-i-1))*bcd[:,:,i]
        
    #subtract foreground from background to get pixels in object, using thresholdC
    imgBackground = plt.imread(imprefixC+"00.png")
    imgWithObject = plt.imread(imprefixC+"01.png")
    colorMask = np.where(np.sum(np.square(imgBackground-imgWithObject), axis=-1)>thresholdC, 1, 0)
    
    return code,mask,colorMask



def reconstruct(imprefixL,imprefixR,threshold,camL,camR, imprefixLC, imprefixRC, thresholdC):
    """
    Simple reconstruction based on triangulating matched pairs of points
    between to view which have been encoded with a 20bit gray code.

    Parameters
    ----------
    imprefix : str
      prefix for where the images are stored

    threshold : float
      decodability threshold

    camL,camR : Camera
      camera parameters

    Returns
    -------
    pts2L,pts2R : 2D numpy.array (dtype=float)

    pts3 : 2D numpy.array (dtype=float)

    colors: 2D numpy.array(dtype=float) shape = 3*N

    """
    #CR, CL = code R and code L(binary light code)
    CLh,maskLh, maskLC = decode(imprefixL,0,threshold, imprefixLC, thresholdC)
    CLv,maskLv, _ = decode(imprefixL,20,threshold,imprefixLC, thresholdC) # we dont use this color mask
    CRh,maskRh, maskRC = decode(imprefixR,0,threshold, imprefixRC, thresholdC)
    CRv,maskRv, _ = decode(imprefixR,20,threshold,  imprefixRC, thresholdC) # we dont use this color mask

    CL = CLh + 1024*CLv
    maskL = maskLh*maskLv*maskLC
    CR = CRh + 1024*CRv
    maskR = maskRh*maskRv*maskRC

    h = CR.shape[0]
    w = CR.shape[1]

    subR = np.nonzero(maskR.flatten())
    subL = np.nonzero(maskL.flatten())

    CRgood = CR.flatten()[subR]
    CLgood = CL.flatten()[subL]

    _,submatchR,submatchL = np.intersect1d(CRgood,CLgood,return_indices=True)

    matchR = subR[0][submatchR]
    matchL = subL[0][submatchL]

    xx,yy = np.meshgrid(range(w),range(h))
    xx = np.reshape(xx,(-1,1))
    yy = np.reshape(yy,(-1,1))

    pts2R = np.concatenate((xx[matchR].T,yy[matchR].T),axis=0)
    pts2L = np.concatenate((xx[matchL].T,yy[matchL].T),axis=0)

    #record colors for each pixel
    imgL = plt.imread(imprefixLC+"01.png")
    imgR = plt.imread(imprefixRC+"01.png")
    
    colorsL = imgL[pts2L[1], pts2L[0]].T
    colorsR = imgR[pts2R[1], pts2R[0]].T

    colors = (colorsR+colorsL)/2
    pts3 = triangulate(pts2L,camL,pts2R,camR)

    return pts2L,pts2R,pts3, colors

def generateMesh(   boxlimits =  np.array([-40,50,-40,20,-40,20]), \
                    trithresh = 2, \
                    pts3 = np.array([]), \
                    pts2L = np.array([]), \
                    pts2R = np.array([]), \
                    colors = np.array([])
                    ):
    
    #bounding box pruning
    
    def is_in_limits(point):
        return  point[0]>boxlimits[0] and point[0]<boxlimits[1] and point[1]>boxlimits[2] and \
                point[1]<boxlimits[3] and point[2]>boxlimits[4] and point[2]<boxlimits[5]

    keep = np.array([i for i in range(pts3.shape[1]) if is_in_limits(pts3[:, i])])

    pts3 = pts3[:, keep]
    pts2L = pts2L[:, keep]
    pts2R = pts2R[:,  keep]
    colors = colors[:, keep]

    
    #trianglate 2d points to get surface mesh
    triL=Delaunay(pts2L.T)
    simp = triL.simplices

    #TODO: mesh smoothing
    def neighbors(i, triangle):
        return triangle.vertex_neighbor_vertices[1][triangle.vertex_neighbor_vertices[0][i]:triangle.vertex_neighbor_vertices[0][i+1]]
    
    def smooth(level):
        for j in range(level):
            print(f"applying smoothing level {j}")
            for i in range(pts3.shape[1]):
                pts3[:,i] = np.mean(pts3[:, neighbors(i, triL)], axis=1)
        return pts3

    pts3 = smooth(4)

    #triangle pruning
    def distance(a, b):
        return np.sqrt(np.sum(np.power(pts3[:,simp[:,a]]-pts3[:,simp[:,b]],2),axis=0))

    keep = (distance(0, 1)<trithresh)&(distance(0, 2)<trithresh)&(distance(1,2)<trithresh)
    simp = simp[keep,:]

    # remove any points which are not referenced in any triangle
    keep=np.unique(simp)
    map = np.zeros(pts3.shape[1])
    pts3=pts3[:,keep]
    colors = colors[:,keep]

    #update tri
    map[keep] = np.arange(0,keep.shape[0])
    simp=map[simp]


    print(simp.shape)
    print(pts3.shape)
    print(colors.shape)
    
    return pts3, simp, colors


