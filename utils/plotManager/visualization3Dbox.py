"""
Ref) https://github.com/lzccccc/3d-bounding-box-estimation-for-autonomous-driving
"""

import numpy as np
import argparse
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
from matplotlib.gridspec import GridSpec
from PIL import Image
from abc import ABC, abstractmethod
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import copy
import cv2
# breakpoint()
from .kitti_util import roty

def getRectangle(center, xSize, ySize):
    # pdb.set_trace()
    xSize = xSize*2 + 0.2
    ySize = ySize*2 + 0.2
   

    ret = np.zeros((4,2))
    ret[0] = center + [-xSize/2, -ySize/2]
    ret[1] = center + [-xSize/2, ySize/2]
    ret[2] = center + [xSize/2, ySize/2]
    ret[3] = center + [xSize/2, -ySize/2]

    return ret

def getRhombus(center, radius = 0.3):
    # pdb.set_trace()
    ret = np.zeros((4,2))
    ret[0] = center + [-radius, 0]
    ret[1] = center + [0, radius*1.5]
    ret[2] = center + [radius, 0]
    ret[3] = center + [0, -radius*1.5]

    return ret


def addImage(smallImg, largeImg, offset=(0, 0)):
    # pdb.set_trace()
    x_offset = offset[0]
    y_offset = offset[1]

    x_end = x_offset + smallImg.shape[1]
    y_end = y_offset + smallImg.shape[0]
    
    largeImg[y_offset:y_end, x_offset:x_end] = smallImg

    return largeImg

class Writer():
    def __init__(self):
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(3, 3))
        self.imageSize = (480, 640)
    
    @abstractmethod
    def initFig(self):
        pass
    
    @abstractmethod
    def write(self):
        pass

    def getImageFromFig(self, r = 0.75):

        img = self.figToImage(self.fig)
        # breakpoint()
        img = cv2.resize(img, self.imageSize)
        self.resetFig()
        self.initFig()
        
        return img
    
    def resetFig(self):
        self.ax.clear()
    
    def figToImage(self, fig):
        canvas = FigureCanvas(fig)
        canvas.draw()
        image = np.array(canvas.renderer.buffer_rgba())
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

        return image_bgr 
    

class BEVWriter(Writer):
    def __init__(self, trkCfg):
        super().__init__()
        self.fig, self.ax = plt.subplots(figsize=(6, 3))
        self.initFig()
        self.scale = 1
        self.trkVisualizer = TrackVisualizer(trkCfg)
        self.trkCfg = trkCfg
        self.colormap = [(1,0,0), (0,1,0), (0,0,1), (1,1,0), (1,0,1), (0,1,1), (1,0,0.5), (1,0.5,0), (0.5,1,0),\
            (0,1,0.5), (0.5,0,1), (0,0.5,1), (1,1,0.5), (1,0.5,1), (0.5,1,1)]
    def __call__(self, radar1_object_list, radar1_track_list, radar2_object_list, radar2_track_list):
        self.write(radar1_object_list, c = (0,1,0))
        self.writeTrk(radar1_track_list, c = (1,0,1))

        return self.getImageFromFig()

    # def draw(self, predictions):
    #     for pred in predictions:
    #         # pdb.set_trace()
    #         pred_corners_2d = compute_birdviewbox(pred, self.shape, self.scale, True)
    #         patch = patches.Polygon(pred_corners_2d, facecolor = 'None', edgecolor =(1, 1, 0), linewidth = 1.5)

    #         self.ax.add_patch(patch)
        # self.ax.imshow(self.birdimage, origin='lower')
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        
        # self.fig.savefig('fig.jpg')
        # pdb.set_trace()
    def getAssocLine(self, trk):
        # breakpoint()
        # meas_x = trk.associatedObj.measVectorRRD[0] * trk.associatedObj.measVectorRRD[2]
        if trk.associatedObj is None:
            return None
        else:
            return [[trk.stateVectorXYZ[0], trk.associatedObj.stateVectorXYZ[0]], [trk.stateVectorXYZ[1], trk.associatedObj.stateVectorXYZ[1]]]
    def writeTrk(self, radar1_track_list, c = (0, 1, 0)):
        # breakpoint()
        for trk in radar1_track_list:
            if trk.tick < self.trkCfg.thresholdTick:
                continue
            # breakpoint()
            # patch = patches.Circle((trk.x, trk.y),radius =0.05, facecolor = c)
            patch = patches.Polygon(getRectangle(np.array([trk.stateVectorXYZ[0], trk.stateVectorXYZ[1]]), trk.xSize, trk.ySize),facecolor = 'None', edgecolor =c, linewidth = 1.0)
            self.ax.add_patch(patch)

            ma,R,P=self.trkVisualizer.getContour(trk)
            # if i == 0:
            #     breakpoint()
            if R[0] is not None:
                self.ax.plot(R[0], R[1],color='magenta', linestyle = ":", linewidth = 1.5)
                self.ax.plot(ma[0], ma[1],color='lime', linestyle = ":", linewidth = 1.5)
            self.ax.plot(P[0], P[1],color='yellow', linestyle = ":", linewidth = 1.5)
            
            
            line= self.getAssocLine(trk)
            if line is not None:
                self.ax.plot(line[0], line[1], color='white', linestyle = "-", linewidth = 1.5)

            


    def write(self, radar1_object_list, c = (0, 1, 0)):
        # pdb.set_trace()
        # breakpoint()
        for obj in radar1_object_list:
            patch = patches.Circle((obj.x, obj.y),radius =0.1, facecolor = self.colormap[obj.clusterId % len(self.colormap)])
            self.ax.add_patch(patch)

        # for trk in radar1_track_list:
            
        #     patch = patches.Polygon(getRhombus(np.array([trk.x, trk.y]), 1.0),facecolor = 'None', edgecolor =(1, 0, 1), linewidth = 1.0)
        #     self.ax.add_patch(patch)

    
    
    def initFig(self):

        self.imageSize = (960, 480)
        
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(0,5)
        self.ax.set_title('MOVON AMR RADAR')
        self.ax.set_xlabel('x(m)')
        self.ax.set_ylabel('y(m)')
        self.ax.grid(True, alpha=0.5, linestyle='--')

class DopplerWriter(Writer):
    def __init__(self):
        super().__init__()
        self.fig, self.ax = plt.subplots(figsize=(3, 3))
        self.initFig()
        self.scale = 1
        
    def __call__(self, radar1_object_list, radar2_object_list, line_x1, line_y1, line_x2, line_y2):
        # breakpoint()
        self.write(radar1_object_list, c = (1,0,1))
        self.write(radar2_object_list, c = (0,1,0))
        self.drawLine(line_x1, line_y1, c = 'magenta')
        self.drawLine(line_x2, line_y2, c = 'lime')
        return self.getImageFromFig()
    
    def write(self, radar1_object_list, c = (0,1,0)):
        
        for obj in radar1_object_list:
            patch = patches.Circle((np.arcsin(obj.sin_azim) * 180 / np.pi, obj.speed), radius = 0.6, facecolor = c)
            self.ax.add_patch(patch)
        
    
    def drawLine(self, line_x, line_y, c = 'white'):
        self.ax.plot(line_x, line_y, c = c)


    def initFig(self):
        self.imageSize = (240, 240)
        
        self.ax.set_xlim(-90,90)
        self.ax.set_ylim(-20,20)
        self.ax.set_title('Doppler-Theta')
        self.ax.set_xlabel('Theta(DEG)')
        self.ax.set_ylabel('Doppler(m/s)')
        self.ax.grid(True, alpha=0.5, linestyle='--')

    def test(self):
        img = self.getImageFromFig()
        cv2.imwrite('./test.jpg', img)

def compute_birdviewbox(line, shape, scale, isFromSMOKE = False):
    # pdb.set_trace()
    if not isFromSMOKE:
        npline = [np.float64(line[i]) for i in range(1, len(line))]
        h = npline[7] * scale
        w = npline[8] * scale
        l = npline[9] * scale
        x = npline[10] * scale
        y = npline[11] * scale
        z = npline[12] * scale
        rot_y = npline[13]
    else:
        h = line.h * scale
        w = line.w * scale
        l = line.l * scale
        x = line.t[0] * scale
        y = line.t[1] * scale
        z = line.t[2] * scale
        rot_y = line.ry

    R = np.array([[np.cos(rot_y), np.sin(rot_y)],
                  [-np.sin(rot_y), np.cos(rot_y)]])
    t = np.array([x, z]).reshape(1, 2).T

    REAR = np.array([[np.cos(np.pi), np.sin(np.pi)],
                  [-np.sin(np.pi), np.cos(np.pi)]])
    # x_corners = [0, l, l, 0]  # -l/2
    # z_corners = [w, w, 0, 0]  # -w/2


    # x_corners += -w / 2
    # z_corners += -l / 2

    
    x_corners = [l, l, 0, 0]  # -l/2
    z_corners = [w, 0, 0, w]  # -w/2

    x_corners += -l / 2
    z_corners += -w / 2

    # bounding box in object coordinate
    corners_2D = np.array([x_corners, z_corners])
    # rotate
    corners_2D = R.dot(corners_2D)
    # translation
    # pdb.set_trace()
    corners_2D = t + corners_2D
    # in camera coordinate
    # corners_2D[0] += int(shape/2)
    # corners_2D = (corners_2D).astype(np.int16)
    # pdb.set_trace()
    corners_2D = REAR.dot(corners_2D) # rotate coordinate for align with rear camera
    corners_2D = corners_2D.T
    # pdb.set_trace()
    return corners_2D #1 : front left corner, 2: front right corner, 3: back right corner, 4 : left back corner

def draw_birdeyes(ax2, line_gt, line_p, shape):
    # shape = 900
    scale = 15

    pred_corners_2d = compute_birdviewbox(line_p, shape, scale)
    gt_corners_2d = compute_birdviewbox(line_gt, shape, scale)

    codes = [Path.LINETO] * gt_corners_2d.shape[0]
    codes[0] = Path.MOVETO
    codes[-1] = Path.CLOSEPOLY
    pth = Path(gt_corners_2d, codes)
    p = patches.PathPatch(pth, fill=False, color='orange', label='ground truth')
    ax2.add_patch(p)

    codes = [Path.LINETO] * pred_corners_2d.shape[0]
    codes[0] = Path.MOVETO
    codes[-1] = Path.CLOSEPOLY
    pth = Path(pred_corners_2d, codes)
    p = patches.PathPatch(pth, fill=False, color='green', label='prediction')
    ax2.add_patch(p)

def compute_3Dbox(P2, line):
    obj = detectionInfo(line)
    # Draw 2D Bounding Box
    xmin = int(obj.xmin)
    xmax = int(obj.xmax)
    ymin = int(obj.ymin)
    ymax = int(obj.ymax)
    # width = xmax - xmin
    # height = ymax - ymin
    # box_2d = patches.Rectangle((xmin, ymin), width, height, fill=False, color='red', linewidth='3')
    # ax.add_patch(box_2d)

    # Draw 3D Bounding Box

    R = np.array([[np.cos(obj.rot_global), 0, np.sin(obj.rot_global)],
                  [0, 1, 0],
                  [-np.sin(obj.rot_global), 0, np.cos(obj.rot_global)]])

    x_corners = [0, obj.l, obj.l, obj.l, obj.l, 0, 0, 0]  # -l/2
    y_corners = [0, 0, obj.h, obj.h, 0, 0, obj.h, obj.h]  # -h
    z_corners = [0, 0, 0, obj.w, obj.w, obj.w, obj.w, 0]  # -w/2

    x_corners = [i - obj.l / 2 for i in x_corners]
    y_corners = [i - obj.h for i in y_corners]
    z_corners = [i - obj.w / 2 for i in z_corners]

    corners_3D = np.array([x_corners, y_corners, z_corners])
    corners_3D = R.dot(corners_3D)
    corners_3D += np.array([obj.tx, obj.ty, obj.tz]).reshape((3, 1))

    corners_3D_1 = np.vstack((corners_3D, np.ones((corners_3D.shape[-1]))))
    corners_2D = P2.dot(corners_3D_1)
    corners_2D = corners_2D / corners_2D[2]
    corners_2D = corners_2D[:2]

    return corners_2D

def draw_3Dbox(ax, P2, line, color):

    corners_2D = compute_3Dbox(P2, line)

    # draw all lines through path
    # https://matplotlib.org/users/path_tutorial.html
    bb3d_lines_verts_idx = [0, 1, 2, 3, 4, 5, 6, 7, 0, 5, 4, 1, 2, 7, 6, 3]
    bb3d_on_2d_lines_verts = corners_2D[:, bb3d_lines_verts_idx]
    verts = bb3d_on_2d_lines_verts.T
    codes = [Path.LINETO] * verts.shape[0]
    codes[0] = Path.MOVETO
    # codes[-1] = Path.CLOSEPOLYq
    pth = Path(verts, codes)
    p = patches.PathPatch(pth, fill=False, color=color, linewidth=2)

    width = corners_2D[:, 3][0] - corners_2D[:, 1][0]
    height = corners_2D[:, 2][1] - corners_2D[:, 1][1]
    # put a mask on the front
    front_fill = patches.Rectangle((corners_2D[:, 1]), width, height, fill=True, color=color, alpha=0.4)
    ax.add_patch(p)
    ax.add_patch(front_fill)

# def visualization(args, image_path, label_path, calib_path, pred_path,
#                   dataset, VEHICLES):

#     for index in range(start_frame, end_frame):
#         image_file = os.path.join(image_path, dataset[index]+ '.png')
#         label_file = os.path.join(label_path, dataset[index] + '.txt')
#         prediction_file = os.path.join(pred_path, dataset[index]+ '.txt')
#         calibration_file = os.path.join(calib_path, dataset[index] + '.txt')
#         for line in open(calibration_file):
#             if 'P2' in line:
#                 P2 = line.split(' ')
#                 P2 = np.asarray([float(i) for i in P2[1:]])
#                 P2 = np.reshape(P2, (3, 4))


#         fig = plt.figure(figsize=(20.00, 5.12), dpi=100)

#         # fig.tight_layout()
#         gs = GridSpec(1, 4)
#         gs.update(wspace=0)  # set the spacing between axes.

#         ax = fig.add_subplot(gs[0, :3])
#         ax2 = fig.add_subplot(gs[0, 3:])

#         # with writer.saving(fig, "kitti_30_20fps.mp4", dpi=100):
#         image = Image.open(image_file).convert('RGB')
#         shape = 900
#         birdimage = np.zeros((shape, shape, 3), np.uint8)

#         with open(label_file) as f1, open(prediction_file) as f2:
#             for line_gt, line_p in zip(f1, f2):
#                 line_gt = line_gt.strip().split(' ')
#                 line_p = line_p.strip().split(' ')

#                 truncated = np.abs(float(line_p[1]))
#                 occluded = np.abs(float(line_p[2]))
#                 trunc_level = 1 if args.a == 'training' else 255

#             # truncated object in dataset is not observable
#                 if line_p[0] in VEHICLES  and truncated < trunc_level:
#                     color = 'green'
#                     if line_p[0] == 'Cyclist':
#                         color = 'yellow'
#                     elif line_p[0] == 'Pedestrian':
#                         color = 'cyan'
#                     draw_3Dbox(ax, P2, line_p, color)
#                     draw_birdeyes(ax2, line_gt, line_p, shape)

#         # visualize 3D bounding box
#         ax.imshow(image)
#         ax.set_xticks([]) #remove axis value
#         ax.set_yticks([])

#         # plot camera view range
#         x1 = np.linspace(0, shape / 2)
#         x2 = np.linspace(shape / 2, shape)
#         ax2.plot(x1, shape / 2 - x1, ls='--', color='grey', linewidth=1, alpha=0.5)
#         ax2.plot(x2, x2 - shape / 2, ls='--', color='grey', linewidth=1, alpha=0.5)
#         ax2.plot(shape / 2, 0, marker='+', markersize=16, markeredgecolor='red')

#         # visualize bird eye view
#         ax2.imshow(birdimage, origin='lower')
#         ax2.set_xticks([])
#         ax2.set_yticks([])
#         # add legend
#         handles, labels = ax2.get_legend_handles_labels()
#         legend = ax2.legend([handles[0], handles[1]], [labels[0], labels[1]], loc='lower right',
#                             fontsize='x-small', framealpha=0.2)
#         for text in legend.get_texts():
#             plt.setp(text, color='w')

#         print(dataset[index])
#         if args.save == False:
#             plt.show()
#         else:
#             fig.savefig(os.path.join(args.path, dataset[index]), dpi=fig.dpi, bbox_inches='tight', pad_inches=0)
#         # video_writer.write(np.uint8(fig))





class TrackVisualizer():
    def __init__(self, trkCfg):
        # breakpoint()
        self.associateGamma = trkCfg.associateGamma
        self.td = trkCfg.td
        self.thresholdTick = trkCfg.thresholdTick
        self.ID = 0
        self.n_meas = 3
    
    def getContour(self, trk):


        x_ma,y_ma = self.getAssociatonContour(trk, self.associateGamma, self.td)

        x_P_prev, y_P_prev, x_P, y_P = self.getPContour(trk, self.associateGamma)
        # print(x,y)
        # plt.plot(x_ma, y_ma,color='royalblue', linestyle = ":")
        if trk.associatedObj != None:
            x_R, y_R = self.getRContour(trk, self.associateGamma, self.n_meas)
            # plt.plot(x_R, y_R, color='lightgreen', linestyle = ":")
        # plt.plot(x_P, y_P, color='orangered', linestyle = ":")
        # plt.plot(x_P_prev, y_P_prev, color='gray', linestyle=":")
        # plt.show()
        if trk.associatedObj != None:
            return (x_ma, y_ma), (x_R, y_R), (x_P, y_P)
            
        else:
            return (x_ma, y_ma), (None, None), (x_P, y_P)
    def getAssociatonContour(self, currTrack, distance, td):

        Center = copy.deepcopy(currTrack.stateVectorXYZ)
        Center[0] = Center[0] + td * Center[2]
        Center[1] = Center[1] + td * Center[3]
        Center = Center[:2]
        # print("Center", Center)

        t = np.linspace(0, 2 * np.pi, 20)
        # print("DEBUG", currTrack.cov_mahalanobis)
        invCov = copy.deepcopy(currTrack.cov_mahalanobis)
        # print("debug", invCov)
        # invCov = invCov[:2,:2]
        eigVal, eigVec = np.linalg.eig(invCov)
        # eigVal = distance * np.sqrt(eigVal)
        eigVal = np.sqrt(distance * eigVal)

        xt = eigVal[0] * np.cos(t)
        yt = eigVal[1] * np.sin(t)
        xtyt = np.array([xt, yt])
        x_associationDistance = np.dot(eigVec[0, :], xtyt)
        y_associationDistance = np.dot(eigVec[1, :], xtyt)
        x_associationDistance = x_associationDistance + Center[0]
        y_associationDistance = y_associationDistance + Center[1]

        return x_associationDistance, y_associationDistance,

    def getRContour(self, currTrack, distance, n_meas):
        t = np.linspace(0, 2 * np.pi, 20)
        centorX = currTrack.associatedObj.measVectorRRD[0] * currTrack.associatedObj.measVectorRRD[2]
        centorY = np.sqrt((currTrack.associatedObj.measVectorRRD[0]) ** 2 - (centorX) ** 2)
        Rmat = np.zeros((n_meas, n_meas), dtype=np.float)
        for i in range(n_meas):
            Rmat[i][i] = currTrack.associatedObj.measCovVec[i]

        invHmat = np.linalg.pinv(currTrack.H_tmp)
        cov_mahal_tmp = np.matmul(invHmat, Rmat)
        cov_mahal_tmp = np.matmul(cov_mahal_tmp, invHmat.T)
        cov_mahalanobis = cov_mahal_tmp[:2, :2]
        invCov = copy.deepcopy(cov_mahalanobis)

        # print("debug", invCov)
        # invCov = invCov[:2,:2]
        eigVal, eigVec = np.linalg.eig(invCov)
        # eigVal = np.sqrt(distance *eigVal)
        eigVal = np.sqrt(distance * eigVal)

        xt = eigVal[0] * np.cos(t)
        yt = eigVal[1] * np.sin(t)
        xtyt = np.array([xt, yt])
        x = np.dot(eigVec[0, :], xtyt)
        y = np.dot(eigVec[1, :], xtyt)
        x = x + centorX
        y = y + centorY

        return x, y

    def getPContour(self, currTrack, distance):
        t = np.linspace(0, 2 * np.pi, 20)
        invCov = copy.deepcopy(currTrack.prevCovariance[:2, :2])

        # print("debug", invCov)
        # invCov = invCov[:2,:2]
        eigVal, eigVec = np.linalg.eig(invCov)
        # eigVal = np.sqrt(distance *eigVal)
        eigVal = np.sqrt(distance * eigVal)

        xt = eigVal[0] * np.cos(t)
        yt = eigVal[1] * np.sin(t)
        xtyt = np.array([xt, yt])
        x_prev = np.dot(eigVec[0, :], xtyt)
        y_prev = np.dot(eigVec[1, :], xtyt)
        x_prev = x_prev + currTrack.prevStateVectorXYZ[0] #P(k|k-1)
        y_prev = y_prev + currTrack.prevStateVectorXYZ[1] #P(k|k-1)

        invCov = copy.deepcopy(currTrack.Covariance[:2, :2])

        # print("debug", invCov)
        # invCov = invCov[:2,:2]
        eigVal, eigVec = np.linalg.eig(invCov)
        # eigVal = np.sqrt(distance *eigVal)
        eigVal = np.sqrt(distance * eigVal)

        xt = eigVal[0] * np.cos(t)
        yt = eigVal[1] * np.sin(t)
        xtyt = np.array([xt, yt])
        x_curr = np.dot(eigVec[0, :], xtyt)
        y_curr = np.dot(eigVec[1, :], xtyt)
        x_curr = x_curr + currTrack.stateVectorXYZ[0] #P(k|k)
        y_curr = y_curr + currTrack.stateVectorXYZ[1] #P(k|k)

        return x_prev, y_prev, x_curr,y_curr

# def pltPredictedTracker(TrackList,td):
# 	for track in TrackList:
# 		# print(track.tick)
# 		if track.ID != CONST.DEBUG_TRACKING_ID and not(CONST.TRACKING_ALL_DEBUG):
# 			continue

# 		xPred = track.stateVectorXYZ[0] + td * track.stateVectorXYZ[2]
# 		yPred = track.stateVectorXYZ[1] + td * track.stateVectorXYZ[3]

# 		plt.scatter(xPred, yPred,
# 		            marker='^', facecolors ='green', edgecolor = None, s = 50, label = "estimation before kalman")
# 		plt.plot([track.stateVectorXYZ[0],xPred],[track.stateVectorXYZ[1], yPred],
# 		         color = 'blue')