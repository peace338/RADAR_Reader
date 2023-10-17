import cv2
import os
import numpy as np
import math
from ..configuration import *
from matplotlib import cm

def draw_projected_box3d(image, qs, color=(255, 0, 255), thickness=1, onlyground = False):
    """ Draw 3d bounding box in image
        qs: (8,3) array of vertices for the 3d box in following order:
            5 -------- 6          
           /|         /|         
          4 -------- 7 .  z      
          | |        | |  |      
          . 1 -------- 2  . y     
          |/         |/   |/          
          0 -------- 3    0 -------- x    
    """     
    # breakpoint()
    qs = qs.astype(np.int32)
    for k in range(0, 4):
        # Ref: http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
        i, j = k, (k + 1) % 4
        # use LINE_AA for opencv3
        # cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness, cv2.CV_AA)
        cv2.line(image, (qs[i, 0], qs[i, 1]), (qs[j, 0], qs[j, 1]), color, thickness)
        if not onlyground:
            i, j = k + 4, (k + 1) % 4 + 4
            cv2.line(image, (qs[i, 0], qs[i, 1]), (qs[j, 0], qs[j, 1]), color, thickness)

            i, j = k, k + 4
            cv2.line(image, (qs[i, 0], qs[i, 1]), (qs[j, 0], qs[j, 1]), color, thickness)
    return image

def _concatFlagObjs(flags, objs3d):
    return np.concatenate((np.array(objs3d).reshape(-1,3), np.array(flags).reshape(-1,2)), axis = 1)

class DataManager():
    def __init__(self, inputFile, outputDir):
        self.inputFile = inputFile
        self.outputDir = outputDir
        self.fileName = self.inputFile.split('/')[-1]
        self.outputWfp = os.path.join(outputDir, self.fileName)

class VideoManager(DataManager):
    def __init__(self, inputFile, outputDir):
        super().__init__(inputFile, outputDir)
        
        self.cap = cv2.VideoCapture(self.inputFile)     
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.codec = "mp4v"
        self.fourcc = cv2.VideoWriter_fourcc(*self.codec)
        # pdb.set_trace()
        # self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) # 240 is trick
        self.width = 1200 # 240 is trick
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("video Size (height, width) = ({}, {})".format(self.height,self.width))
        self.vid_writer = cv2.VideoWriter(self.outputWfp, self.fourcc, self.fps, (self.width, self.height))
        
        assert SCATTER_FRAME > 0
        self.scatterFrame = SCATTER_FRAME 
        self.sp = [[],[],[]]
        # breakpoint()
        self.pingpongIdx = 0

        self.colormap = cm.get_cmap('jet')
        self.setColorbar()
        self.maxSize = self.height/480*10
        # print((self.width, self.height))
    def getVideoSize(self):
        
        return (self.height, self.width)

    # def captureVideo(self, frame, count):
    #     cv2.imwrite(os.path.join(self.outputDir, ).format(count), frame)
    def setColorbar(self):
        
        colorbar_width = int(self.width*30/640)
        colorbar_height = int(self.height*100/480)
        colorbar_image = np.zeros((colorbar_height, colorbar_width, 3), dtype=np.uint8)

        # Colorbar에 대한 높이에 따라 색상 생성
        for y in range(colorbar_height):
            normalized_value = 1 - y / colorbar_height  # 0~1 사이 값으로 정규화
            # breakpoint()
            color = np.array(self.colormap(normalized_value)[0:3][::-1])*255  # 컬러맵으로 색상 생성
            colorbar_image[y, :] = color

        # 빈 흰 배경 이미지 생성
        background = np.ones((colorbar_height, int(self.width*10/640), 3), dtype=np.uint8) * 255

        # Colorbar 이미지와 배경 이미지를 결합하여 최종 Colorbar 생성
        final_colorbar = np.hstack((colorbar_image, background))

        # Colorbar 그리드 생성
        num_grid_lines = 10
        grid_interval = colorbar_height // num_grid_lines
        for i in range(1, num_grid_lines):
            y = i * grid_interval
            cv2.line(final_colorbar, (0, i * grid_interval), (colorbar_width, i * grid_interval), (0, 0, 0), 1)
            label_text = f"{i} m"
            cv2.putText(final_colorbar, label_text, (colorbar_width -20, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        self.colorbar = final_colorbar
        self.colorbarPos = (0, self.width-self.colorbar.shape[1]) #y,x
        # breakpoint()
    def drawColorbar(self, frame):
        # breakpoint()
        frame[self.colorbarPos[0]:self.colorbarPos[0]+self.colorbar.shape[0], self.colorbarPos[1]:self.colorbarPos[1]+self.colorbar.shape[1], :] = self.colorbar
        # breakpoint()
    def videoWrite(self, frame, count = None):
        # breakpoint()
        self.vid_writer.write(frame)
        # print(frame.size)
        if count:
            # pdb.set_trace()
            cv2.imwrite(os.path.join(self.outputDir,'{}_{}.jpg'.format(self.fileName, count)), frame)

    def videoRead(self):
        # pdb.set_trace()
        ret, frame = self.cap.read()
        return ret, frame
    def drawCuboid(self, frame, trks, onylground = False):
        for trk in trks:
            draw_projected_box3d(frame, trk, onlyground=onylground)
            

    def scatter(self, frame, objs, flags):
        assert len(objs) == len(flags)
        # breakpoint()
        self.sp[self.pingpongIdx] = _concatFlagObjs(flags, objs)

        

        for objs in self.sp:
            for obj in objs:
                # breakpoint()
                # print(obj)
                if math.isnan(obj[2]):
                    # print()
                    continue
                z_normalized = (GRAPH_MAX_Y - obj[3]) / (GRAPH_MAX_Y - GRAPH_MIN_Y)
                color = self.colormap(z_normalized)
                # breakpoint()
                color = np.array(color)*255
                color = color[:3][::-1] #RGB to BGR
                size = round(self.maxSize * (z_normalized))
                # print(obj)
                if obj [0] > self.width or obj [1] > self.height or obj [0] < 0 or obj [1] < 0:
                    continue
                if obj[4] == 1:
                    cv2.circle(frame, (int(obj[0]), int(obj[1])), size, color, 1)
                else:
                    cv2.circle(frame, (int(obj[0]), int(obj[1])), size, color, -1)

        self.pingpongIdx += 1
        self.pingpongIdx = self.pingpongIdx % self.scatterFrame
    
    def textWrite(self, frame, text, location,plotColor = (255, 255, 0), plotFontScale = 1):
        cv2.putText(frame, 
                        text, 
                        location, 
                        cv2.FONT_ITALIC, 
                        color = plotColor, 
                        fontScale = plotFontScale)
        return frame
