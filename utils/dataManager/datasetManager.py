from pathlib import Path
import glob
import os

from .videoManager import VideoManager

IMG_FORMATS = 'bmp', 'dng', 'jpeg', 'jpg', 'mpo', 'png', 'tif', 'tiff', 'webp'  # include image suffixes
VID_FORMATS = 'asf', 'avi', 'gif', 'm4v', 'mkv', 'mov', 'mp4', 'mpeg', 'mpg', 'ts', 'wmv'  # include video suffixes

class VideoDataSetManager():
    def __init__(self, inputDir, outputDir, prefix = ''):
        # pdb.set_trace()
        
        try:
            videoFiles = []
            for p in inputDir if isinstance(inputDir, list) else [inputDir]:
                p_path = Path(p)  # os-agnostic
                if p_path.is_dir():  # dir
                    videoFiles += glob.glob(str(p_path / '**' / '*.*'), recursive=True)
                elif p_path.is_file():
                    videoFiles += [p]
                else:
                    raise Exception(f'{prefix}{p} does not exist')

            self.videoFiles = sorted(x.replace('/', os.sep) for x in videoFiles if x.split('.')[-1].lower() in VID_FORMATS)

        except Exception as e:
            raise Exception(f'{prefix}Error loading data from {inputDir}: {e}')

        self.videoFiles.sort(key = lambda x: x.split('/')[-1]) # sort list by video File Name.

        self.outputDir = outputDir
    
    def __len__(self):
        return len(self.videoFiles)

    def __getitem__(self, idx):
        videoFile = self.videoFiles[idx]

        videoManger = VideoManager(videoFile,  
                                os.path.join(self.outputDir, 'processedOutput'))

        return videoManger, videoFile
