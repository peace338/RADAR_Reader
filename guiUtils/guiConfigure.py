from radarEquipInfo import EQUIP_HEIGHT, AZIM_FOV, ELEV_FOV, MAX_HEIGHT

#####################################################################################
#   Constants - for User. User can modify the values freely                         #
#####################################################################################
GRAPH_MIN_X = -10          # (-20)
GRAPH_MAX_X = 10           # (20)
GRAPH_MIN_Y = 0
GRAPH_MAX_Y = 10
GRAPH_MAX_Z = 2.0
GRAPH_MIN_Z = 0

EGO_GRAPH_MIN_X = -90           # (-20)
EGO_GRAPH_MAX_X = 90           # (20)
EGO_GRAPH_MIN_Y = -4.6
EGO_GRAPH_MAX_Y = 4.6

VIDEO_FLIP_DEFAULT = True
RADAR1_UPDOWN_FLIP = False
RADAR2_UPDOWN_FLIP = True

SCATTER_FRAME = 3

ROI_MIN_X = -3
ROI_MAX_X = 3
ROI_MIN_Y = 0
ROI_MAX_Y = 5
ROI_MIN_Z = -EQUIP_HEIGHT
ROI_MAX_Z = ROI_MIN_Z + MAX_HEIGHT

HALF_AZIM_FOV = AZIM_FOV / 2
HALF_ELEV_FOV = ELEV_FOV / 2



BRUSH = [(255,255,255,255), (0,255,255,255), (255,0,255,255), (255,255,125,255), (255,125,255,255), (125,255,255,255), \
         (125,0,255,255), (125,255,0,255), (255,125,0,255), (0,125,255,255), (0,255,125,255), (255,0,125,255)]

