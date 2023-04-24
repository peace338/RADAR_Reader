from asyncio.windows_events import NULL
from lzma import is_check_supported
from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *
import simulation_algorithm_main as SAM
import dataStructure as DT
import pyqtgraph as pg                              # python -m pip install pyqtgraph
import os
import sys
import numpy as np                                  # python -m pip install numpy
import cv2                                          # python -m pip install opencv-python
import pickle                                       # python -m pip install pickle
import importlib

#####################################################################################
#   Constants - for User. User can modify the values freely                         #
#####################################################################################
GRAPH_MIN_X = -25           # (-20)
GRAPH_MAX_X = 25            # (20)
GRAPH_MIN_Y = 30
GRAPH_MAX_Y = 0
VIDEO_FLIP_DEFAULT = True
RADAR1_UPDOWN_FLIP = False
RADAR2_UPDOWN_FLIP = True
RADAR_OBJECT_ATTRIBUTES_LIST = ["range_idx","doppler_idx","range","speed","sin_azim",\
                                "peak_val","range_snr_db","doppler_snr_db","sin_azim_srn_lin","x",\
                                "y","z","rotate_x","rotate_y","vel_disamb_fac_valid",\
                                "vel_disamb_fac_valid_als","status_flag"]
RADAR_TRACK_ATTRIBUTES_LIST = ["id","x","y","xd","yd",\
                                "x_size","y_size","tick","age","flag",\
                                "reserved0"]
SIMULATED_OBJECT_ATTRIBUTES_LIST = ["detID","clusterId","rangeIdx","dopplerIdx","range",\
                                    "speed","sinAzim","rangeVal","rangeSNR","dopplerSNR",\
                                    "angleSNR","x","y","velDisambFacValid","status_flag",\
                                    "xd","yd"] 
SIMULATED_TRACK_ATTRIBUTES_LIST = ["ID","SNR","age","associ_ID","dopplerSNR",\
                                    "measVectorRRD_R","measVectorRRD_vel","measVectorRRD_Theta","statVecXYZ_x","statVecXYZ_y",\
                                    "statVecXYZ_xd","statVecXYZ_yd","peakVal","prevXd","prevYd",\
                                    "rangeSNR","tick","xSize","ySize","plotValidity",\
                                    "Status_Flag0","Status_Flag1"]


#####################################################################################
#   Class ==> Object and Track                                                      #
#####################################################################################
class radar_object() :
    def __init__(self):
        self.attributes_list = RADAR_OBJECT_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0

    
    def get_attributes_list() :
        attributes_list = RADAR_OBJECT_ATTRIBUTES_LIST
        return attributes_list

    def uint2int_16(input_data) :
        if input_data > 32767 :
            input_data = -(65535 - input_data + 1)
        return input_data
    
    def int2uint_16(input_data) :
        if input_data < 0 :
            input_data = 65535 + input_data + 1
        return input_data
        

class radar_track() :
    def __init__(self):
        self.attributes_list  =  RADAR_TRACK_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0

    def get_attributes_list() :
        attributes_list = RADAR_TRACK_ATTRIBUTES_LIST
        return attributes_list



class simulated_object() :
    def __init__(self):
        self.attributes_list  = SIMULATED_OBJECT_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0
    
    def get_attributes_list() :
        attributes_list = SIMULATED_OBJECT_ATTRIBUTES_LIST
        return attributes_list



class simulated_track() :
    def __init__(self):
        self.attributes_list  =  SIMULATED_TRACK_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0
    
    def get_attributes_list() :
        attributes_list = SIMULATED_TRACK_ATTRIBUTES_LIST
        return attributes_list


#####################################################################################
#   Constants - for system -  !!!!!!DO NOT CHANGE THE CONSTANTS below!!!!!!         #
#####################################################################################
RADAR1 = 1
RADAR2 = 2
Q7_DIVISOR = 128
Q8_DIVISOR = 256
Q14_DIVISOR = 16384
IT_IS_NOT_SIMULATION = 0
IT_IS_SIMULATION = 1


class App(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

#####################################################################################
#   Class - App ==> Drawing GUIs                                                    #
#####################################################################################
    def initUI(self):
        window_size = [1150, 800]
        dir_path = os.path.dirname(os.path.abspath(__file__))
        self.radar_file_name = []
        self.video_file_name = []
        self.file_name_load_flag = 0
        self.no_video_flag = 0
        self.origianl_radar1_frame_index = []
        self.origianl_radar2_frame_index = []
        self.simulated_radar1_frame_index = []
        self.simulated_radar2_frame_index = []
        self.video_current_frame_num = 0
        self.radar_current_frame_num = 0
        self.radar_video_frame_offset = 0
        self.video_end_frame_num = NULL
        self.radar_end_frame_num = NULL
        self.radar1_end_frame_num = NULL
        self.radar2_end_frame_num = NULL
        self.play_toggle_flag = 0 
        self.original_object_list = []
        self.original_track_list = []
        self.simulated_object_list = []
        self.simulated_track_list = []
        self.can_data = []


        self.setWindowTitle('BSD Developement Tool')
        self.setFixedSize(window_size[0],window_size[1])
        self.bold_font = QFont()
        self.bold_font.setBold(True)
        self.bold_font.setPointSize(11)
        self.tmp_font = QFont('Arial')
        self.tmp_font.setPointSize(11)
        
    # drawing Frames
        self.file_load_frame = QFrame(self)
        self.file_load_frame.setFixedSize(1110,40)
        self.file_load_frame.move(20,10)
        self.file_load_frame.setLineWidth(1)
        self.file_load_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)

        self.video_frame = QFrame(self)
        self.video_frame.setFixedSize(380, 285)
        self.video_frame.move(20,60)
        self.video_frame.setLineWidth(1)
        self.video_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)

        self.control_frame = QFrame(self)
        self.control_frame.setFixedSize(380, 205)
        self.control_frame.move(20,350)
        self.control_frame.setLineWidth(1)
        self.control_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)

        self.select_frame = QFrame(self)
        self.select_frame.setFixedSize(380, 120)
        self.select_frame.move(20,560)
        self.select_frame.setLineWidth(1)
        self.select_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)

        self.simulation_frame = QFrame(self)
        self.simulation_frame.setFixedSize(380, 95)
        self.simulation_frame.move(20,685)
        self.simulation_frame.setLineWidth(1)
        self.simulation_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)

        self.graph_frame = QFrame(self)
        self.graph_frame.setFixedSize(720, 720)
        self.graph_frame.move(410,60)
        self.graph_frame.setLineWidth(1)
        self.graph_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)

    # File Load Frame Contents
        self.file_path_line_edit = QLineEdit(self.file_load_frame)
        self.file_path_line_edit.setFixedSize(790,30)
        self.file_path_line_edit.move(5,5)
        self.file_path_line_edit.setText("Please, push 'OPEN' button to select Radar data file (*.rbd)")
        self.file_path_line_edit.setReadOnly(True)

        self.file_path_open_btn = QPushButton(self.file_load_frame)
        self.file_path_open_btn.setFixedSize(100,32)
        self.file_path_open_btn.move(800,4)
        self.file_path_open_btn.setText("OPEN")
        self.file_path_open_btn.clicked.connect(self.folder_select_btn_event)

        self.file_path_load_btn = QPushButton(self.file_load_frame)
        self.file_path_load_btn.setFixedSize(200,32)
        self.file_path_load_btn.move(905,4)
        self.file_path_load_btn.setText("L O A D")
        self.file_path_load_btn.setFont(self.bold_font)
        self.file_path_load_btn.clicked.connect(self.load_btn_event)
    
    # Control Frame contents
        # Draw Frame control tab contents
        self.frame_control_widget = QWidget(self)
        self.frame_control_widget.setFixedSize(380, 180)
        self.previous_frame_btn = QPushButton(self.frame_control_widget)
        self.previous_frame_btn.setFixedSize(100,60)
        self.previous_frame_btn.move(17,15)
        self.previous_frame_btn.setText("-1")
        self.previous_frame_btn.setFont(self.bold_font)
        self.previous_frame_btn.setShortcut("a")
        self.previous_frame_btn.setShortcutEnabled(True)
        self.previous_frame_btn.clicked.connect(self.previous_frame_btn_event)
        self.play_btn = QPushButton(self.frame_control_widget)
        self.play_btn.setFixedSize(100,60)
        self.play_btn.move(130,15)
        self.play_btn.setText("▶")
        # self.play_btn.setShortcut("s")
        self.play_btn.setFont(self.bold_font)
        self.play_btn.clicked.connect(self.play_btn_event)
        self.forward_frame_btn = QPushButton(self.frame_control_widget)
        self.forward_frame_btn.setFixedSize(100,60)
        self.forward_frame_btn.move(243,15)
        self.forward_frame_btn.setText("+1")
        self.forward_frame_btn.setFont(self.bold_font)
        self.forward_frame_btn.setShortcut("d")
        self.forward_frame_btn.setShortcutEnabled(True)
        self.forward_frame_btn.clicked.connect(self.forward_frame_btn_event)
        self.fast_rewind_btn = QPushButton(self.frame_control_widget)
        self.fast_rewind_btn.setFixedSize(155,40)
        self.fast_rewind_btn.move(17,85)
        self.fast_rewind_btn.setText("-10")
        self.fast_rewind_btn.setFont(self.bold_font)
        self.fast_rewind_btn.setShortcut("q")
        self.fast_rewind_btn.setShortcutEnabled(True)
        self.fast_rewind_btn.clicked.connect(self.fast_rewind_btn_event)
        self.fast_forward_btn = QPushButton(self.frame_control_widget)
        self.fast_forward_btn.setFixedSize(155,40)
        self.fast_forward_btn.move(188,85)
        self.fast_forward_btn.setText("+10")
        self.fast_forward_btn.setFont(self.bold_font)
        self.fast_forward_btn.setShortcut("e")
        self.fast_forward_btn.setShortcutEnabled(True)
        self.fast_forward_btn.clicked.connect(self.fast_forward_btn_event)
        self.frame_number_label = QLabel(self.frame_control_widget)
        self.frame_number_label.setFont(self.tmp_font)
        self.frame_number_label.setText("Go to")
        self.frame_number_label.move(17, 138)
        self.frame_number_line_edit = QLineEdit(self.frame_control_widget)
        self.frame_number_line_edit.setFixedSize(110,25)
        self.frame_number_line_edit.move(55, 135)
        self.frame_number_line_edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.frame_number_line_edit.editingFinished.connect(self.frame_number_jump_btn_event)
        self.frame_number_jump_btn = QPushButton(self.frame_control_widget)
        self.frame_number_jump_btn.setText("JUMP")
        self.frame_number_jump_btn.setFixedSize(69,27)
        self.frame_number_jump_btn.move(180, 134)
        self.frame_number_jump_btn.clicked.connect(self.frame_number_jump_btn_event)
        self.fast_speed_combobox = QComboBox(self.frame_control_widget)
        self.fast_speed_combobox.setFixedSize(80,25)
        self.fast_speed_combobox.move(263,135)
        self.fast_speed_combobox.setEditable(True)
        self.fast_speed_combobox.setFont(self.tmp_font)
        self.fast_speed_combobox_edit = self.fast_speed_combobox.lineEdit()
        self.fast_speed_combobox_edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fast_speed_combobox_edit.setReadOnly(True)
        self.fast_speed_combobox.addItem("x1")
        self.fast_speed_combobox.addItem("x2")
        self.fast_speed_combobox.addItem("x4")
        self.fast_speed_combobox.addItem("x8")

        # Draw Sync control tab contents
        self.sync_control_widget = QWidget(self)
        self.sync_control_widget.setFixedSize(380, 180)
        self.radar_previous_frame_btn = QPushButton(self.sync_control_widget)
        self.radar_previous_frame_btn.setFixedSize(160,40)
        self.radar_previous_frame_btn.move(17,15)
        self.radar_previous_frame_btn.setText("◀")
        self.radar_previous_frame_btn.setFont(self.bold_font)
        self.radar_previous_frame_btn.setShortcut("a")
        self.radar_previous_frame_btn.clicked.connect(self.radar_previous_frame_btn_event)
        self.radar_next_frame_btn = QPushButton(self.sync_control_widget)
        self.radar_next_frame_btn.setFixedSize(160,40)
        self.radar_next_frame_btn.move(187,15)
        self.radar_next_frame_btn.setText("▶")
        self.radar_next_frame_btn.setFont(self.bold_font)
        self.radar_next_frame_btn.setShortcut("d")
        self.radar_next_frame_btn.clicked.connect(self.radar_next_frame_btn_event)
        self.radar_frame_slider = QSlider(self.sync_control_widget,orientation = Qt.Orientation.Horizontal)
        self.radar_frame_slider.setFixedSize(270,20)
        self.radar_frame_slider.move(17,65)
        self.radar_frame_slider.setMinimum(0)
        self.radar_frame_slider.setMaximum(1)
        self.radar_frame_slider.setValue(0)
        self.radar_frame_info_label = QLabel(self.sync_control_widget)
        self.radar_frame_info_label.setFixedSize(80,20)
        self.radar_frame_info_label.move(300,62)
        self.radar_frame_info_label.setText("No Radar")
        self.video_previous_frame_btn = QPushButton(self.sync_control_widget)
        self.video_previous_frame_btn.setFixedSize(160,40)
        self.video_previous_frame_btn.move(17,95)
        self.video_previous_frame_btn.setText("◀")
        self.video_previous_frame_btn.setFont(self.bold_font)
        self.video_previous_frame_btn.setShortcut("q")
        self.video_previous_frame_btn.clicked.connect(self.video_previous_frame_btn_event)
        self.video_next_frame_btn = QPushButton(self.sync_control_widget)
        self.video_next_frame_btn.setFixedSize(160,40)
        self.video_next_frame_btn.move(187,95)
        self.video_next_frame_btn.setText("▶")
        self.video_next_frame_btn.setFont(self.bold_font)
        self.video_next_frame_btn.setShortcut("e")
        self.video_next_frame_btn.clicked.connect(self.video_next_frame_btn_event)
        self.video_frame_slider = QSlider(self.sync_control_widget,orientation = Qt.Orientation.Horizontal)
        self.video_frame_slider.setFixedSize(270,20)
        self.video_frame_slider.move(17,145)
        self.video_frame_slider.setMinimum(0)
        self.video_frame_slider.setMaximum(1)
        self.video_frame_slider.setValue(0)
        self.video_frame_info_label = QLabel(self.sync_control_widget)
        self.video_frame_info_label.setFixedSize(80,20)
        self.video_frame_info_label.move(300,142)
        self.video_frame_info_label.setText("No Video")

        # Draw View control tab contents
        self.view_control_widget = QWidget(self)
        self.view_control_widget.setFixedSize(380, 180)



        # Add above control tabs to Tabwidget
        self.frame_control_tabwidget = QTabWidget(self.control_frame)
        self.frame_control_tabwidget.setFixedSize(370, 195)
        self.frame_control_tabwidget.move(5, 5)
        self.frame_control_tabwidget.addTab(self.frame_control_widget,'Frame Control')
        self.frame_control_tabwidget.addTab(self.sync_control_widget,'Sync Control')
        self.frame_control_tabwidget.addTab(self.view_control_widget,'View Control')
        self.frame_control_tabwidget.setTabShape(QTabWidget.TabShape.Triangular)
        
        # Change Tab Widget Colors
        self.frame_control_widget.setAutoFillBackground(True)
        tmp_plt = self.frame_control_widget.palette()
        tmp_plt.setColor(self.frame_control_widget.backgroundRole(),QColor(240, 240, 240, 255)) #QColor RGB : 240, 240, 240 is Background gray color.
        self.frame_control_widget.setPalette(tmp_plt)
        self.sync_control_widget.setAutoFillBackground(True)
        tmp_plt = self.sync_control_widget.palette()
        tmp_plt.setColor(self.sync_control_widget.backgroundRole(),QColor(240, 240, 240, 255)) #QColor RGB : 240, 240, 240 is Background gray color.
        self.sync_control_widget.setPalette(tmp_plt)
        self.view_control_widget.setAutoFillBackground(True)
        tmp_plt = self.view_control_widget.palette()
        tmp_plt.setColor(self.view_control_widget.backgroundRole(),QColor(240, 240, 240, 255)) #QColor RGB : 240, 240, 240 is Background gray color.
        self.view_control_widget.setPalette(tmp_plt)
        


    # Select Frame contents
        self.tmp_font.setPointSize(15)
        self.tmp_font.setBold(True)
        self.radar1_select_radiobutton = QRadioButton(self.select_frame)
        self.radar1_select_radiobutton.setText("Radar1")
        self.radar1_select_radiobutton.move(30, 10)
        self.radar1_select_radiobutton.setFont(self.tmp_font)
        self.radar1_select_radiobutton.setChecked(True)
        self.radar1_select_radiobutton.clicked.connect(self.radar_select_radiobutton_event)
        self.radar2_select_radiobutton = QRadioButton(self.select_frame)
        self.radar2_select_radiobutton.setText("Radar2")
        self.radar2_select_radiobutton.setFont(self.tmp_font)
        self.radar2_select_radiobutton.move(160, 10)
        self.radar2_select_radiobutton.clicked.connect(self.radar_select_radiobutton_event)
        self.tmp_font.setPointSize(10)
        self.video12_flip = QCheckBox(self.select_frame)
        self.video12_flip.setText("Video Flip")
        self.video12_flip.setFont(self.tmp_font)
        self.video12_flip.move(270, 15)
        self.video12_flip.setChecked(VIDEO_FLIP_DEFAULT)
        self.tmp_font.setPointSize(14)
        self.original_radar_label = QLabel(self.select_frame)
        self.original_radar_label.setText("Original")
        self.original_radar_label.setFont(self.tmp_font)
        self.original_radar_label.move(40, 39)
        self.simulateed_radar_label = QLabel(self.select_frame)
        self.simulateed_radar_label.setText("Simulated")
        self.simulateed_radar_label.setFont(self.tmp_font)
        self.simulateed_radar_label.move(40, 82)
        self.tmp_font.setPointSize(12)
        self.original_object_checkbox = QCheckBox(self.select_frame)
        self.original_object_checkbox.setText("Object")
        self.original_object_checkbox.setFont(self.tmp_font)
        self.original_object_checkbox.move(150, 40)
        self.original_object_checkbox.setChecked(True)
        self.original_object_checkbox.stateChanged.connect(self.checkbox_change_event)
        self.original_track_checkbox = QCheckBox(self.select_frame)
        self.original_track_checkbox.setText("Track")
        self.original_track_checkbox.setFont(self.tmp_font)
        self.original_track_checkbox.move(270, 40)
        self.original_track_checkbox.setChecked(True)
        self.original_track_checkbox.stateChanged.connect(self.checkbox_change_event)
        self.simulated_object_checkbox = QCheckBox(self.select_frame)
        self.simulated_object_checkbox.setText("Object")
        self.simulated_object_checkbox.setFont(self.tmp_font)
        self.simulated_object_checkbox.move(150, 70)
        self.simulated_object_checkbox.setChecked(True)
        self.simulated_object_checkbox.stateChanged.connect(self.checkbox_change_event)
        self.simulated_track_checkbox = QCheckBox(self.select_frame)
        self.simulated_track_checkbox.setText("Track")
        self.simulated_track_checkbox.setFont(self.tmp_font)
        self.simulated_track_checkbox.move(270, 70)
        self.simulated_track_checkbox.setChecked(True)
        self.simulated_track_checkbox.stateChanged.connect(self.checkbox_change_event)
        self.simulated_pruned_checkbox = QCheckBox(self.select_frame)
        self.simulated_pruned_checkbox.setText("Pruned obj")
        self.simulated_pruned_checkbox.setFont(self.tmp_font)
        self.simulated_pruned_checkbox.move(150, 95)
        self.simulated_pruned_checkbox.setChecked(True)
        self.simulated_pruned_checkbox.stateChanged.connect(self.checkbox_change_event)
        self.simulated_pruned_checkbox.setEnabled(False)
        self.simulated_candidate_checkbox = QCheckBox(self.select_frame)
        self.simulated_candidate_checkbox.setText("Candidate")
        self.simulated_candidate_checkbox.setFont(self.tmp_font)
        self.simulated_candidate_checkbox.move(270, 95)
        self.simulated_candidate_checkbox.setChecked(True)
        self.simulated_candidate_checkbox.stateChanged.connect(self.checkbox_change_event)
        

    # Simulation Frame contents
        self.radar1_simulation_checkbox = QCheckBox(self.simulation_frame)
        self.radar1_simulation_checkbox.setText("Radar1")
        self.radar1_simulation_checkbox.setFont(self.tmp_font)
        self.radar1_simulation_checkbox.move(70, 10)
        self.radar1_simulation_checkbox.setChecked(True)
        self.radar2_simulation_checkbox = QCheckBox(self.simulation_frame)
        self.radar2_simulation_checkbox.setText("Radar2")
        self.radar2_simulation_checkbox.setFont(self.tmp_font)
        self.radar2_simulation_checkbox.move(220, 10)
        self.radar2_simulation_checkbox.setChecked(True)
        self.simulation_btn = QPushButton(self.simulation_frame)
        self.simulation_btn.setFixedSize(343,35)
        self.simulation_btn.move(17,42)
        self.simulation_btn.setText("Simulation")
        self.simulation_btn.setFont(self.tmp_font)
        self.simulation_btn.clicked.connect(self.simulation_btn_event)
        self.simulation_progress_bar = QProgressBar(self.simulation_frame)
        self.simulation_progress_bar.setFixedSize(343,35)
        self.simulation_progress_bar.move(17,42)
        self.simulation_progress_bar.setRange(0,1)
        self.simulation_progress_bar.setValue(0)
        self.simulation_progress_bar.setTextVisible(True)
        self.simulation_progress_bar.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.simulation_progress_bar.setVisible(False)
        
        

    # Video Frame contetns(using OpenCV)
        self.image_file_name = dir_path+"\\movon_img.jpg"
        self.image = cv2.imread(self.image_file_name)
        height, width, bytevalue = self.image.shape
        bytevalue = bytevalue * width
        cv2.cvtColor(self.image,cv2.COLOR_BGR2RGB,self.image)
        self.image_from_video = QImage(self.image,width,height,bytevalue,QImage.Format.Format_RGB888)
        self.pixmap = QPixmap()
        self.pixmap.convertFromImage(self.image_from_video)
        self.image_lable = QLabel(self.video_frame)
        self.image_lable.setFixedSize(380,285)
        self.image_lable.setPixmap(self.pixmap)


    # Graph Frame Contents
        self.original_radar_widget = QWidget(self.graph_frame)
        self.original_radar_widget.setFixedSize(350,690)
        self.original_radar_widget.move(10, 20)
        self.simulated_radar_widget = QWidget(self.graph_frame)
        self.simulated_radar_widget.setFixedSize(350,690)
        self.simulated_radar_widget.move(360, 20)
        self.original_graph_label = QLabel(self.graph_frame)
        self.original_graph_label.move(20,5)
        self.original_graph_label.setText("Original Data")
        self.graph_font = self.original_graph_label.font()
        self.graph_font.setBold(True)
        self.original_graph_label.setFont(self.graph_font)
        self.simulated_graph_label = QLabel(self.graph_frame)
        self.simulated_graph_label.move(370,5)
        self.simulated_graph_label.setText("Simulated Data")
        self.simulated_graph_label.setFont(self.graph_font)


        self.original_radar_plot = pg.plot()
        self.simulated_radar_plot = pg.plot()
        original_radar_plot_menu_item = self.original_radar_plot.plotItem.vb.menu.actions()
        self.original_radar_plot.plotItem.vb.menu.removeAction(original_radar_plot_menu_item[3])
        simulated_radar_plot_menu_item = self.simulated_radar_plot.plotItem.vb.menu.actions()
        self.simulated_radar_plot.plotItem.vb.menu.removeAction(simulated_radar_plot_menu_item[3])
        #################################### Menu Control Ingredient ############################
        # self.original_radar_plot.plotItem.setMenuEnabled(enableMenu=False, enableViewBoxMenu=True)
        # test = QMenu('test_menu')
        # test.addMenu('test_new_menu')
        # self.original_radar_plot.sceneObj.contextMenu.insert(0,test)
        # test = self.original_radar_plot.plotItem.vb.menu.actions()
        # self.original_radar_plot.plotItem.vb.menu.removeAction(test[0])
        # self.original_radar_plot.plotItem.vb.menu.removeAction(test[1])
        # self.original_radar_plot.plotItem.vb.menu.removeAction(test[2])
        # self.original_radar_plot.plotItem.vb.menu.removeAction(test[3])
        # print(self.original_radar_plot.plotItem.vb.menu)
        # print(dir(self.original_radar_plot.plotItem.vb.menu))
        # print((test[0]))
        #########################################################################################
        self.original_radar_plot.setRange(QRectF(GRAPH_MIN_X, GRAPH_MIN_Y, GRAPH_MAX_X-GRAPH_MIN_X, GRAPH_MAX_Y-GRAPH_MIN_Y),disableAutoRange = True)
        self.original_radar_plot.plotItem.showGrid(True, True, 0.5)
        self.simulated_radar_plot.setRange(QRectF(GRAPH_MIN_X, GRAPH_MIN_Y, GRAPH_MAX_X-GRAPH_MIN_X, GRAPH_MAX_Y-GRAPH_MIN_Y),disableAutoRange = True)
        self.simulated_radar_plot.plotItem.showGrid(True, True, 0.5)
        

        self.original_scatter = pg.ScatterPlotItem(size=3, brush=pg.mkBrush(255, 255, 255, 255))
        self.simulated_scatter = pg.ScatterPlotItem(size=3, brush=pg.mkBrush(255, 255, 255, 255))
        self.original_scatter.setSize(7)
        self.simulated_scatter.setSize(7)
        self.original_scatter.sigClicked.connect(self.read_points)
        self.simulated_scatter.sigClicked.connect(self.read_points)
        
        pg.setConfigOption('leftButtonPan', False)

        self.original_radar_plot.addItem(self.original_scatter)
        self.simulated_radar_plot.addItem(self.simulated_scatter)

        self.original_layout = QGridLayout()
        self.simulated_layout = QGridLayout()

        self.original_layout.addWidget(self.original_radar_plot)
        self.simulated_layout.addWidget(self.simulated_radar_plot)

        self.original_radar_widget.setLayout(self.original_layout)
        self.simulated_radar_widget.setLayout(self.simulated_layout)

        self.original_graph_info_window = QDialog(self)
        self.original_graph_info_window.setFixedSize(275, 550)
        self.original_graph_info_window.setWindowTitle("Original Data Info Window")
        self.original_graph_table = QTableWidget(self.original_graph_info_window)
        self.original_graph_table.setFixedSize(275,550)
        self.original_graph_table.setRowCount(15)
        self.original_graph_table.setColumnCount(2)
        self.original_graph_table.setColumnWidth(1,150)
        self.original_graph_table.setFont(QFont('Arial', 10))
        self.original_graph_table.horizontalHeader().hide()
        self.original_graph_table.verticalHeader().hide()
        self.original_graph_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        
        self.simulated_graph_info_window = QDialog(self)
        self.simulated_graph_info_window.setFixedSize(275, 300)
        self.simulated_graph_info_window.setWindowTitle("Simulated Data Info Window")
        self.simulated_graph_table = QTableWidget(self.simulated_graph_info_window)
        self.simulated_graph_table.setFixedSize(275,300)
        self.simulated_graph_table.setRowCount(15)
        self.simulated_graph_table.setColumnCount(2)
        self.simulated_graph_table.setColumnWidth(1,150)
        self.simulated_graph_table.setFont(QFont('Arial', 10))
        self.simulated_graph_table.horizontalHeader().hide()
        self.simulated_graph_table.verticalHeader().hide()
        self.simulated_graph_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)

    # Register shortcuts
        self.enable_all_components(False)
        self.reset_graphview_shortcut = QShortcut("f",self)
        self.reset_graphview_shortcut.activated.connect(self.view_reset_shortcut_event)
        self.reset_frame_curser = QShortcut("r",self)
        self.reset_frame_curser.activated.connect(self.reset_frame_curser_event)
        self.jumpscale_2pow1_shortcut = QShortcut("1",self)
        self.jumpscale_2pow1_shortcut.activated.connect(self.jump_scale1_sel_event)
        self.jumpscale_2pow2_shortcut = QShortcut("2",self)
        self.jumpscale_2pow2_shortcut.activated.connect(self.jump_scale2_sel_event)
        self.jumpscale_2pow3_shortcut = QShortcut("3",self)
        self.jumpscale_2pow3_shortcut.activated.connect(self.jump_scale3_sel_event)
        self.jumpscale_2pow4_shortcut = QShortcut("4",self)
        self.jumpscale_2pow4_shortcut.activated.connect(self.jump_scale4_sel_event)
        self.play_shortcut = QShortcut(" ",self)
        self.play_shortcut.activated.connect(self.play_btn_event)

    ########################################## GUI Drawing End ####################################




    #####################################################################################
    #   Class - App ==> Button Functions                                                #
    #####################################################################################
    def folder_select_btn_event(self):
        tmp_file_name = self.file_path_line_edit.text()
        new_file_structure = QFileDialog.getOpenFileName(self, "Select File", filter = "Radar Data Bin(*.rdb)")
        self.no_video_flag = 0
        self.no_video2_flag = 0
        self.no_simulation_data_flag = 0
        if new_file_structure[0] == '' :
            new_file_name = tmp_file_name
        else :
            new_file_name = new_file_structure[0]
            extention = new_file_name.split('.')
            if extention[1] != 'rdb' :
                self.general_error_window("FILE ERROR","It is NOT Radar Data file. Check Again")
                new_file_name = tmp_file_name
            else :
                video_file_name = extention[0]+'_1.avi'
                video2_file_name = extention[0]+'_2.avi'
                if os.path.isfile(video_file_name) :
                    self.video_file_name = video_file_name
                else :
                    self.no_video_flag = 1

                if os.path.isfile(video2_file_name) :
                    self.video2_file_name = video2_file_name 
                else :
                    self.no_video2_flag = 1

                if (self.no_video_flag & self.no_video2_flag) == 1 :
                    self.general_warning_window("NO VIDEO","Radar Data is loaded without Video")
                
                simulation_file_name = extention[0]+'.sdb'
                if os.path.isfile(simulation_file_name) :
                    self.simulation_file_name = simulation_file_name
                else :
                    self.no_simulation_data_flag = 1
            self.file_path_line_edit.setText(new_file_name)
            self.video_file_name = video_file_name
            self.radar_file_name = new_file_name
            self.simulation_file_name = simulation_file_name
            self.file_name_load_flag = 1


    def load_btn_event(self) :
        self.origianl_radar1_frame_index = []
        self.origianl_radar2_frame_index = []
        self.simulated_radar1_frame_index = []
        self.simulated_radar2_frame_index = []
        self.radar_end_frame_num = 0
        self.radar1_end_frame_num = 0
        self.radar2_end_frame_num = 0
        self.video_end_frame_num = 0
        self.video_current_frame_num = 0
        self.radar_current_frame_num = 0
        self.radar_video_frame_offset = 0


        self.original_scatter.clear()
        self.simulated_scatter.clear()

        if self.file_name_load_flag == 0 :
            self.general_error_window("RADAR FILE ERROR","Radar Data File is NOT selected...")
        else : 
            load_flag = 0
            if os.path.getsize(self.radar_file_name) == 0 :
                self.general_error_window("RADAR FILE ERROR","Radar file is unavailable. Check the file.") 
                load_error_flag_neg = 0
            else :
                self.original_radar_file_handle = open(self.radar_file_name,'rb')
                load_flag = 1               # indicator for meeting end of file.
                load_error_flag_neg = 1     # 0 means error
            load_error_counter = 0
            while (load_flag * load_error_flag_neg):
                while True : 
                    try :
                        data1 = self.original_radar_file_handle.read(1)
                    except : 
                        load_flag = 0
                        break
                    if data1 == b'M' :
                        data1 = self.original_radar_file_handle.read(1)
                        if data1 == b'V' :
                            data1 = self.original_radar_file_handle.read(1)
                            if data1 == b'R' :
                                data1 = self.original_radar_file_handle.read(1)
                                if data1 == b'S' :
                                    data1 = self.original_radar_file_handle.read(6)
                                    if data1 == b'_RPAS1' :
                                        file_curser = self.original_radar_file_handle.tell()
                                        load_error_counter = 0
                                        self.origianl_radar1_frame_index.append(file_curser)
                                    if data1 == b'_RPAS2' :
                                        file_curser = self.original_radar_file_handle.tell()
                                        load_error_counter = 0
                                        self.origianl_radar2_frame_index.append(file_curser)
                                    break
                    elif data1 == b'' :
                        load_flag = 0
                        break
                    else : 
                        load_error_counter += 1
                        if load_error_counter > 2048*4 :
                            load_error_flag_neg = 0
                            # self.general_error_window("RADAR FILE ERROR","Radar file is unavailable. \nCheck the file.") 

            if self.no_simulation_data_flag == 0 :
                self.simulated_radar_file_handle = open(self.simulation_file_name,'rb')
                load_flag = 1
                load_error_counter = 0
                load_error_flag_neg = 1 
                while (load_flag * load_error_flag_neg):
                    while True : 
                        try :
                            data1 = self.simulated_radar_file_handle.read(1)
                        except : 
                            load_flag = 0
                            break
                        if data1 == b'R' :
                            data1 = self.simulated_radar_file_handle.read(1)
                            if data1 == b'S' :
                                data1 = self.simulated_radar_file_handle.read(1)
                                if data1 == b'I' :
                                    data1 = self.simulated_radar_file_handle.read(1)
                                    if data1 == b'M' :
                                        data1 = self.simulated_radar_file_handle.read(6)
                                        if data1 == b'_RPAS1' :
                                            file_curser = self.simulated_radar_file_handle.tell()
                                            load_error_counter = 0
                                            self.simulated_radar1_frame_index.append(file_curser)
                                        if data1 == b'_RPAS2' :
                                            file_curser = self.simulated_radar_file_handle.tell()
                                            load_error_counter = 0
                                            self.simulated_radar2_frame_index.append(file_curser)
                                        break
                        elif data1 == b'' :
                            load_flag = 0
                            break
                        else : 
                            load_error_counter += 1
                            if load_error_counter > 2048*4 :
                                load_error_flag_neg = 0
                                self.general_error_window("SIMULATION FILE ERROR","Simulation file is unavailable. \nCheck the file.") 
            self.radar1_end_frame_num = len(self.origianl_radar1_frame_index)
            self.radar2_end_frame_num = len(self.origianl_radar2_frame_index)
            if self.radar1_end_frame_num == NULL :
                self.radar_end_frame_num = self.radar2_end_frame_num
            elif self.radar2_end_frame_num == NULL :
                self.radar_end_frame_num = self.radar1_end_frame_num
            else :
                self.radar_end_frame_num = min(self.radar1_end_frame_num, self.radar2_end_frame_num)
            if self.no_simulation_data_flag == 0 :
                if len(self.simulated_radar1_frame_index) == NULL :
                    tmp_simulation_frame_num = len(self.simulated_radar2_frame_index)
                elif len(self.simulated_radar2_frame_index) == NULL :
                    tmp_simulation_frame_num = len(self.simulated_radar1_frame_index)
                elif len(self.simulated_radar1_frame_index) == len(self.simulated_radar2_frame_index) :
                    tmp_simulation_frame_num = len(self.simulated_radar1_frame_index)
                else :
                    self.general_warning_window("SDB FILE ERROR", "Simulation frame information is not matched")
                    self.no_simulation_data_flag = 1
                    self.simulated_radar1_frame_index = []
                    self.simulated_radar2_frame_index = []
                if  tmp_simulation_frame_num != self.radar_end_frame_num :
                    self.general_warning_window("SDB FILE ERROR", "Simulation frame information is not matched")
                    self.no_simulation_data_flag = 1
                    self.simulated_radar1_frame_index = []
                    self.simulated_radar2_frame_index = []
            self.radar_frame_info_label.setText(str(self.radar_current_frame_num)+"/"+str(self.radar_end_frame_num))
            self.radar_frame_slider.setMaximum(self.radar_end_frame_num)
            self.frame_number_line_edit.setText(str(self.radar_current_frame_num))
            self.error_flag = 0
            if self.no_video_flag & self.no_video2_flag == 0:
                if self.no_video_flag == 0 :
                    self.video_handle = cv2.VideoCapture(self.video_file_name)
                    self.video_end_frame_num = int(self.video_handle.get(cv2.CAP_PROP_FRAME_COUNT))
                if self.no_video2_flag == 0 :
                    self.video2_handle = cv2.VideoCapture(self.video2_file_name)
                    self.video2_end_frame_num = int(self.video2_handle.get(cv2.CAP_PROP_FRAME_COUNT))
                    self.video_end_frame_num = self.video2_end_frame_num    # This makes other functions and codes happy, even if there is no Video1 file
                if self.no_video_flag | self.no_video2_flag == 0:
                    if self.video_end_frame_num != self.video2_end_frame_num :
                        self.general_warning_window("VIDEO MISMATCH","Video frame lengths are not matched.\nRadar1 video frame number is applied.")
                self.video_frame_info_label.setText(str(self.video_current_frame_num)+"/"+str(self.video_end_frame_num))
                self.video_frame_slider.setMaximum(self.video_end_frame_num)
                    
            else :
                height, width, bytevalue = self.image.shape
                bytevalue = bytevalue * width
                # cv2.cvtColor(self.image,cv2.COLOR_BGR2RGB,self.image)
                self.image_from_video = QImage(self.image,width,height,bytevalue,QImage.Format.Format_RGB888)
                self.pixmap.convertFromImage(self.image_from_video)
                self.image_lable.setPixmap(self.pixmap)
                self.video_frame_info_label.setText("No Video")
                self.video_frame_slider.setMaximum(1)
                self.video_frame_slider.setValue(self.video_current_frame_num)
            if load_error_flag_neg :                    # it means no Error
                self.component_enable_check_load()
                self.set_to_this_frame(0)
            


    def previous_frame_btn_event(self) :
        if self.radar_current_frame_num > 0:
            self.radar_current_frame_num -= 1
            self.set_to_this_frame(self.radar_current_frame_num)
        else :
            return
        
        
    def forward_frame_btn_event(self) :
        if self.radar_current_frame_num < self.radar_end_frame_num-1 :  # it is because index starts from 0
            self.radar_current_frame_num += 1
            self.set_to_this_frame(self.radar_current_frame_num)
        else :
            return
    
    def fast_forward_btn_event(self) :
        scale = 2 ** self.fast_speed_combobox.currentIndex()
        if self.radar_current_frame_num < self.radar_end_frame_num-10*scale -1 :  # it is because index starts from 0
            self.radar_current_frame_num += 10*scale
            self.set_to_this_frame(self.radar_current_frame_num)
        else :
            self.radar_current_frame_num = self.radar_end_frame_num-1
            self.set_to_this_frame(self.radar_current_frame_num)
    
    
    def fast_rewind_btn_event(self) :
        scale = 2 ** self.fast_speed_combobox.currentIndex()
        if self.radar_current_frame_num > 10*scale:
            self.radar_current_frame_num -= 10*scale
            self.set_to_this_frame(self.radar_current_frame_num)
        else :
            self.radar_current_frame_num = 0
            self.set_to_this_frame(self.radar_current_frame_num)

    def frame_number_jump_btn_event(self) :
        if self.frame_number_line_edit.text() == "" :
            return
        try :
            int(self.frame_number_line_edit.text())
        except :
            self.frame_number_line_edit.setText(str(self.radar_current_frame_num))
            return
        if (int(self.frame_number_line_edit.text()) >= self.radar_end_frame_num) :
            self.frame_number_line_edit.setText(str(self.radar_end_frame_num-1))
            
        self.radar_current_frame_num = int(self.frame_number_line_edit.text())
        self.set_to_this_frame(self.radar_current_frame_num)

    def radar_select_radiobutton_event(self) :
        self.set_to_this_frame(self.radar_current_frame_num)

    def radar_previous_frame_btn_event(self) :
        self.radar_current_frame_num -= 1
        self.radar_video_frame_offset += 1
        self.set_to_this_frame(self.radar_current_frame_num)

    def radar_next_frame_btn_event(self) :
        self.radar_current_frame_num += 1
        self.radar_video_frame_offset -= 1
        self.set_to_this_frame(self.radar_current_frame_num)
        
    def video_previous_frame_btn_event(self) :
        # self.radar_current_frame_num += 1
        self.radar_video_frame_offset -= 1
        self.set_to_this_frame(self.radar_current_frame_num)
        
    def video_next_frame_btn_event(self) :
        self.radar_video_frame_offset += 1
        self.set_to_this_frame(self.radar_current_frame_num)

    def play_btn_event(self) :
        if self.play_toggle_flag == 0 :
            self.play_toggle_flag = 1
            self.play_btn.setText("■")
            x = playing_thread(self)
            x.start()
        else : 
            self.play_toggle_flag = 0
            self.play_btn.setText("▶")

    def simulation_btn_event(self) :
        importlib.reload(SAM)
        if (self.radar1_simulation_checkbox.isChecked() | self.radar2_simulation_checkbox.isChecked()) :
            if self.no_simulation_data_flag == 0 :
                answer = QMessageBox.warning(self, "Overwrite Warning", "Simulation result exist. \nOverwrite it?",QMessageBox.StandardButton.Ok|QMessageBox.StandardButton.Cancel)
                if answer == QMessageBox.StandardButton.Cancel :
                    return
            self.simulation_save_file = open(self.simulation_file_name,'wb')
        #try :
        if (self.radar1_simulation_checkbox.isChecked() & self.radar1_simulation_checkbox.isEnabled()) :
            self.simulation_btn.setVisible(False)
            self.simulation_progress_bar.setVisible(True)
            self.simulation_progress_bar.setRange(0,self.radar_end_frame_num)
            simulated_track_data = DT.genTracker()
            for ii in range(self.radar_end_frame_num) :
                self.get_original_radar_data_in_frame(ii,IT_IS_SIMULATION,RADAR1)
                simulated_object_data = SAM.radar_algorithm_simulation(self.original_object_list, simulated_track_data, ii, self.can_data)
                self.save_simulation_data(simulated_object_data, simulated_track_data, ii, RADAR1)
                self.simulation_progress_bar.setValue(ii)
            self.simulation_btn.setVisible(True)
            self.simulation_progress_bar.setVisible(False)
        if (self.radar2_simulation_checkbox.isChecked() & self.radar2_simulation_checkbox.isEnabled()) :                
            self.simulation_btn.setVisible(False)
            self.simulation_progress_bar.setVisible(True)
            self.simulation_progress_bar.setRange(0,self.radar_end_frame_num)
            simulated_track_data = DT.genTracker()
            for ii in range(self.radar_end_frame_num) :
                self.get_original_radar_data_in_frame(ii,IT_IS_SIMULATION,RADAR2)
                simulated_object_data = SAM.radar_algorithm_simulation(self.original_object_list, simulated_track_data, ii, self.can_data)
                self.save_simulation_data(simulated_object_data, simulated_track_data, ii, RADAR2)
                self.simulation_progress_bar.setValue(ii)
            self.simulation_btn.setVisible(True)
            self.simulation_progress_bar.setVisible(False)
        if (self.radar1_simulation_checkbox.isChecked() | self.radar2_simulation_checkbox.isChecked()) :
            self.no_simulation_data_flag = 0
            self.simulation_save_file.close()
        # except :
        #     self.simulation_btn.setVisible(True)
        #     self.simulation_progress_bar.setVisible(False)
        self.load_btn_event()

    def view_reset_shortcut_event(self) :
        self.original_radar_plot.setRange(QRectF(GRAPH_MIN_X, GRAPH_MIN_Y, GRAPH_MAX_X-GRAPH_MIN_X, GRAPH_MAX_Y-GRAPH_MIN_Y),disableAutoRange = True)
        self.simulated_radar_plot.setRange(QRectF(GRAPH_MIN_X, GRAPH_MIN_Y, GRAPH_MAX_X-GRAPH_MIN_X, GRAPH_MAX_Y-GRAPH_MIN_Y),disableAutoRange = True)

    def checkbox_change_event(self) :
        self.set_to_this_frame(self.radar_current_frame_num)

    def jump_scale1_sel_event(self) : 
        self.fast_speed_combobox.setCurrentIndex(0)
    def jump_scale2_sel_event(self) : 
        self.fast_speed_combobox.setCurrentIndex(1)
    def jump_scale3_sel_event(self) : 
        self.fast_speed_combobox.setCurrentIndex(2)
    def jump_scale4_sel_event(self) : 
        self.fast_speed_combobox.setCurrentIndex(3)
    def reset_frame_curser_event(self) :
        self.radar_current_frame_num = 0
        self.set_to_this_frame(self.radar_current_frame_num)
    ########################################## Button Call-back End #######################################################



    #####################################################################################
    #   Class - App ==> Internal Method Functions                                       #
    #####################################################################################
    def read_points(self,plot,points,event) :
        point_position = points[0].pos()
        data_type = points[0].data()
        point_index = points[0].index()
        if plot == self.original_scatter :
            if data_type == 1 :                              # object data
                attribute_list = radar_object.get_attributes_list()
                self.original_graph_table.setRowCount(len(attribute_list))
                self.original_graph_table.clear()
                for ii in range(len(attribute_list)) :
                    self.original_graph_table.setItem(ii,0,QTableWidgetItem(attribute_list[ii]))
                    exec('self.original_graph_table.setItem(ii,1,QTableWidgetItem(str(round(self.original_object_list[point_index].%s,5))))' % attribute_list[ii])

            if data_type == 2 :                                 # track data
                attribute_list = radar_track.get_attributes_list()
                point_index = point_index - len(self.original_object_list)   # point index is acculuated
                self.original_graph_table.setRowCount(len(attribute_list))
                self.original_graph_table.clear()
                for ii in range(len(attribute_list)) :
                    self.original_graph_table.setItem(ii,0,QTableWidgetItem(attribute_list[ii]))
                    exec('self.original_graph_table.setItem(ii,1,QTableWidgetItem(str(round(self.original_track_list[point_index].%s,5))))' % attribute_list[ii])
            self.original_graph_info_window.show()
       
            
        if plot == self.simulated_scatter :
            if data_type == 1 : # object data
                attribute_list = simulated_object.get_attributes_list()
                self.simulated_graph_table.setRowCount(len(attribute_list))
                self.simulated_graph_table.clear()
                for ii in range(len(attribute_list)) :
                    self.simulated_graph_table.setItem(ii,0,QTableWidgetItem(attribute_list[ii]))
                    exec('self.simulated_graph_table.setItem(ii,1,QTableWidgetItem(str(round(self.simulated_object_list[point_index].%s,5))))' % attribute_list[ii])

            if data_type == 2 : # track data
                attribute_list = simulated_track.get_attributes_list()
                point_index = point_index - len(self.simulated_object_list)   # point index is acculuated
                self.simulated_graph_table .setRowCount(len(attribute_list))
                self.simulated_graph_table .clear()
                for ii in range(len(attribute_list)) :
                    self.simulated_graph_table.setItem(ii,0,QTableWidgetItem(attribute_list[ii]))
                    exec('self.simulated_graph_table.setItem(ii,1,QTableWidgetItem(str(round(self.simulated_track_list[point_index].%s,5))))' % attribute_list[ii])
            self.simulated_graph_info_window.show()
            

        
    def get_original_radar_data_in_frame(self, frame_index, simulation_flag, num_radar) :
        if simulation_flag :
            if num_radar == 1 :     # 1 means radar1
                self.original_radar_file_handle.seek(self.origianl_radar1_frame_index[frame_index])         # find specific frame
            elif num_radar == 2 :   # 2 means radar2
                self.original_radar_file_handle.seek(self.origianl_radar2_frame_index[frame_index])         # find specific frame
        else :
            if self.radar1_select_radiobutton.isChecked() :
                self.original_radar_file_handle.seek(self.origianl_radar1_frame_index[frame_index])         # find specific frame
            elif self.radar2_select_radiobutton.isChecked() :
                self.original_radar_file_handle.seek(self.origianl_radar2_frame_index[frame_index])         # find specific frame
        data_type = self.original_radar_file_handle.read(1)
        data_length_bin = self.original_radar_file_handle.read(2)
        data_length = np.frombuffer(data_length_bin, dtype=np.uint8)
        payload_len = data_length[1]*256+data_length[0]+1       # data length doesn't have end marker byte. so, we have to plus 1 byte to get the frame end marker
        data_payload = self.original_radar_file_handle.read(payload_len)
        data_payload_uint = np.frombuffer(data_payload,dtype=np.uint8)
        if data_payload_uint[payload_len-1] != 95 :
            self.general_error_window("RADAR FILE ERROR","Frame end marker is missing.")
            self.error_flag = 1
            return
        track_frame_number = data_payload_uint[3]*16777216+data_payload_uint[2]*65535+data_payload_uint[1]*256+data_payload_uint[0] # uint32 frame_number
        cpu_cycle_time = data_payload_uint[7]*16777216+data_payload_uint[6]*65535+data_payload_uint[5]*256+data_payload_uint[4] # uint32 timecpucycles
        num_object = data_payload_uint[11]*16777216+data_payload_uint[10]*65535+data_payload_uint[9]*256+data_payload_uint[8] # uint32 timecpucycles
        num_track = data_payload_uint[15]*16777216+data_payload_uint[14]*65535+data_payload_uint[13]*256+data_payload_uint[12] # uint32 timecpucycles
        sub_frame_num = data_payload_uint[19]*16777216+data_payload_uint[18]*65535+data_payload_uint[17]*256+data_payload_uint[16] # uint32 timecpucycles
        print(track_frame_number,num_object)
        ii=20


        if data_payload_uint[13] == 0 :     # number of tracks should be less than 100(by the setting), so, if data_payload_uint[13] is larger than 0, it is error
            self.original_object_list = []
            for jj in range(num_object) :
                obj = radar_object()
                read_data = data_payload[ii:(ii+34)]
                if len(read_data) == 34 :
                    obj.range_idx                  = (read_data[ 1]*256 + read_data[ 0])    # is np.frombuffer faster than calculation?
                    obj.doppler_idx                = (read_data[ 3]*256 + read_data[ 2])
                    obj.range                      = radar_object.uint2int_16(read_data[ 5]*256 + read_data[ 4])/Q7_DIVISOR
                    obj.speed                      = radar_object.uint2int_16(read_data[ 7]*256 + read_data[ 6])/Q7_DIVISOR
                    obj.sin_azim                   = radar_object.uint2int_16(read_data[ 9]*256 + read_data[ 8])/Q14_DIVISOR
                    obj.peak_val                   = (read_data[11]*256 + read_data[10])/Q8_DIVISOR
                    obj.range_snr_db               = (read_data[13]*256 + read_data[12])/Q8_DIVISOR
                    obj.doppler_snr_db             = (read_data[15]*256 + read_data[14])/Q8_DIVISOR
                    obj.sin_azim_srn_lin           = (read_data[17]*256 + read_data[16])/Q8_DIVISOR
                    obj.x                          = radar_object.uint2int_16(read_data[19]*256 + read_data[18])/Q7_DIVISOR
                    obj.y                          = radar_object.uint2int_16(read_data[21]*256 + read_data[20])/Q7_DIVISOR
                    obj.z                          = radar_object.uint2int_16(read_data[23]*256 + read_data[22])/Q7_DIVISOR
                    obj.rotate_x                   = radar_object.uint2int_16(read_data[25]*256 + read_data[24])/Q7_DIVISOR
                    obj.rotate_y                   = radar_object.uint2int_16(read_data[27]*256 + read_data[26])/Q7_DIVISOR
                    obj.vel_disamb_fac_valid       = (read_data[29]*256 + read_data[28])
                    obj.vel_disamb_fac_valid_als   = (read_data[31]*256 + read_data[30])
                    obj.status_flag                = (read_data[33]*256 + read_data[32])
                    self.original_object_list.append(obj)
                ii = ii+34

            self.original_track_list = []
            for jj in range(num_track) :
                trk = radar_track()
                read_data = data_payload[ii:(ii+22)]
                if len(read_data) == 22 :
                    trk.id                         = (read_data[ 1]*256 + read_data[ 0])
                    trk.x                          = radar_object.uint2int_16((read_data[ 3]*256 + read_data[ 2]))/Q7_DIVISOR
                    trk.y                          = radar_object.uint2int_16((read_data[ 5]*256 + read_data[ 4]))/Q7_DIVISOR
                    trk.xd                         = radar_object.uint2int_16((read_data[ 7]*256 + read_data[ 6]))/Q7_DIVISOR
                    trk.yd                         = radar_object.uint2int_16((read_data[ 9]*256 + read_data[ 8]))/Q7_DIVISOR
                    trk.x_size                     = (read_data[11]*256 + read_data[10])
                    trk.y_size                     = (read_data[13]*256 + read_data[12])
                    trk.tick                       = (read_data[15]*256 + read_data[14])
                    trk.age                        = (read_data[17]*256 + read_data[16])
                    trk.flag                       = (read_data[19]*256 + read_data[18])
                    trk.reserved0                  = (read_data[21]*256 + read_data[20])
                    self.original_track_list.append(trk)
                ii = ii+22

            self.can_data = []
            read_data = np.frombuffer(data_payload[ii:(ii+16)],dtype=np.uint8)
            if len(read_data) == 16 :  
                for ii in range(16) :
                    self.can_data.append(read_data[ii])
            vehicle_steer_angle = (self.can_data[14]*256 + self.can_data[15])/10   # Steer Angle(deg)
            if vehicle_steer_angle > 3276.7 :
                vehicle_steer_angle -= 6553.5
            vehicle_steer_angle = vehicle_steer_angle * 38.0/650.0
            print(self.can_data, round(vehicle_steer_angle, 1))

    def get_simulated_radar_data_in_frame(self, frame_index) :
        simulated_data_flag = 0
        self.simulated_object_list = []
        self.simulated_track_list = []
        if self.radar1_select_radiobutton.isChecked() :
            if self.simulated_radar1_frame_index != [] :
                self.simulated_radar_file_handle.seek(self.simulated_radar1_frame_index[frame_index])         # find specific frame
                simulated_data_flag = 1
        elif self.radar2_select_radiobutton.isChecked() :
            if self.simulated_radar2_frame_index != [] :
                self.simulated_radar_file_handle.seek(self.simulated_radar2_frame_index[frame_index])         # find specific frame
                simulated_data_flag = 1
        if simulated_data_flag == 1 :
            data_type = self.simulated_radar_file_handle.read(1)
            data_length_bin = self.simulated_radar_file_handle.read(2)
            data_length = np.frombuffer(data_length_bin, dtype=np.uint8)
            payload_len = data_length[1]*256+data_length[0]+1       # data length doesn't have end marker byte. so, we have to plus 1 byte to get the frame end marker
            data_payload = self.simulated_radar_file_handle.read(payload_len)
            data_payload_uint = np.frombuffer(data_payload,dtype=np.uint8)
            if data_payload_uint[payload_len-1] != 95 :
                self.general_error_window("SIMULATED DATA FILE ERROR","Frame end marker is missing.")
                self.error_flag = 1
                return
            track_frame_number = data_payload_uint[3]*16777216+data_payload_uint[2]*65535+data_payload_uint[1]*256+data_payload_uint[0]     # uint32 frame_number
            cpu_cycle_time = data_payload_uint[7]*16777216+data_payload_uint[6]*65535+data_payload_uint[5]*256+data_payload_uint[4]         # uint32 timecpucycles
            num_object = data_payload_uint[11]*16777216+data_payload_uint[10]*65535+data_payload_uint[9]*256+data_payload_uint[8]           # uint32 timecpucycles
            num_track = data_payload_uint[15]*16777216+data_payload_uint[14]*65535+data_payload_uint[13]*256+data_payload_uint[12]          # uint32 timecpucycles
            sub_frame_num = data_payload_uint[19]*16777216+data_payload_uint[18]*65535+data_payload_uint[17]*256+data_payload_uint[16]      # uint32 timecpucycles
            ii=20
            if (frame_index != track_frame_number) :
                print("simulation frame is wierd")
            
            if data_payload_uint[13] == 0 :
                for jj in range(num_object) :
                    obj = simulated_object()
                    read_data = data_payload[ii:(ii+34)]
                    if len(read_data) == 34 :
                        obj.detID                       = (read_data[ 1]*256 + read_data[ 0])
                        obj.clusterId                   = (read_data[ 3]*256 + read_data[ 2])
                        obj.rangeIdx                    = (read_data[ 5]*256 + read_data[ 4])
                        obj.dopplerIdx                  = (read_data[ 7]*256 + read_data[ 6])
                        obj.range                       = radar_object.uint2int_16((read_data[ 9]*256 + read_data[ 8]))/Q7_DIVISOR
                        obj.speed                       = radar_object.uint2int_16((read_data[11]*256 + read_data[10]))/Q7_DIVISOR
                        obj.sinAzim                     = radar_object.uint2int_16((read_data[13]*256 + read_data[12]))/Q14_DIVISOR
                        obj.rangeVal                    = radar_object.uint2int_16((read_data[15]*256 + read_data[14]))/Q7_DIVISOR
                        obj.rangeSNR                    = (read_data[17]*256 + read_data[16])/Q8_DIVISOR
                        obj.dopplerSNR                  = (read_data[19]*256 + read_data[18])/Q8_DIVISOR
                        obj.angleSNR                    = (read_data[21]*256 + read_data[20])/Q8_DIVISOR
                        obj.x                           = radar_object.uint2int_16((read_data[23]*256 + read_data[22]))/Q7_DIVISOR
                        obj.y                           = radar_object.uint2int_16((read_data[25]*256 + read_data[24]))/Q7_DIVISOR
                        obj.velDisambFacValid           = (read_data[27]*256 + read_data[26])
                        obj.status_flag                 = (read_data[29]*256 + read_data[28])
                        obj.xd                          = radar_object.uint2int_16((read_data[31]*256 + read_data[30]))/Q7_DIVISOR
                        obj.yd                          = radar_object.uint2int_16((read_data[33]*256 + read_data[32]))/Q7_DIVISOR
                        self.simulated_object_list.append(obj)
                    ii = ii+34
    
                for jj in range(num_track) :
                    trk = simulated_track()
                    read_data = data_payload[ii:(ii+44)]
                    if len(read_data) == 44 :
                        trk.ID                          = (read_data[ 1]*256 + read_data[ 0])
                        trk.SNR                         = (read_data[ 3]*256 + read_data[ 2])/Q8_DIVISOR
                        trk.age                         = (read_data[ 5]*256 + read_data[ 4])
                        trk.associ_ID                   = radar_object.uint2int_16(read_data[ 7]*256 + read_data[ 6])
                        trk.dopplerSNR                  = (read_data[ 9]*256 + read_data[ 8])/Q8_DIVISOR
                        trk.measVectorRRD_R             = radar_object.uint2int_16((read_data[11]*256 + read_data[10]))/Q7_DIVISOR
                        trk.measVectorRRD_vel           = radar_object.uint2int_16((read_data[13]*256 + read_data[12]))/Q7_DIVISOR
                        trk.measVectorRRD_Theta         = radar_object.uint2int_16((read_data[15]*256 + read_data[14]))/Q7_DIVISOR
                        trk.statVecXYZ_x                = radar_object.uint2int_16((read_data[17]*256 + read_data[16]))/Q7_DIVISOR
                        trk.statVecXYZ_y                = radar_object.uint2int_16((read_data[19]*256 + read_data[18]))/Q7_DIVISOR
                        trk.statVecXYZ_xd               = radar_object.uint2int_16((read_data[21]*256 + read_data[20]))/Q7_DIVISOR
                        trk.statVecXYZ_yd               = radar_object.uint2int_16((read_data[23]*256 + read_data[22]))/Q7_DIVISOR
                        trk.peakVal                     = (read_data[25]*256 + read_data[24])/Q8_DIVISOR
                        trk.prevXd                      = radar_object.uint2int_16((read_data[27]*256 + read_data[26]))/Q7_DIVISOR
                        trk.prevYd                      = radar_object.uint2int_16((read_data[29]*256 + read_data[28]))/Q7_DIVISOR
                        trk.rangeSNR                    = (read_data[31]*256 + read_data[30])/Q8_DIVISOR
                        trk.tick                        = (read_data[33]*256 + read_data[32])
                        trk.xSize                       = radar_object.uint2int_16((read_data[35]*256 + read_data[34]))/Q7_DIVISOR
                        trk.ySize                       = radar_object.uint2int_16((read_data[37]*256 + read_data[36]))/Q7_DIVISOR
                        trk.plotValidity                = (read_data[39]*256 + read_data[38])
                        trk.Status_Flag0                = (read_data[41]*256 + read_data[40])
                        trk.Status_Flag1                = (read_data[43]*256 + read_data[42])
                        self.simulated_track_list.append(trk)
                    ii = ii+44
                ii=ii+16        # this is for CAN data


    def original_graph_update(self, obj, trk) :
        obj_spots=[]
        trk_spots=[]
        for ii in range(len(obj)) :
            obj_spots.append({'pos':[obj[ii].x, obj[ii].y],'data': 1})
        for ii in range(len(trk)) :
            trk_spots.append({'pos':[trk[ii].x, trk[ii].y],'data': 2})
        self.original_scatter.clear()
        if self.original_object_checkbox.isChecked() :
            self.original_scatter.addPoints(obj_spots, brush=(255,0,0,255))
        if self.original_track_checkbox.isChecked() :
            self.original_scatter.addPoints(trk_spots, brush=(255,255,255,255))
        # print(speed_num, speed_als_num, len(obj), speed_num/len(obj), speed_als_num/len(obj) )
    
    def simulated_graph_update(self, obj, trk) :
        obj_spots=[]
        trk_spots=[]
        trk_candidate_spots=[]
        for ii in range(len(obj)) :
            obj_spots.append({'pos':[obj[ii].x, obj[ii].y],'data': 1})
        for ii in range(len(trk)) :
            if trk[ii].plotValidity :
                trk_spots.append({'pos':[trk[ii].statVecXYZ_x, trk[ii].statVecXYZ_y],'data': 2})
            else :
                trk_candidate_spots.append({'pos':[trk[ii].statVecXYZ_x, trk[ii].statVecXYZ_y],'data': 2})
        self.simulated_scatter.clear()
        if self.simulated_object_checkbox.isChecked() :
            self.simulated_scatter.addPoints(obj_spots, brush=(255,0,0,255))
        if self.simulated_track_checkbox.isChecked() :
            self.simulated_scatter.addPoints(trk_spots, brush=(255,255,255,255))
        if self.simulated_candidate_checkbox.isChecked() :
            self.simulated_scatter.addPoints(trk_candidate_spots, brush=(0,255,0,255))
        

    def enable_all_components(self, onoff) :
        self.previous_frame_btn.setEnabled(onoff)
        self.play_btn.setEnabled(onoff)                   # under-construction
        self.forward_frame_btn.setEnabled(onoff)
        self.fast_rewind_btn.setEnabled(onoff)
        self.fast_forward_btn.setEnabled(onoff)
        self.frame_number_line_edit.setEnabled(onoff)
        self.frame_number_jump_btn.setEnabled(onoff)
        self.fast_speed_combobox.setEnabled(onoff)
        self.radar_previous_frame_btn.setEnabled(onoff)
        self.radar_next_frame_btn.setEnabled(onoff)
        self.radar_frame_slider.setEnabled(False)
        # self.radar_frame_slider.setEnabled(onoff)         # sliders are not enable forever......
        self.video_previous_frame_btn.setEnabled(onoff)
        self.video_next_frame_btn.setEnabled(onoff)
        self.video_frame_slider.setEnabled(False)
        # self.video_frame_slider.setEnabled(onoff)         # sliders are not enable forever......
        self.radar1_select_radiobutton.setEnabled(onoff)
        self.radar2_select_radiobutton.setEnabled(onoff)
        self.original_radar_label.setEnabled(onoff)
        self.simulateed_radar_label.setEnabled(onoff)
        self.original_object_checkbox.setEnabled(onoff)
        self.original_track_checkbox.setEnabled(onoff)
        self.simulated_object_checkbox.setEnabled(onoff)
        self.simulated_track_checkbox.setEnabled(onoff)
        self.radar1_simulation_checkbox.setEnabled(onoff)
        self.radar2_simulation_checkbox.setEnabled(onoff)
        self.simulation_btn.setEnabled(onoff)
        self.simulated_candidate_checkbox.setEnabled(onoff)
        self.video12_flip.setEnabled(onoff)
        # self.simulated_pruned_checkbox.setEnabled(onoff)

    def component_enable_check_frame(self) :
        if self.radar_current_frame_num <= 0 : 
            self.previous_frame_btn.setEnabled(False)
            self.fast_rewind_btn.setEnabled(False)
            self.radar_previous_frame_btn.setEnabled(False)
        else :
            self.previous_frame_btn.setEnabled(True)
            self.fast_rewind_btn.setEnabled(True)
            self.radar_previous_frame_btn.setEnabled(True)
        
        if self.radar_current_frame_num >= self.radar_end_frame_num-1 :
            self.forward_frame_btn.setEnabled(False)
            self.fast_forward_btn.setEnabled(False)
            self.radar_next_frame_btn.setEnabled(False)
        else : 
            self.forward_frame_btn.setEnabled(True)
            self.fast_forward_btn.setEnabled(True)
            self.radar_next_frame_btn.setEnabled(True)

        if self.video_current_frame_num <= 0 :
            self.video_previous_frame_btn.setEnabled(False)
        else :
            self.video_previous_frame_btn.setEnabled(True)

        if self.video_current_frame_num >= self.video_end_frame_num-1 :
            self.video_next_frame_btn.setEnabled(False)
        else :
            self.video_next_frame_btn.setEnabled(True)

    def component_enable_check_load(self) :
        if self.radar1_end_frame_num == 0 :
            self.radar1_select_radiobutton.setEnabled(False)
            self.radar1_simulation_checkbox.setEnabled(False)
        else : 
            self.radar1_select_radiobutton.setEnabled(True)
            self.radar1_simulation_checkbox.setEnabled(True)
        
        if self.radar2_end_frame_num == 0 :
            self.radar2_select_radiobutton.setEnabled(False)
            self.radar2_simulation_checkbox.setEnabled(False)
        else :
            self.radar2_select_radiobutton.setEnabled(True)
            self.radar2_simulation_checkbox.setEnabled(True)
        
        self.play_btn.setEnabled(True)            # under-construction
        self.frame_number_line_edit.setEnabled(True)
        self.frame_number_jump_btn.setEnabled(True)
        self.fast_speed_combobox.setEnabled(True)

        self.original_radar_label.setEnabled(True)
        self.simulateed_radar_label.setEnabled(True)
        self.original_object_checkbox.setEnabled(True)
        self.original_track_checkbox.setEnabled(True)
        self.simulated_object_checkbox.setEnabled(True)
        self.simulated_track_checkbox.setEnabled(True)
        self.simulated_candidate_checkbox.setEnabled(True)
        # self.simulated_pruned_checkbox.setEnabled(True)
        self.simulation_btn.setEnabled(True)
        self.video12_flip.setEnabled(True)
        self.radar_frame_slider.setEnabled(False)   # never enable.
        self.video_frame_slider.setEnabled(False)   # never enable.

    def set_to_this_frame(self, num_frame) :
        self.radar_current_frame_num = num_frame
        self.video_current_frame_num = num_frame + self.radar_video_frame_offset
        self.frame_update()
        
    def update_video_frame(self, num_frame) :
        ret = 0
        if (self.no_video_flag | self.no_video2_flag) == 0 :
            if self.video12_flip.isChecked() == 0 : 
                if self.radar1_select_radiobutton.isChecked() :
                    self.video_handle.set(cv2.CAP_PROP_POS_FRAMES,num_frame)
                    ret, image = self.video_handle.read()
                    image = cv2.flip(image,-1)
                elif self.radar2_select_radiobutton.isChecked() :
                    self.video2_handle.set(cv2.CAP_PROP_POS_FRAMES,num_frame)
                    ret, image = self.video2_handle.read()
                    
            else :
                if self.radar1_select_radiobutton.isChecked() :
                    self.video2_handle.set(cv2.CAP_PROP_POS_FRAMES,num_frame)
                    ret, image = self.video2_handle.read()
                elif self.radar2_select_radiobutton.isChecked() :
                    self.video_handle.set(cv2.CAP_PROP_POS_FRAMES,num_frame)
                    ret, image = self.video_handle.read()
                    image = cv2.flip(image,-1)
        elif self.no_video_flag == 0 :
            self.video_handle.set(cv2.CAP_PROP_POS_FRAMES,num_frame)
            ret, image = self.video_handle.read()
        elif self.no_video2_flag == 0 :
            self.video2_handle.set(cv2.CAP_PROP_POS_FRAMES,num_frame)
            ret, image = self.video2_handle.read()

        if ret : 
            resize_image = cv2.resize(image,(380, 285))
            height, width, bytevalue = resize_image.shape
            bytevalue = bytevalue * width
            cv2.cvtColor(resize_image,cv2.COLOR_BGR2RGB,resize_image)
            # self.image_from_video = QImage(image,380, 285,bytevalue,QImage.Format.Format_RGB888)
            self.image_from_video = QImage(resize_image,width,height,bytevalue,QImage.Format.Format_RGB888)
            self.pixmap.convertFromImage(self.image_from_video)
            self.image_lable.setPixmap(self.pixmap)
        else :
            # self.general_warning_window("NO VIDEO", "Video file is unavailable. Check the file.")
            self.no_video_flag = 1

    def frame_update(self) :
        self.componenet_update()
        self.get_original_radar_data_in_frame(self.radar_current_frame_num, IT_IS_NOT_SIMULATION, 0)       # simulation_flag = 0, radar_num = 0
        self.original_graph_update(self.original_object_list, self.original_track_list)
        if self.no_simulation_data_flag == 0 :
            self.get_simulated_radar_data_in_frame(self.radar_current_frame_num)
            self.simulated_graph_update(self.simulated_object_list, self.simulated_track_list)
        if (self.video_current_frame_num >= self.video_end_frame_num) :
            self.update_video_frame(self.video_end_frame_num-1)
        elif (self.video_current_frame_num < 0) :
            self.update_video_frame(0)
        else :
            self.update_video_frame(self.video_current_frame_num)              

    def componenet_update(self) :
        self.component_enable_check_frame()
        self.radar_frame_info_label.setText(str(self.radar_current_frame_num)+"/"+str(self.radar_end_frame_num))
        self.frame_number_line_edit.setText(str(self.radar_current_frame_num))
        self.radar_frame_slider.setValue(self.radar_current_frame_num)
        if self.no_video_flag & self.no_video2_flag == 0 :
            self.video_frame_info_label.setText(str(self.video_current_frame_num)+"/"+str(self.video_end_frame_num))
            self.video_frame_slider.setValue(self.video_current_frame_num)


    def general_error_window(self, title, msg) :
        QMessageBox.critical(self, title, msg)

    def general_warning_window(self, title, msg) :
        QMessageBox.warning(self, title, msg)


    def save_simulation_data(self, sim_object, sim_target, num_frame, radar_num) :
        sim_object_save_data = []
        sim_track_save_data = []
        for ii in range(len(sim_object)) :
            sim_object_save_data.append(round((sim_object[ii].detID),0))
            sim_object_save_data.append(round((sim_object[ii].clusterId),0))
            sim_object_save_data.append(round((sim_object[ii].rangeIdx),0))
            sim_object_save_data.append(round((sim_object[ii].dopplerIdx),0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].range)*Q7_DIVISOR,0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].speed)*Q7_DIVISOR,0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].sinAzim)*Q14_DIVISOR,0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].rangeVal)*Q7_DIVISOR,0))
            sim_object_save_data.append(round((sim_object[ii].rangeSNR)*Q8_DIVISOR,0))
            sim_object_save_data.append(round((sim_object[ii].dopplerSNR)*Q8_DIVISOR,0))
            sim_object_save_data.append(round((sim_object[ii].angleSNR)*Q8_DIVISOR,0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].x)*Q7_DIVISOR,0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].y)*Q7_DIVISOR,0))
            sim_object_save_data.append(round((sim_object[ii].velDisambFacValidity),0))
            sim_object_save_data.append(round((sim_object[ii].statusFlag),0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].xd)*Q7_DIVISOR,0))
            sim_object_save_data.append(round(radar_object.int2uint_16(sim_object[ii].yd)*Q7_DIVISOR,0))
        
        for ii in range(len(sim_target.List)) : ## these attributes in sim_target.List[] are from dataStructure.py
            sim_track_save_data.append(round((sim_target.List[ii].ID),0))
            sim_track_save_data.append(round((sim_target.List[ii].SNR)*Q8_DIVISOR,0))
            sim_track_save_data.append(round((sim_target.List[ii].age),0))
            try : 
                if sim_target.List[ii].associatedObj.ID == None :
                    sim_track_save_data.append(-1)
                else :
                    sim_track_save_data.append(sim_target.List[ii].associatedObj.ID)
            except : 
                sim_track_save_data.append(-1)
            sim_track_save_data.append(round((sim_target.List[ii].dopplerSNR)*Q8_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].measVectorRRD[0])*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].measVectorRRD[1])*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].measVectorRRD[2])*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].stateVectorXYZ[0])*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].stateVectorXYZ[1])*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].stateVectorXYZ[2])*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].stateVectorXYZ[3])*Q7_DIVISOR,0))
            sim_track_save_data.append(round((sim_target.List[ii].peakVal)*Q8_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].prevXd)*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].prevYd)*Q7_DIVISOR,0))
            sim_track_save_data.append(round((sim_target.List[ii].rangeSNR)*Q8_DIVISOR,0))
            sim_track_save_data.append(round((sim_target.List[ii].tick),0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].xSize)*Q7_DIVISOR,0))
            sim_track_save_data.append(round(radar_object.int2uint_16(sim_target.List[ii].ySize)*Q7_DIVISOR,0))
            sim_track_save_data.append(round((sim_target.List[ii].plotValidity),0))
            sim_track_save_data.append(round((sim_target.List[ii].Status_Flag0),0))
            sim_track_save_data.append(round((sim_target.List[ii].Status_Flag1),0))
        sim_save_data_content = sim_object_save_data + sim_track_save_data
        
        if radar_num == 1 :
            header = b'RSIM_RPAS1' + np.ndarray.tobytes(np.array(0,dtype = np.uint8)) + np.ndarray.tobytes(np.array(len(sim_save_data_content)*2+20+16,dtype = np.uint16))
        else :
            header = b'RSIM_RPAS2' + np.ndarray.tobytes(np.array(0,dtype = np.uint8)) + np.ndarray.tobytes(np.array(len(sim_save_data_content)*2+20+16,dtype = np.uint16))
        data_header = [num_frame, 0, len(sim_object),len(sim_target.List),0]    # [Frame_number, reserved0, Num_object, Num_track, reserved1]
        simulation_save_data_bin = header + np.ndarray.tobytes(np.array(data_header,dtype = np.uint32)) + \
            np.ndarray.tobytes(np.array(sim_save_data_content,dtype = np.uint16)) + np.ndarray.tobytes(np.array(self.can_data,dtype = np.uint8)) + np.ndarray.tobytes(np.array(95,dtype = np.uint8))
        pickle.dump(simulation_save_data_bin,self.simulation_save_file)

        
###################################### End of Internal method Function #####################################

#####################################################################################
#   Class - Thread ==> Playing frames                                               #
#####################################################################################
class playing_thread(QThread) :
    def __init__(self, parent) :
        super().__init__(parent)
        self.parent = parent

    def run(self) :
        while(self.parent.play_toggle_flag) :
            if self.parent.radar_current_frame_num < self.parent.radar_end_frame_num-1 :
                self.parent.forward_frame_btn.clicked.emit()
                self.msleep(66)
            else : 
                self.parent.play_btn.clicked.emit()
                break



###################################### End of Class Declaration ############################################
    

if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    my_window = App()
    my_window.show()
    app.exec()


