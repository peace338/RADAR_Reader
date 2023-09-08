
# RADAR Reader

## 1. 프로젝트 개요

본 코드는 Movon RADAR Unit으로부터 취득한 `.rdb` 파일과 video 파일을 처리하는 프로그램이다. 
주요 기능으로는 저장된 데이터를 3차원 공간에 출력하는 것과 `.rdb` 파일의 object data를 통해 perception algorithm을 emulation 하는 것이다.


## 2. 동작 환경

```
python 3.8.10
OS : window

pip install pyqt6==13.5.1
pip install pyqtgraph==0.13.3
pip install numpy==1.22.3
pip install opencv-python==4.7.0.72
pip install scikit-learn==1.3.0
pip install PyOpenGL==3.1.5
...

```

## 3. File Structure

```
RADAR_Reader
├── algorithm
│   ├── dataManager
│   ├── radarConfigure      # RADAR Emulator Configuration
│   ├── radarDetection
│   ├── radarPerception
│   └── algorithmMain.py    # RADAR Emulator algorithm main
├── guiUtils
│   ├── application
│   │   ├── mainApp.py      # main Application
│   │   └── plotApp.py      # plot Application
│   ├── dataParser
│   └──images
├── demo.py                 # main. run this code.
├── radarEquipInfo.py       # equip Configuration
└── README.md
```

## 4. parameter 설정

`./radarEquipInfo.py`에서 radar 장착 정보를 입력한다.

`./algorithm/radarConfigure/configManager_MRR_DEMO.py` 에서 algorithm 관련 parameter 설정이 가능하다.

`./guiUtils/guiConfigure.py`에서 gui 관련 parameter를 조정하게 하려고 했으나 아직 정리가 좀 안되어 있다.

