
# RADAR Reader

---

## 1. 프로젝트 개요

본 코드는 Movon RADAR Unit으로부터 취득한 `.rdb` 파일과 video 파일을 처리하는 프로그램이다. 
주요 기능으로는 저장된 데이터를 3차원 공간에 출력하는 것과 `.rdb` 파일의 object data를 통해 perception algorithm(Perception algorithm)을 emulation 하는 것이다.

---

## 2. 필요한 package

```
pip install pyqt6

pip install pyqtgraph

pip install numpy 

pip install opencv-python

```

버전 호환은 체크하지 못함.

---

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

---

## 4. parameter 설정

`./radarEquipInfo.py`에서 radar 장착 정보를 입력한다.

`./algorithm/radarConfigure/configManager_MRR_DEMO.py` 에서 algorithm 관련 parameter 설정이 가능하다.

`./guiUtils/guiConfigure.py`에서 gui 관련 parameter를 조정하게 하려고 했으나 정리가 좀 안되어 있다.
