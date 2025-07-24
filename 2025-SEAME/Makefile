CXX = g++
CXXFLAGS = -std=c++17 -Wall -fPIC \
           -Iinclude -I/usr/include \
           `pkg-config --cflags opencv4`

LDFLAGS = `pkg-config --libs opencv4` -lrt -pthread

SRC_DIR = src
OBJ_DIR = build
OUT = auto_drive

SRCS = $(SRC_DIR)/main.cpp \
       $(SRC_DIR)/usb_cam.cpp \
       $(SRC_DIR)/video_recorder.cpp \
       $(SRC_DIR)/lane_detector.cpp \
       $(SRC_DIR)/obstacle_detector.cpp \
       $(SRC_DIR)/track_detector.cpp \
       $(SRC_DIR)/traffic_light_detector.cpp \
       $(SRC_DIR)/control.cpp \
       $(SRC_DIR)/constants.cpp \
       $(SRC_DIR)/console_visualizer.cpp

# 오브젝트 파일 리스트
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))

# 전체 빌드
all: $(OUT)

# 실행파일 빌드
$(OUT): $(OBJS)
	@echo "🚗 Linking $@"
	@$(CXX) $(CXXFLAGS) -Wl,-rpath=. -o $@ $^ $(LDFLAGS)

# .o 파일 빌드 규칙
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# clean
clean:
	@echo "🧹 Cleaning build files"
	@rm -f $(OUT)
	@rm -rf $(OBJ_DIR)
