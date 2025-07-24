// usb_cam.cpp (리팩터링: 클래스 기반, main 제거)
#include "usb_cam.hpp"
#include "constants.hpp"

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

bool USBCam::init() {
    // 실제 USB 카메라 사용
    cap.open(0); // 0번 카메라 (또는 /dev/video0)
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 카메라 열기 실패" << std::endl;
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    std::cout << "[INFO] 실제 카메라 모드로 초기화" << std::endl;
    return true;
}

cv::Mat USBCam::getFrame() {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "[ERROR] 카메라 프레임 없음" << std::endl;
    }
    return frame;
}
