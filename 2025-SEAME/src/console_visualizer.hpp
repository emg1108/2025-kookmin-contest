#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "lane_detector.hpp"
#include "obstacle_detector.hpp"
#include "control.hpp"

class ConsoleVisualizer {
public:
    ConsoleVisualizer(int width = 80, int height = 20);
    
    // 차선 정보 시각화
    void visualizeLane(int offset, const cv::Mat& frame);
    
    // 장애물 정보 시각화
    void visualizeObstacle(const std::vector<cv::Point2f>& obstacles);
    
    // 제어 상태 시각화
    void visualizeControl(float steering, float throttle, const std::string& state);
    
    // 화면 클리어
    void clearScreen();
    
    // 전체 시각화
    void visualizeAll(int lane_offset, float steering, float throttle, 
                     const std::string& state, const std::vector<cv::Point2f>& obstacles);

private:
    int console_width_;
    int console_height_;
    std::vector<std::string> screen_buffer_;
    
    void drawRoad();
    void drawCar();
    void drawObstacles(const std::vector<cv::Point2f>& obstacles);
    void drawInfo(float steering, float throttle, const std::string& state);
    void render();
}; 