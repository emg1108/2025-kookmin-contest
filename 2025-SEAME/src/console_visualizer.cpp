#include "console_visualizer.hpp"
#include <iomanip>
#include <sstream>

ConsoleVisualizer::ConsoleVisualizer(int width, int height) 
    : console_width_(width), console_height_(height) {
    screen_buffer_.resize(height, std::string(width, ' '));
}

void ConsoleVisualizer::clearScreen() {
    std::cout << "\033[2J\033[H"; // 화면 클리어 및 커서를 맨 위로
}

void ConsoleVisualizer::visualizeAll(int lane_offset, float steering, float throttle, 
                                    const std::string& state, const std::vector<cv::Point2f>& obstacles) {
    clearScreen();
    
    // 화면 버퍼 초기화
    for (auto& line : screen_buffer_) {
        line = std::string(console_width_, ' ');
    }
    
    // 도로 그리기
    drawRoad();
    
    // 차량 그리기
    drawCar();
    
    // 장애물 그리기
    drawObstacles(obstacles);
    
    // 정보 표시
    drawInfo(steering, throttle, state);
    
    // 렌더링
    render();
}

void ConsoleVisualizer::drawRoad() {
    // 도로 경계선
    for (int y = 0; y < console_height_; y++) {
        screen_buffer_[y][0] = '|';
        screen_buffer_[y][console_width_-1] = '|';
    }
    
    // 차선 (중앙 기준)
    int center = console_width_ / 2;
    for (int y = console_height_ - 5; y < console_height_; y++) {
        if (y >= 0 && y < console_height_) {
            screen_buffer_[y][center] = '|';
            screen_buffer_[y][center-10] = '|';
            screen_buffer_[y][center+10] = '|';
        }
    }
}

void ConsoleVisualizer::drawCar() {
    // 차량을 하단 중앙에 그리기
    int car_y = console_height_ - 2;
    int car_x = console_width_ / 2;
    
    if (car_y >= 0 && car_y < console_height_) {
        screen_buffer_[car_y][car_x-1] = '[';
        screen_buffer_[car_y][car_x] = 'O';
        screen_buffer_[car_y][car_x+1] = ']';
    }
}

void ConsoleVisualizer::drawObstacles(const std::vector<cv::Point2f>& obstacles) {
    for (const auto& obstacle : obstacles) {
        // 장애물 위치를 콘솔 좌표로 변환
        int x = (int)(obstacle.x * console_width_ / 320.0f);
        int y = (int)(obstacle.y * console_height_ / 200.0f);
        
        if (x >= 0 && x < console_width_ && y >= 0 && y < console_height_) {
            screen_buffer_[y][x] = 'X';
        }
    }
}

void ConsoleVisualizer::drawInfo(float steering, float throttle, const std::string& state) {
    // 상단에 정보 표시
    double speed_kmh = throttle * 100.0;
    double steering_deg = steering * 30.0;
    
    std::string info = "조향: " + std::to_string(steering_deg).substr(0, 6) + "° | " +
                      "속도: " + std::to_string(speed_kmh).substr(0, 6) + "km/h | " +
                      "상태: " + state;
    
    if (info.length() < console_width_) {
        screen_buffer_[0] = info + std::string(console_width_ - info.length(), ' ');
    } else {
        screen_buffer_[0] = info.substr(0, console_width_);
    }
}

void ConsoleVisualizer::render() {
    for (const auto& line : screen_buffer_) {
        std::cout << line << std::endl;
    }
}

void ConsoleVisualizer::visualizeLane(int offset, const cv::Mat& frame) {
    // 차선 정보만 시각화 (간단한 버전)
    std::cout << "Lane Offset: " << offset << std::endl;
}

void ConsoleVisualizer::visualizeObstacle(const std::vector<cv::Point2f>& obstacles) {
    std::cout << "Obstacles: " << obstacles.size() << " detected" << std::endl;
}

void ConsoleVisualizer::visualizeControl(float steering, float throttle, const std::string& state) {
    std::cout << "Control - Steering: " << steering << ", Throttle: " << throttle 
              << ", State: " << state << std::endl;
} 