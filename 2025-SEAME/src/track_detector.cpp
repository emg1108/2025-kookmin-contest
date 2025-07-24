#include "track_detector.hpp"
#include <iostream>
#include <algorithm>

TrackDetector::TrackDetector() {
}

TrackDetector::~TrackDetector() {
}

TrackInfo TrackDetector::analyzeTrack(const cv::Mat& frame) {
    TrackInfo info;
    
    // 곡률 계산
    info.track_curvature = calculateCurvature(frame);
    
    // 직선 구간 판단
    info.is_straight = isStraightSection(frame);
    
    // 구간 분류
    info.current_section = classifySection(frame, info.track_curvature);
    
    // 진행률 계산
    info.section_progress = calculateProgress(frame);
    
    // 앞차 존재 여부 (간단한 시뮬레이션)
    static int frame_count = 0;
    frame_count++;
    info.has_obstacle_ahead = (frame_count % 300 == 0); // 300프레임마다 앞차 시뮬레이션
    
    return info;
}

std::string TrackDetector::getRecommendedMode(const TrackInfo& track_info) {
    switch (track_info.current_section) {
        case TrackSection::START_LINE:
            return "DRIVE"; // 신호 대기 후 출발
            
        case TrackSection::LANE_FOLLOWING:
            return "DRIVE"; // 기본 차선 추종
            
        case TrackSection::HIGH_SPEED:
            return "HIGH_SPEED"; // 고속주행
            
        case TrackSection::OVERTAKING:
            if (track_info.has_obstacle_ahead) {
                return "OVERTAKING"; // 추월
            } else {
                return "HIGH_SPEED"; // 추월 완료 후 고속주행
            }
            
        case TrackSection::FINISH_LINE:
            return "DRIVE"; // 결승선 통과
            
        default:
            return "DRIVE";
    }
}

float TrackDetector::calculateProgress(const cv::Mat& frame) {
    // 간단한 진행률 계산 (화면 하단에서 상단으로)
    if (frame.empty()) return 0.0f;
    
    // 하단 영역의 차선 위치로 진행률 추정
    cv::Mat roi = frame(cv::Rect(0, frame.rows * 0.7, frame.cols, frame.rows * 0.3));
    
    // 차선 중심점 계산
    cv::Moments moments = cv::moments(roi);
    if (moments.m00 != 0) {
        float center_x = moments.m10 / moments.m00;
        float normalized_progress = center_x / frame.cols;
        return std::max(0.0f, std::min(1.0f, normalized_progress));
    }
    
    return 0.5f; // 기본값
}

bool TrackDetector::isStraightSection(const cv::Mat& frame) {
    float curvature = calculateCurvature(frame);
    return curvature < 0.1f; // 곡률이 작으면 직선
}

float TrackDetector::calculateCurvature(const cv::Mat& frame) {
    if (frame.empty()) return 0.0f;
    
    // 그레이스케일 변환
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // 엣지 검출
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);
    
    // 차선 검출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50, 10);
    
    if (lines.empty()) return 0.0f;
    
    // 선들의 각도로 곡률 추정
    float total_angle = 0.0f;
    int valid_lines = 0;
    
    for (const auto& line : lines) {
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]);
        if (std::abs(angle) < CV_PI/4) { // 수직에 가까운 선만
            total_angle += std::abs(angle);
            valid_lines++;
        }
    }
    
    if (valid_lines == 0) return 0.0f;
    
    return total_angle / valid_lines;
}

std::vector<cv::Point> TrackDetector::detectTrackFeatures(const cv::Mat& frame) {
    std::vector<cv::Point> features;
    
    if (frame.empty()) return features;
    
    // 그레이스케일 변환
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // 코너 검출
    cv::goodFeaturesToTrack(gray, features, 100, 0.01, 10);
    
    return features;
}

TrackSection TrackDetector::classifySection(const cv::Mat& frame, float curvature) {
    // 곡률과 진행률을 바탕으로 구간 분류
    float progress = calculateProgress(frame);
    
    if (progress < 0.1f) {
        return TrackSection::START_LINE;
    } else if (progress < 0.3f) {
        return TrackSection::LANE_FOLLOWING;
    } else if (progress < 0.6f) {
        return TrackSection::HIGH_SPEED;
    } else if (progress < 0.8f) {
        return TrackSection::OVERTAKING;
    } else {
        return TrackSection::FINISH_LINE;
    }
}

bool TrackDetector::detectStartFinishLine(const cv::Mat& frame) {
    if (frame.empty()) return false;
    
    // 시작선/결승선 검출 (간단한 구현)
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // 수평선 검출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(gray, lines, 1, CV_PI/180, 50, 50, 10);
    
    for (const auto& line : lines) {
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]);
        if (std::abs(angle) < CV_PI/18) { // 거의 수평한 선
            return true;
        }
    }
    
    return false;
} 