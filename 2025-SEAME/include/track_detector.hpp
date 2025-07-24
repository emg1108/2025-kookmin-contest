#pragma once
#include <opencv2/opencv.hpp>
#include <string>

enum class TrackSection {
    START_LINE,      // 시작선
    LANE_FOLLOWING,  // 차선인식 자율주행
    HIGH_SPEED,      // 고속주행
    OVERTAKING,      // 추월구간
    FINISH_LINE      // 결승선
};

struct TrackInfo {
    TrackSection current_section;
    float section_progress;  // 구간 진행률 (0.0 ~ 1.0)
    bool is_straight;        // 직선 구간 여부
    bool has_obstacle_ahead; // 앞차 존재 여부
    float track_curvature;   // 트랙 곡률
};

class TrackDetector {
public:
    TrackDetector();
    ~TrackDetector();
    
    // 트랙 구간 분석
    TrackInfo analyzeTrack(const cv::Mat& frame);
    
    // 구간별 권장 주행 모드 결정
    std::string getRecommendedMode(const TrackInfo& track_info);
    
    // 트랙 진행률 계산
    float calculateProgress(const cv::Mat& frame);
    
    // 직선 구간 판단
    bool isStraightSection(const cv::Mat& frame);
    
    // 곡률 계산
    float calculateCurvature(const cv::Mat& frame);

private:
    // 트랙 특징점 검출
    std::vector<cv::Point> detectTrackFeatures(const cv::Mat& frame);
    
    // 구간 분류
    TrackSection classifySection(const cv::Mat& frame, float curvature);
    
    // 시작선/결승선 검출
    bool detectStartFinishLine(const cv::Mat& frame);
}; 