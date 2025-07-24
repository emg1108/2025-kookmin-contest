#pragma once
#include <opencv2/opencv.hpp>
#include <string>

enum class TrafficLightState {
    RED,        // 빨간불 - 정지
    YELLOW,     // 노란불 - 주의
    GREEN,      // 초록불 - 진행
    UNKNOWN     // 인식 불가
};

struct TrafficLightInfo {
    TrafficLightState state;
    float confidence;        // 인식 신뢰도 (0.0 ~ 1.0)
    cv::Point2f position;    // 신호등 위치
    cv::Size2f size;         // 신호등 크기
    
    TrafficLightInfo() : state(TrafficLightState::UNKNOWN), confidence(0.0f) {}
};

class TrafficLightDetector {
public:
    TrafficLightDetector();
    ~TrafficLightDetector();
    
    // 초기화
    bool init();
    
    // 신호등 검출 메인 함수
    TrafficLightInfo detectTrafficLight(const cv::Mat& frame);
    
    // 설정 함수들
    void setMinLightSize(int min_width, int min_height);
    void setMaxLightSize(int max_width, int max_height);
    void setConfidenceThreshold(float threshold);
    
    // 디버깅용 시각화
    cv::Mat getDebugImage() const;
    
    // 신호등 상태 문자열 반환
    std::string getStateString(TrafficLightState state) const;

private:
    // 신호등 검출 파라미터
    cv::Size min_light_size_;
    cv::Size max_light_size_;
    float confidence_threshold_;
    
    // 색상 범위 (HSV)
    cv::Scalar red_lower1_, red_upper1_;    // 빨간색 범위 1
    cv::Scalar red_lower2_, red_upper2_;    // 빨간색 범위 2 (HSV에서 빨간색은 두 구간)
    cv::Scalar yellow_lower_, yellow_upper_; // 노란색 범위
    cv::Scalar green_lower_, green_upper_;   // 초록색 범위
    
    // 디버깅용 이미지
    mutable cv::Mat debug_image_;
    
    // 내부 처리 함수들
    std::vector<cv::Rect> detectLightCandidates(const cv::Mat& frame);
    TrafficLightInfo analyzeLight(const cv::Mat& frame, const cv::Rect& roi);
    TrafficLightState classifyLightColor(const cv::Mat& roi);
    float calculateConfidence(const cv::Mat& roi, TrafficLightState state);
    
    // 전처리 함수들
    cv::Mat preprocessFrame(const cv::Mat& frame);
    cv::Mat createROIMask(const cv::Mat& frame);
    
    // 색상 검출 함수들
    cv::Mat detectRedLight(const cv::Mat& hsv);
    cv::Mat detectYellowLight(const cv::Mat& hsv);
    cv::Mat detectGreenLight(const cv::Mat& hsv);
}; 