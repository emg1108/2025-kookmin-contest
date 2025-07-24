#include "traffic_light_detector.hpp"
#include <iostream>
#include <algorithm>

TrafficLightDetector::TrafficLightDetector() 
    : min_light_size_(20, 20), max_light_size_(100, 100), confidence_threshold_(0.3f) {
    
    // HSV 색상 범위 설정
    // 빨간색 (HSV에서 빨간색은 0도와 180도 근처에 있음)
    red_lower1_ = cv::Scalar(0, 100, 100);      // H: 0-10, S: 100-255, V: 100-255
    red_upper1_ = cv::Scalar(10, 255, 255);
    red_lower2_ = cv::Scalar(170, 100, 100);    // H: 170-180, S: 100-255, V: 100-255
    red_upper2_ = cv::Scalar(180, 255, 255);
    
    // 노란색
    yellow_lower_ = cv::Scalar(20, 100, 100);   // H: 20-30, S: 100-255, V: 100-255
    yellow_upper_ = cv::Scalar(30, 255, 255);
    
    // 초록색
    green_lower_ = cv::Scalar(35, 100, 100);    // H: 35-85, S: 100-255, V: 100-255
    green_upper_ = cv::Scalar(85, 255, 255);
}

TrafficLightDetector::~TrafficLightDetector() {
}

bool TrafficLightDetector::init() {
    std::cout << "[INFO] 신호등 검출기 초기화 완료" << std::endl;
    return true;
}

TrafficLightInfo TrafficLightDetector::detectTrafficLight(const cv::Mat& frame) {
    TrafficLightInfo result;
    
    if (frame.empty()) {
        return result;
    }
    
    // 프레임 전처리
    cv::Mat processed = preprocessFrame(frame);
    
    // 신호등 후보 영역 검출
    std::vector<cv::Rect> candidates = detectLightCandidates(processed);
    
    // 각 후보 영역 분석
    for (const auto& candidate : candidates) {
        TrafficLightInfo light = analyzeLight(frame, candidate);
        
        if (light.confidence >= confidence_threshold_ && light.confidence > result.confidence) {
            result = light; // 가장 신뢰도가 높은 신호등 선택
        }
    }
    
    // 디버깅 이미지 생성
    debug_image_ = frame.clone();
    if (result.state != TrafficLightState::UNKNOWN) {
        cv::Point center(result.position.x, result.position.y);
        cv::Size size(result.size.width, result.size.height);
        
        // 신호등 바운딩 박스 그리기
        cv::Scalar color;
        switch (result.state) {
            case TrafficLightState::RED:
                color = cv::Scalar(0, 0, 255); // 빨간색
                break;
            case TrafficLightState::YELLOW:
                color = cv::Scalar(0, 255, 255); // 노란색
                break;
            case TrafficLightState::GREEN:
                color = cv::Scalar(0, 255, 0); // 초록색
                break;
            default:
                color = cv::Scalar(128, 128, 128); // 회색
        }
        
        cv::rectangle(debug_image_, 
                     cv::Rect(center.x - size.width/2, center.y - size.height/2, size.width, size.height),
                     color, 2);
        
        // 상태 텍스트 표시
        std::string state_text = getStateString(result.state);
        cv::putText(debug_image_, state_text, 
                   cv::Point(center.x - size.width/2, center.y - size.height/2 - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    
    return result;
}

std::vector<cv::Rect> TrafficLightDetector::detectLightCandidates(const cv::Mat& frame) {
    std::vector<cv::Rect> candidates;
    
    // 윤곽선 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        cv::Rect bounding_rect = cv::boundingRect(contour);
        
        // 크기 필터링
        if (bounding_rect.width >= min_light_size_.width && 
            bounding_rect.height >= min_light_size_.height &&
            bounding_rect.width <= max_light_size_.width && 
            bounding_rect.height <= max_light_size_.height) {
            
            // 면적 비율 필터링 (원형 신호등)
            float aspect_ratio = (float)bounding_rect.width / bounding_rect.height;
            if (aspect_ratio > 0.5 && aspect_ratio < 2.0) {
                candidates.push_back(bounding_rect);
            }
        }
    }
    
    return candidates;
}

TrafficLightInfo TrafficLightDetector::analyzeLight(const cv::Mat& frame, const cv::Rect& roi) {
    TrafficLightInfo light;
    
    // 중심점 계산
    light.position = cv::Point2f(roi.x + roi.width/2, roi.y + roi.height/2);
    light.size = cv::Size2f(roi.width, roi.height);
    
    // ROI 추출
    cv::Mat roi_mat = frame(roi);
    
    // 색상 분류
    light.state = classifyLightColor(roi_mat);
    
    // 신뢰도 계산
    light.confidence = calculateConfidence(roi_mat, light.state);
    
    return light;
}

TrafficLightState TrafficLightDetector::classifyLightColor(const cv::Mat& roi) {
    if (roi.empty()) return TrafficLightState::UNKNOWN;
    
    // HSV 변환
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    
    // 각 색상별 마스크 생성
    cv::Mat red_mask1, red_mask2, yellow_mask, green_mask;
    cv::inRange(hsv, red_lower1_, red_upper1_, red_mask1);
    cv::inRange(hsv, red_lower2_, red_upper2_, red_mask2);
    cv::inRange(hsv, yellow_lower_, yellow_upper_, yellow_mask);
    cv::inRange(hsv, green_lower_, green_upper_, green_mask);
    
    // 빨간색 마스크 합치기
    cv::Mat red_mask;
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    
    // 각 색상의 픽셀 수 계산
    int red_pixels = cv::countNonZero(red_mask);
    int yellow_pixels = cv::countNonZero(yellow_mask);
    int green_pixels = cv::countNonZero(green_mask);
    int total_pixels = roi.rows * roi.cols;
    
    // 임계값 설정 (전체 픽셀의 20% 이상)
    int threshold = total_pixels * 0.2;
    
    // 가장 많은 픽셀을 가진 색상 선택
    if (red_pixels > threshold && red_pixels > yellow_pixels && red_pixels > green_pixels) {
        return TrafficLightState::RED;
    } else if (yellow_pixels > threshold && yellow_pixels > red_pixels && yellow_pixels > green_pixels) {
        return TrafficLightState::YELLOW;
    } else if (green_pixels > threshold && green_pixels > red_pixels && green_pixels > yellow_pixels) {
        return TrafficLightState::GREEN;
    }
    
    return TrafficLightState::UNKNOWN;
}

float TrafficLightDetector::calculateConfidence(const cv::Mat& roi, TrafficLightState state) {
    if (roi.empty() || state == TrafficLightState::UNKNOWN) return 0.0f;
    
    // HSV 변환
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    
    cv::Mat color_mask;
    switch (state) {
        case TrafficLightState::RED: {
            cv::Mat red_mask1, red_mask2;
            cv::inRange(hsv, red_lower1_, red_upper1_, red_mask1);
            cv::inRange(hsv, red_lower2_, red_upper2_, red_mask2);
            cv::bitwise_or(red_mask1, red_mask2, color_mask);
            break;
        }
        case TrafficLightState::YELLOW:
            cv::inRange(hsv, yellow_lower_, yellow_upper_, color_mask);
            break;
        case TrafficLightState::GREEN:
            cv::inRange(hsv, green_lower_, green_upper_, color_mask);
            break;
        default:
            return 0.0f;
    }
    
    int color_pixels = cv::countNonZero(color_mask);
    int total_pixels = roi.rows * roi.cols;
    
    return (float)color_pixels / total_pixels;
}

cv::Mat TrafficLightDetector::preprocessFrame(const cv::Mat& frame) {
    cv::Mat processed;
    
    // 그레이스케일 변환
    cv::cvtColor(frame, processed, cv::COLOR_BGR2GRAY);
    
    // 가우시안 블러로 노이즈 제거
    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 0);
    
    // 이진화 (밝은 신호등 검출)
    cv::threshold(processed, processed, 150, 255, cv::THRESH_BINARY);
    
    // 모폴로지 연산으로 노이즈 제거
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(processed, processed, cv::MORPH_OPEN, kernel);
    
    return processed;
}

cv::Mat TrafficLightDetector::getDebugImage() const {
    return debug_image_;
}

std::string TrafficLightDetector::getStateString(TrafficLightState state) const {
    switch (state) {
        case TrafficLightState::RED:
            return "RED";
        case TrafficLightState::YELLOW:
            return "YELLOW";
        case TrafficLightState::GREEN:
            return "GREEN";
        default:
            return "UNKNOWN";
    }
}

void TrafficLightDetector::setMinLightSize(int min_width, int min_height) {
    min_light_size_ = cv::Size(min_width, min_height);
}

void TrafficLightDetector::setMaxLightSize(int max_width, int max_height) {
    max_light_size_ = cv::Size(max_width, max_height);
}

void TrafficLightDetector::setConfidenceThreshold(float threshold) {
    confidence_threshold_ = threshold;
} 