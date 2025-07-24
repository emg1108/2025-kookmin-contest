#include "obstacle_detector.hpp"
#include <iostream>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ObstacleDetector::ObstacleDetector() 
    : min_obstacle_size_(20, 20), 
      max_obstacle_size_(200, 200),
      danger_distance_(0.3f),  // 0.3미터 이내만 위험 (더 엄격하게)
      confidence_threshold_(0.7f),  // 신뢰도 임계값 높임
      lane_width_(100.0f),     // 차선 폭 (픽셀)
      image_width_(320),       // 이미지 폭 (기본값)
      cone_detection_radius_(4.5f),  // 라바콘 검출 반경
      cone_min_distance_(1.0f),      // 최소 거리
      cone_max_distance_(7.0f),      // 최대 거리
      cone_min_count_(3),            // 최소 라바콘 개수
      prev_midpoint_(0.0f, 0.0f)    // 이전 중간점 초기화
{
}

ObstacleDetector::~ObstacleDetector() {
}

bool ObstacleDetector::init() {
    std::cout << "[INFO] ObstacleDetector 초기화 완료" << std::endl;
    std::cout << "  - 최소 장애물 크기: " << min_obstacle_size_.width << "x" << min_obstacle_size_.height << std::endl;
    std::cout << "  - 최대 장애물 크기: " << max_obstacle_size_.width << "x" << max_obstacle_size_.height << std::endl;
    std::cout << "  - 위험 거리: " << danger_distance_ << "m" << std::endl;
    std::cout << "  - 신뢰도 임계값: " << confidence_threshold_ << std::endl;
    std::cout << "  - 차선 폭: " << lane_width_ << "픽셀" << std::endl;
    std::cout << "  - 이미지 폭: " << image_width_ << "픽셀" << std::endl;
    return true;
}

ObstacleDetectionResult ObstacleDetector::process(const cv::Mat& frame) {
    ObstacleDetectionResult result;
    
    if (frame.empty()) {
        return result;
    }
    
    // 이미지 크기 업데이트
    image_width_ = frame.cols;
    
    // 프레임 전처리
    cv::Mat processed = preprocessFrame(frame);
    
    // 장애물 후보 영역 검출
    std::vector<cv::Rect> candidates = detectObstacleCandidates(processed);
    
    // 각 후보 영역 분석
    for (const auto& candidate : candidates) {
        ObstacleInfo obstacle = analyzeObstacle(frame, candidate);
        
        if (obstacle.confidence >= confidence_threshold_) {
            // 장애물의 차선 할당
            obstacle.lane_number = assignLaneToObstacle(obstacle);
            result.obstacles.push_back(obstacle);
        }
    }
    
    // 차선별 상태 분석
    result.lanes = analyzeLanes(result.obstacles);
    
    // 결과 정리
    result.has_obstacle = !result.obstacles.empty();
    
    if (result.has_obstacle) {
        // 가장 가까운 장애물 찾기
        auto nearest = std::min_element(result.obstacles.begin(), result.obstacles.end(),
            [](const ObstacleInfo& a, const ObstacleInfo& b) {
                return a.distance < b.distance;
            });
        
        result.nearest_distance = nearest->distance;
        
        // 권장 행동 결정
        if (nearest->is_dangerous) {
            result.recommended_action = nearest->avoidance_direction;
        }
        
        // 최적 차선 찾기
        result.recommended_lane = findBestLane(result.lanes);
        result.should_change_lane = shouldChangeLane(result.lanes);
    }
    
    // 디버깅 이미지 생성
    debug_image_ = frame.clone();
    
    // 차선 구분선 그리기
    drawLaneLines(debug_image_);
    
    // 장애물 표시
    for (const auto& obstacle : result.obstacles) {
        cv::Point center(obstacle.center.x, obstacle.center.y);
        cv::Size size(obstacle.size.width, obstacle.size.height);
        
        // 장애물 바운딩 박스 그리기
        cv::Scalar color = obstacle.is_dangerous ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        cv::rectangle(debug_image_, 
                     cv::Rect(center.x - size.width/2, center.y - size.height/2, size.width, size.height),
                     color, 2);
        
        // 정보 텍스트 표시
        std::string info = "L" + std::to_string(obstacle.lane_number) + 
                          " D:" + std::to_string(int(obstacle.distance * 100)) + "cm";
        cv::putText(debug_image_, info, 
                   cv::Point(center.x - size.width/2, center.y - size.height/2 - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    
    // 차선 상태 표시
    drawLaneStatus(debug_image_, result.lanes);
    
    return result;
}

cv::Mat ObstacleDetector::preprocessFrame(const cv::Mat& frame) {
    cv::Mat processed;
    
    // 그레이스케일 변환
    cv::cvtColor(frame, processed, cv::COLOR_BGR2GRAY);
    
    // 가우시안 블러로 노이즈 제거
    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 0);
    
    // 이진화 (어두운 장애물 검출)
    cv::threshold(processed, processed, 100, 255, cv::THRESH_BINARY_INV);
    
    // 모폴로지 연산으로 노이즈 제거
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(processed, processed, cv::MORPH_OPEN, kernel);
    
    return processed;
}

std::vector<cv::Rect> ObstacleDetector::detectObstacleCandidates(const cv::Mat& frame) {
    std::vector<cv::Rect> candidates;
    
    // 윤곽선 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        cv::Rect bounding_rect = cv::boundingRect(contour);
        
        // 크기 필터링
        if (bounding_rect.width >= min_obstacle_size_.width && 
            bounding_rect.height >= min_obstacle_size_.height &&
            bounding_rect.width <= max_obstacle_size_.width && 
            bounding_rect.height <= max_obstacle_size_.height) {
            
            // 면적 비율 필터링 (너무 길쭉한 것은 제외)
            float aspect_ratio = (float)bounding_rect.width / bounding_rect.height;
            if (aspect_ratio > 0.3 && aspect_ratio < 3.0) {
                candidates.push_back(bounding_rect);
            }
        }
    }
    
    return candidates;
}

ObstacleInfo ObstacleDetector::analyzeObstacle(const cv::Mat& frame, const cv::Rect& roi) {
    ObstacleInfo obstacle;
    
    // 중심점 계산
    obstacle.center = cv::Point2f(roi.x + roi.width/2, roi.y + roi.height/2);
    obstacle.size = cv::Size2f(roi.width, roi.height);
    
    // 거리 추정
    obstacle.distance = estimateDistance(roi.size());
    
    // 신뢰도 계산 (크기와 위치 기반)
    float size_confidence = std::min(1.0f, (float)(roi.width * roi.height) / (max_obstacle_size_.width * max_obstacle_size_.height));
    float position_confidence = 1.0f - (obstacle.center.y / frame.rows); // 하단에 있을수록 높은 신뢰도
    obstacle.confidence = (size_confidence + position_confidence) / 2.0f;
    
    // 위험 여부 판단
    obstacle.is_dangerous = isObstacleDangerous(obstacle);
    
    // 회피 방향 결정
    obstacle.avoidance_direction = determineAvoidanceDirection(obstacle);
    
    return obstacle;
}

float ObstacleDetector::estimateDistance(const cv::Size& obstacle_size) {
    // 간단한 거리 추정 (크기 기반)
    // 실제로는 카메라 캘리브레이션과 깊이 센서가 필요
    float area = obstacle_size.width * obstacle_size.height;
    float estimated_distance = 1000.0f / (area + 1.0f); // 픽셀 면적이 클수록 가까움
    
    return std::min(10.0f, std::max(0.1f, estimated_distance)); // 0.1m ~ 10m 범위로 제한
}

bool ObstacleDetector::isObstacleDangerous(const ObstacleInfo& obstacle) {
    return obstacle.distance <= danger_distance_;
}

int ObstacleDetector::determineAvoidanceDirection(const ObstacleInfo& obstacle) {
    // 화면 중앙 기준으로 장애물이 어느 쪽에 있는지 판단
    float center_x = image_width_ / 2.0f;
    float obstacle_x = obstacle.center.x;
    
    if (obstacle_x < center_x - 20) {
        return 1;  // 우회전 (오른쪽으로 회피)
    } else if (obstacle_x > center_x + 20) {
        return -1; // 좌회전 (왼쪽으로 회피)
    } else {
        return 0;  // 직진 (중앙에 있으면 기본적으로 우회전)
    }
}

// ── 차선별 분석 함수들 ──

int ObstacleDetector::assignLaneToObstacle(const ObstacleInfo& obstacle) {
    float center_x = image_width_ / 2.0f;
    float obstacle_x = obstacle.center.x;
    
    // 차선 중심점 계산
    float lane1_center = center_x - lane_width_;
    float lane2_center = center_x;
    float lane3_center = center_x + lane_width_;
    
    // 가장 가까운 차선 할당
    float dist1 = std::abs(obstacle_x - lane1_center);
    float dist2 = std::abs(obstacle_x - lane2_center);
    float dist3 = std::abs(obstacle_x - lane3_center);
    
    if (dist1 <= dist2 && dist1 <= dist3) {
        return 1; // 1차선
    } else if (dist2 <= dist3) {
        return 2; // 2차선 (중앙)
    } else {
        return 3; // 3차선
    }
}

std::vector<LaneInfo> ObstacleDetector::analyzeLanes(const std::vector<ObstacleInfo>& obstacles) {
    std::vector<LaneInfo> lanes(3);
    
    // 각 차선 초기화
    for (int i = 0; i < 3; ++i) {
        lanes[i].lane_number = i + 1;
        lanes[i].is_occupied = false;
        lanes[i].is_safe_to_use = true;
        
        // 차선 중심점 계산
        float center_x = image_width_ / 2.0f;
        lanes[i].lane_center_x = center_x + (i - 1) * lane_width_;
        lanes[i].lane_width = lane_width_;
    }
    
    // 장애물이 있는 차선 표시
    for (const auto& obstacle : obstacles) {
        if (obstacle.lane_number >= 1 && obstacle.lane_number <= 3) {
            int lane_idx = obstacle.lane_number - 1;
            lanes[lane_idx].is_occupied = true;
            
            // 위험한 장애물이 있으면 안전하지 않음
            if (obstacle.is_dangerous) {
                lanes[lane_idx].is_safe_to_use = false;
            }
        }
    }
    
    return lanes;
}

int ObstacleDetector::findBestLane(const std::vector<LaneInfo>& lanes) {
    // 중앙 차선(2차선) 우선
    if (lanes[1].is_safe_to_use && !lanes[1].is_occupied) {
        return 2;
    }
    
    // 왼쪽 차선(1차선) 차선
    if (lanes[0].is_safe_to_use && !lanes[0].is_occupied) {
        return 1;
    }
    
    // 오른쪽 차선(3차선) 차선
    if (lanes[2].is_safe_to_use && !lanes[2].is_occupied) {
        return 3;
    }
    
    // 모든 차선이 점유되었으면 중앙 차선 반환
    return 2;
}

bool ObstacleDetector::shouldChangeLane(const std::vector<LaneInfo>& lanes) {
    // 현재 차선(중앙 차선)이 안전하지 않으면 차선 변경 필요
    return !lanes[1].is_safe_to_use || lanes[1].is_occupied;
}

// ── 시각화 함수들 ──

void ObstacleDetector::drawLaneLines(cv::Mat& image) {
    float center_x = image_width_ / 2.0f;
    
    // 차선 구분선 그리기
    cv::line(image, cv::Point(center_x - lane_width_, 0), 
             cv::Point(center_x - lane_width_, image.rows), 
             cv::Scalar(255, 255, 0), 2); // 노란색
    
    cv::line(image, cv::Point(center_x + lane_width_, 0), 
             cv::Point(center_x + lane_width_, image.rows), 
             cv::Scalar(255, 255, 0), 2); // 노란색
}

void ObstacleDetector::drawLaneStatus(cv::Mat& image, const std::vector<LaneInfo>& lanes) {
    for (const auto& lane : lanes) {
        std::string status = "L" + std::to_string(lane.lane_number) + ":";
        status += lane.is_occupied ? "OCC" : "FREE";
        status += lane.is_safe_to_use ? "(S)" : "(U)";
        
        cv::Scalar color = lane.is_safe_to_use ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        
        cv::putText(image, status, 
                   cv::Point(10, 20 + lane.lane_number * 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }
}

// ── 설정 함수들 ──

void ObstacleDetector::setMinObstacleSize(int min_width, int min_height) {
    min_obstacle_size_ = cv::Size(min_width, min_height);
}

void ObstacleDetector::setMaxObstacleSize(int max_width, int max_height) {
    max_obstacle_size_ = cv::Size(max_width, max_height);
}

void ObstacleDetector::setDangerDistance(float distance) {
    danger_distance_ = distance;
}

void ObstacleDetector::setConfidenceThreshold(float threshold) {
    confidence_threshold_ = threshold;
}

void ObstacleDetector::setLaneWidth(float width) {
    lane_width_ = width;
}

void ObstacleDetector::setImageWidth(int width) {
    image_width_ = width;
}

// 라바콘 검출 설정 함수들
void ObstacleDetector::setConeDetectionRadius(float radius) {
    cone_detection_radius_ = radius;
}

void ObstacleDetector::setConeMinDistance(float min_dist) {
    cone_min_distance_ = min_dist;
}

void ObstacleDetector::setConeMaxDistance(float max_dist) {
    cone_max_distance_ = max_dist;
}

void ObstacleDetector::setConeMinCount(int min_count) {
    cone_min_count_ = min_count;
}

// 라이다 기반 라바콘 검출 메인 함수
ConeDetectionResult ObstacleDetector::detectCones(const std::vector<float>& lidar_ranges) {
    ConeDetectionResult result;
    
    if (lidar_ranges.empty() || lidar_ranges.size() < 360) {
        return result;
    }
    
    // 1. 라이다 데이터를 카테시안 좌표로 변환
    std::vector<LidarPoint> cartesian_points = convertLidarToCartesian(lidar_ranges);
    
    // 2. 가까운 점들을 필터링하여 라바콘 후보 추출
    std::vector<LidarPoint> filtered_points = filterClosePoints(cartesian_points, 1.0f);
    
    // 3. 전방 90도 범위에서 라바콘 감지
    if (isFanDetected(filtered_points, cone_detection_radius_) >= cone_min_count_) {
        result.has_cones = true;
        result.is_rubber_mode = true;
        result.cone_count = filtered_points.size();
        
        // 4. 중간점 계산
        result.midpoint = calculateMidpoint(filtered_points);
        
        // 5. 조향각 계산
        result.steering_angle = calculateSteeringAngle(result.midpoint);
        
        // 6. 라바콘 정보 저장
        for (const auto& point : filtered_points) {
            ConeInfo cone;
            cone.position = cv::Point2f(point.x, point.y);
            cone.distance = point.distance;
            cone.confidence = 1.0f;  // 기본 신뢰도
            cone.is_valid = true;
            result.cones.push_back(cone);
        }
    } else {
        result.has_cones = false;
        result.is_rubber_mode = false;
        result.cone_count = 0;
        result.midpoint = prev_midpoint_;
        result.steering_angle = 0.0f;
    }
    
    return result;
}

// 라이다 데이터를 카테시안 좌표로 변환
std::vector<LidarPoint> ObstacleDetector::convertLidarToCartesian(const std::vector<float>& ranges) {
    std::vector<LidarPoint> points;
    
    for (int i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        
        // 유효한 거리 범위 체크
        if (range > cone_min_distance_ && range <= cone_max_distance_ && std::isfinite(range)) {
            // 각도 계산 (라이다는 0도가 정면, 시계방향)
            float angle = (i * 2.0f * M_PI / ranges.size()) + M_PI / 2.0f;
            
            // 카테시안 좌표로 변환
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            
            points.emplace_back(x, y, range, angle);
        }
    }
    
    return points;
}

// 가까운 점들을 필터링하여 군집화
std::vector<LidarPoint> ObstacleDetector::filterClosePoints(const std::vector<LidarPoint>& points, float min_dist) {
    if (points.empty()) return {};
    
    std::vector<LidarPoint> remaining;
    std::vector<bool> used(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (used[i]) continue;
        
        const LidarPoint& ref = points[i];
        remaining.push_back(ref);
        
        // min_dist 이내의 모든 점들을 마킹
        for (size_t j = 0; j < points.size(); ++j) {
            if (!used[j]) {
                float dist = std::sqrt(std::pow(points[j].x - ref.x, 2) + std::pow(points[j].y - ref.y, 2));
                if (dist < min_dist) {
                    used[j] = true;
                }
            }
        }
    }
    
    return remaining;
}

// 전방 90도 범위에서 라바콘 감지
int ObstacleDetector::isFanDetected(const std::vector<LidarPoint>& points, float radius) {
    if (points.empty()) return 0;
    
    int count = 0;
    
    for (const auto& point : points) {
        // 각도 조건: 45도 ~ 135도 (전방 90도)
        float angle = std::atan2(point.y, point.x);
        bool angle_condition = (angle >= M_PI / 4.0f) && (angle <= 3.0f * M_PI / 4.0f);
        
        // 거리 조건: 반지름 이내
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        bool distance_condition = distance <= radius;
        
        if (angle_condition && distance_condition) {
            count++;
        }
    }
    
    return count;
}

// 좌우 라바콘의 중간점 계산
cv::Point2f ObstacleDetector::calculateMidpoint(const std::vector<LidarPoint>& points) {
    if (points.empty()) return prev_midpoint_;
    
    // y > 1.5인 전방 점들만 선택
    std::vector<LidarPoint> forward_points;
    for (const auto& point : points) {
        if (point.y > 1.5f) {
            forward_points.push_back(point);
        }
    }
    
    if (forward_points.empty()) return prev_midpoint_;
    
    // 좌우 분리
    std::vector<LidarPoint> right_front, left_front;
    for (const auto& point : forward_points) {
        if (point.x > 0) {
            right_front.push_back(point);
        } else {
            left_front.push_back(point);
        }
    }
    
    // 각 방향에서 가장 가까운 점 찾기
    cv::Point2f right_min(0.0f, 0.0f), left_min(0.0f, 0.0f);
    
    if (!right_front.empty()) {
        auto min_it = std::min_element(right_front.begin(), right_front.end(),
            [](const LidarPoint& a, const LidarPoint& b) { return a.y < b.y; });
        right_min = cv::Point2f(min_it->x, min_it->y);
    }
    
    if (!left_front.empty()) {
        auto min_it = std::min_element(left_front.begin(), left_front.end(),
            [](const LidarPoint& a, const LidarPoint& b) { return a.y < b.y; });
        left_min = cv::Point2f(min_it->x, min_it->y);
    }
    
    // 중간점 계산
    cv::Point2f midpoint = (right_min + left_min) * 0.5f;
    
    // 이상치 필터링 (x값이 너무 멀면 이전값 유지)
    if (std::abs(midpoint.x) > 2.0f) {
        midpoint = prev_midpoint_;
    } else {
        prev_midpoint_ = midpoint;
    }
    
    return midpoint;
}

// 중간점을 기반으로 조향각 계산
float ObstacleDetector::calculateSteeringAngle(const cv::Point2f& midpoint) {
    // Python 코드와 동일하게 중간점의 x좌표를 50배로 확대
    return midpoint.x * 50.0f;
}

// 라바콘 검출 시각화
void ObstacleDetector::drawConeDetection(cv::Mat& image, const ConeDetectionResult& result) {
    if (result.has_cones) {
        // 중간점 표시
        cv::circle(image, cv::Point(320, 240), 5, cv::Scalar(0, 255, 255), -1);
        
        // 라바콘들 표시
        for (const auto& cone : result.cones) {
            // 라바콘 위치를 이미지 좌표로 변환 (간단한 변환)
            int x = static_cast<int>((cone.position.x + 7.0f) * 320.0f / 14.0f);
            int y = static_cast<int>((7.0f - cone.position.y) * 480.0f / 14.0f);
            
            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
            }
        }
        
        // 조향각 정보 표시
        std::string angle_text = "Cone Angle: " + std::to_string(static_cast<int>(result.steering_angle));
        cv::putText(image, angle_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
}

cv::Mat ObstacleDetector::getDebugImage() const {
    return debug_image_;
} 