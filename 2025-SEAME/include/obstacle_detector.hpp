#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <cmath>

// 라이다 포인트 정보
struct LidarPoint {
    float x, y;  // 카테시안 좌표
    float distance;  // 거리
    float angle;     // 각도
    
    LidarPoint(float x = 0.0f, float y = 0.0f, float d = 0.0f, float a = 0.0f) 
        : x(x), y(y), distance(d), angle(a) {}
};

// 라바콘 정보
struct ConeInfo {
    cv::Point2f position;    // 라바콘 위치
    float distance;          // 거리
    float confidence;        // 신뢰도
    bool is_valid;           // 유효한 라바콘인지
    
    ConeInfo() : distance(0.0f), confidence(0.0f), is_valid(false) {}
};

// 라바콘 인식 결과
struct ConeDetectionResult {
    std::vector<ConeInfo> cones;     // 검출된 라바콘들
    cv::Point2f midpoint;            // 좌우 라바콘 중간점
    bool has_cones;                  // 라바콘 존재 여부
    int cone_count;                  // 라바콘 개수
    bool is_rubber_mode;             // 라바콘 주행 모드 여부
    float steering_angle;            // 조향각
    
    ConeDetectionResult() : has_cones(false), cone_count(0), 
                           is_rubber_mode(false), steering_angle(0.0f) {}
};

// 차선 정보를 담는 구조체
struct LaneInfo {
    int lane_number;         // 차선 번호 (1, 2, 3)
    bool is_occupied;        // 장애물 점유 여부
    float lane_center_x;     // 차선 중심 x좌표
    float lane_width;        // 차선 폭
    bool is_safe_to_use;     // 안전한 차선인지 여부
    
    LaneInfo() : lane_number(0), is_occupied(false), 
                 lane_center_x(0.0f), lane_width(0.0f), 
                 is_safe_to_use(true) {}
};

// 장애물 정보를 담는 구조체
struct ObstacleInfo {
    cv::Point2f center;      // 장애물 중심점 (x, y)
    cv::Size2f size;         // 장애물 크기 (width, height)
    float distance;          // 카메라로부터의 거리 (추정)
    float confidence;        // 인식 신뢰도 (0.0 ~ 1.0)
    bool is_dangerous;       // 위험 여부 (회피 필요)
    int avoidance_direction; // 회피 방향 (-1: 왼쪽, 0: 직진, 1: 오른쪽)
    int lane_number;         // 장애물이 있는 차선 번호 (1, 2, 3)
    
    ObstacleInfo() : distance(0.0f), confidence(0.0f), 
                     is_dangerous(false), avoidance_direction(0), lane_number(0) {}
};

// 장애물 인식 결과를 담는 구조체
struct ObstacleDetectionResult {
    std::vector<ObstacleInfo> obstacles;  // 검출된 장애물들
    std::vector<LaneInfo> lanes;          // 각 차선의 상태 정보
    bool has_obstacle;                    // 장애물 존재 여부
    float nearest_distance;               // 가장 가까운 장애물 거리
    int recommended_action;               // 권장 행동 (-1: 좌회전, 0: 직진, 1: 우회전)
    int recommended_lane;                 // 권장 차선 (1, 2, 3)
    bool should_change_lane;              // 차선 변경 필요 여부
    
    ObstacleDetectionResult() : has_obstacle(false), 
                               nearest_distance(999.0f), 
                               recommended_action(0),
                               recommended_lane(2),  // 기본값은 중앙 차선
                               should_change_lane(false) {}
};

class ObstacleDetector {
public:
    ObstacleDetector();
    ~ObstacleDetector();
    
    // 초기화
    bool init();
    
    // 장애물 검출 메인 함수
    ObstacleDetectionResult process(const cv::Mat& frame);
    
    // 라이다 기반 라바콘 검출 함수
    ConeDetectionResult detectCones(const std::vector<float>& lidar_ranges);
    
    // 설정 함수들
    void setMinObstacleSize(int min_width, int min_height);
    void setMaxObstacleSize(int max_width, int max_height);
    void setDangerDistance(float distance);
    void setConfidenceThreshold(float threshold);
    void setLaneWidth(float width);  // 차선 폭 설정
    void setImageWidth(int width);   // 이미지 폭 설정
    
    // 라바콘 검출 설정
    void setConeDetectionRadius(float radius);
    void setConeMinDistance(float min_dist);
    void setConeMaxDistance(float max_dist);
    void setConeMinCount(int min_count);
    
    // 디버깅용 시각화
    cv::Mat getDebugImage() const;

private:
    // 장애물 검출 파라미터
    cv::Size min_obstacle_size_;
    cv::Size max_obstacle_size_;
    float danger_distance_;
    float confidence_threshold_;
    float lane_width_;       // 차선 폭
    int image_width_;        // 이미지 폭
    
    // 라바콘 검출 파라미터
    float cone_detection_radius_;  // 라바콘 검출 반경
    float cone_min_distance_;      // 최소 거리
    float cone_max_distance_;      // 최대 거리
    int cone_min_count_;          // 최소 라바콘 개수
    cv::Point2f prev_midpoint_;   // 이전 중간점
    
    // 디버깅용 이미지
    mutable cv::Mat debug_image_;
    
    // 내부 처리 함수들
    std::vector<cv::Rect> detectObstacleCandidates(const cv::Mat& frame);
    ObstacleInfo analyzeObstacle(const cv::Mat& frame, const cv::Rect& roi);
    float estimateDistance(const cv::Size& obstacle_size);
    int determineAvoidanceDirection(const ObstacleInfo& obstacle);
    bool isObstacleDangerous(const ObstacleInfo& obstacle);
    
    // 차선별 분석 함수들
    std::vector<LaneInfo> analyzeLanes(const std::vector<ObstacleInfo>& obstacles);
    int assignLaneToObstacle(const ObstacleInfo& obstacle);
    int findBestLane(const std::vector<LaneInfo>& lanes);
    bool isLaneSafe(const LaneInfo& lane, const std::vector<ObstacleInfo>& obstacles);
    
    // 전처리 함수들
    cv::Mat preprocessFrame(const cv::Mat& frame);
    cv::Mat createROIMask(const cv::Mat& frame);
    
    // 라바콘 검출 내부 함수들
    std::vector<LidarPoint> convertLidarToCartesian(const std::vector<float>& ranges);
    std::vector<LidarPoint> filterClosePoints(const std::vector<LidarPoint>& points, float min_dist = 1.0f);
    int isFanDetected(const std::vector<LidarPoint>& points, float radius = 4.5f);
    cv::Point2f calculateMidpoint(const std::vector<LidarPoint>& points);
    float calculateSteeringAngle(const cv::Point2f& midpoint);
    
    // 시각화 함수들
    void drawLaneLines(cv::Mat& image);
    void drawLaneStatus(cv::Mat& image, const std::vector<LaneInfo>& lanes);
    void drawConeDetection(cv::Mat& image, const ConeDetectionResult& result);
}; 