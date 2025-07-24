// control.hpp
#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <thread>
#include <atomic>
#include <chrono>
#include "obstacle_detector.hpp"

enum class DriveState {
    DRIVE,              // 일반 주행 (중앙선 추종)
    HIGH_SPEED,         // 고속주행 모드
    OVERTAKING,         // 추월 모드
    AVOID_LEFT,         // 왼쪽 회피
    AVOID_RIGHT,        // 오른쪽 회피
    LANE_CHANGE_LEFT,   // 왼쪽 차선 변경
    LANE_CHANGE_RIGHT,  // 오른쪽 차선 변경
    RETURN_TO_CENTER,   // 중앙선 복귀
    EMERGENCY_STOP      // 긴급 정지
};

class Controller {
public:
    Controller();
    ~Controller();

    // 기존 함수 (차선 기반 제어)
    void update(int cross_offset);
    
    // 새로운 함수 (장애물 회피 포함)
    void updateWithObstacle(int cross_offset, const ObstacleDetectionResult& obstacle_result);

    // 시각화를 위한 접근자
    float getSteering() const { return steering_; }
    float getThrottle() const { return throttle_; }
    DriveState getDriveState() const { return drive_state_; }
    int getCurrentLane() const { return current_lane_; }
    int getTargetLane() const { return target_lane_; }

private:
    // ── 기존 멤버 ──
    DriveState drive_state_;
    float steering_;
    float throttle_;

    // ── 새로운 멤버 ──
    float emergency_steering_;  // 긴급 회피용 조향값
    float emergency_throttle_;  // 긴급 제어용 스로틀값
    bool obstacle_detected_;    // 장애물 감지 여부
    
    // ── 차선 관리 ──
    int current_lane_;          // 현재 차선 (1, 2, 3)
    int target_lane_;           // 목표 차선 (1, 2, 3)
    float lane_change_progress_; // 차선 변경 진행도 (0.0 ~ 1.0)
    bool is_changing_lane_;     // 차선 변경 중인지 여부
    
    // ── 제어 파라미터 ──
    float center_lane_offset_;  // 중앙선 오프셋
    float lane_width_;          // 차선 폭
    float lane_change_speed_;   // 차선 변경 속도
    
    // ── 기존 함수 ──
    float computeSteering(int offset) const;
    float computeThrottle(int offset) const;
    
    // ── 새로운 함수 ──
    float computeObstacleSteering(int cross_offset, const ObstacleDetectionResult& obstacle_result) const;
    float computeObstacleThrottle(int cross_offset, const ObstacleDetectionResult& obstacle_result) const;
    DriveState determineDriveState(const ObstacleDetectionResult& obstacle_result) const;
    void applyEmergencyControl();
    
    // ── 차선 관리 함수 ──
    void updateLaneChange();
    float computeLaneChangeSteering() const;
    int findBestLane(const ObstacleDetectionResult& obstacle_result) const;
    bool shouldChangeLane(const ObstacleDetectionResult& obstacle_result) const;
    void startLaneChange(int target_lane);
    void returnToCenterLane();
    float getLaneCenterOffset(int lane_number) const;
    
};

#endif // CONTROL_HPP
