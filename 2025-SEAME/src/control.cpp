// control.cpp
#include "control.hpp"
#include "constants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include "shared_mem.hpp"

using namespace std::chrono;  // 시간 관련 유틸 사용

// 전역 공유 변수 정의
std::atomic<float> global_steering{0.0f};
std::atomic<float> global_throttle{0.0f};

// 생성자: 
Controller::Controller()
    : drive_state_(DriveState::DRIVE),   // 초기 주행 상태 설정
      steering_(-0.0f),                 // 기본 스티어링 초기값
      throttle_(0.0f),                  // 기본 스로틀 초기값
      emergency_steering_(0.0f),        // 긴급 조향 초기값
      emergency_throttle_(0.0f),        // 긴급 스로틀 초기값
      obstacle_detected_(false),        // 장애물 감지 초기값
      current_lane_(2),                 // 현재 차선 (중앙 차선)
      target_lane_(2),                  // 목표 차선 (중앙 차선)
      lane_change_progress_(0.0f),      // 차선 변경 진행도
      is_changing_lane_(false),         // 차선 변경 중 여부
      center_lane_offset_(0.0f),        // 중앙선 오프셋
      lane_width_(100.0f),              // 차선 폭 (픽셀)
      lane_change_speed_(0.02f)         // 차선 변경 속도
{}

// 소멸자: 모터 정지,
Controller::~Controller() {
    // 모터를 완전히 중지시켜 안전 확보
}

// cross_offset: 차선 중심 대비 오프셋
void Controller::update(int cross_offset) {
    try {
        throttle_ = computeThrottle(cross_offset);
        steering_ = computeSteering(cross_offset);
        
        int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        ftruncate(shm_fd, sizeof(SharedData));
        SharedData* shm_ptr = (SharedData*)mmap(0, sizeof(SharedData), PROT_WRITE, MAP_SHARED, shm_fd, 0);

        // 쓰기 예시
        shm_ptr->steering = steering_;   // 계산된 값
        shm_ptr->throttle = throttle_;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] Python 제어 실패: " << e.what() << "\n";
    }
    // std::cerr << "[DEBUG] update called: steering = " << steering_ << ", throttle = " << throttle_ << "\n";
}

// computeSteering: 오프셋 기반 조향 계산 (비례 제어 + 범위 제한)
float Controller::computeSteering(int offset) const {
    return std::clamp(STEERING_OFFSET + STEERING_KP * offset, -0.7f, 0.7f);
}

// computeThrottle: 현재 고정 스로틀 반환 (추후 속도 제어 로직 보완 가능)
float Controller::computeThrottle(int /*offset*/) const {
    return BASE_THROTTLE;
}

// 새로운 함수들 구현

void Controller::updateWithObstacle(int cross_offset, const ObstacleDetectionResult& obstacle_result) {
    try {
        // 차선 변경 진행도 업데이트
        if (is_changing_lane_) {
            updateLaneChange();
        }
        
        // 주행 상태 결정
        drive_state_ = determineDriveState(obstacle_result);
        
        // 장애물이 감지되었는지 확인
        obstacle_detected_ = obstacle_result.has_obstacle;
        
        // 차선 변경 필요 여부 확인
        if (!is_changing_lane_ && shouldChangeLane(obstacle_result)) {
            int best_lane = findBestLane(obstacle_result);
            if (best_lane != current_lane_) {
                startLaneChange(best_lane);
            }
        }
        
        // 상태에 따른 제어값 계산
        switch (drive_state_) {
            case DriveState::DRIVE:
                // 일반 주행: 중앙선 추종
                throttle_ = computeObstacleThrottle(cross_offset, obstacle_result);
                steering_ = computeObstacleSteering(cross_offset, obstacle_result);
                break;
                
            case DriveState::HIGH_SPEED:
                // 고속주행: 최대 속도로 주행
                throttle_ = MAX_THROTTLE;
                steering_ = computeObstacleSteering(cross_offset, obstacle_result);
                break;
                
            case DriveState::LANE_CHANGE_LEFT:
            case DriveState::LANE_CHANGE_RIGHT:
                // 차선 변경 중
                throttle_ = BASE_THROTTLE * 0.8f; // 차선 변경 시 속도 감소
                steering_ = computeLaneChangeSteering();
                break;
                
            case DriveState::RETURN_TO_CENTER:
                // 중앙선 복귀
                throttle_ = BASE_THROTTLE;
                steering_ = computeSteering(cross_offset);
                returnToCenterLane();
                break;
                
            case DriveState::OVERTAKING:
                // 추월: 바깥쪽 차선으로 이동 후 복귀
                throttle_ = BASE_THROTTLE * 1.2f; // 추월 시 속도 증가
                steering_ = 0.4f; // 오른쪽으로 조향 (추월)
                break;
                
            case DriveState::AVOID_LEFT:
                // 왼쪽 회피
                throttle_ = BASE_THROTTLE * 0.8f; // 속도 감소
                steering_ = -0.5f; // 왼쪽으로 강하게 조향
                break;
                
            case DriveState::AVOID_RIGHT:
                // 오른쪽 회피
                throttle_ = BASE_THROTTLE * 0.8f; // 속도 감소
                steering_ = 0.5f; // 오른쪽으로 강하게 조향
                break;
                
            case DriveState::EMERGENCY_STOP:
                // 긴급 정지
                applyEmergencyControl();
                break;
        }
        
        // 공유 메모리에 제어값 전달
        int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        ftruncate(shm_fd, sizeof(SharedData));
        SharedData* shm_ptr = (SharedData*)mmap(0, sizeof(SharedData), PROT_WRITE, MAP_SHARED, shm_fd, 0);

        shm_ptr->steering = steering_;
        shm_ptr->throttle = throttle_;
        
        // 디버그 출력
        if (obstacle_detected_ || is_changing_lane_) {
            std::cout << "[DEBUG] 상태: " 
                      << (drive_state_ == DriveState::DRIVE ? "DRIVE" :
                          drive_state_ == DriveState::LANE_CHANGE_LEFT ? "LANE_CHANGE_LEFT" :
                          drive_state_ == DriveState::LANE_CHANGE_RIGHT ? "LANE_CHANGE_RIGHT" :
                          drive_state_ == DriveState::RETURN_TO_CENTER ? "RETURN_TO_CENTER" :
                          drive_state_ == DriveState::AVOID_LEFT ? "AVOID_LEFT" :
                          drive_state_ == DriveState::AVOID_RIGHT ? "AVOID_RIGHT" : "EMERGENCY_STOP")
                      << ", 현재차선: " << current_lane_ 
                      << ", 목표차선: " << target_lane_
                      << ", 조향: " << steering_ 
                      << ", 스로틀: " << throttle_ << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] 장애물 회피 제어 실패: " << e.what() << "\n";
    }
}

float Controller::computeObstacleSteering(int cross_offset, const ObstacleDetectionResult& obstacle_result) const {
    float base_steering = computeSteering(cross_offset);
    
    if (!obstacle_result.has_obstacle) {
        return base_steering; // 장애물이 없으면 기본 조향
    }
    
    // 장애물이 있으면 회피 조향 추가
    float avoidance_steering = 0.0f;
    
    if (obstacle_result.recommended_action == -1) {
        avoidance_steering = -0.3f; // 왼쪽 회피
    } else if (obstacle_result.recommended_action == 1) {
        avoidance_steering = 0.3f;  // 오른쪽 회피
    }
    
    // 기본 조향과 회피 조향을 결합 (가중치 적용)
    float combined_steering = base_steering * 0.7f + avoidance_steering * 0.3f;
    
    return std::clamp(combined_steering, -0.7f, 0.7f);
}

float Controller::computeObstacleThrottle(int cross_offset, const ObstacleDetectionResult& obstacle_result) const {
    float base_throttle = computeThrottle(cross_offset);
    
    if (!obstacle_result.has_obstacle) {
        return base_throttle; // 장애물이 없으면 기본 속도
    }
    
    // 장애물이 있으면 속도 감소
    float distance_factor = std::min(1.0f, obstacle_result.nearest_distance / 3.0f);
    float reduced_throttle = base_throttle * distance_factor;
    
    return std::max(0.1f, reduced_throttle); // 최소 속도 보장
}

DriveState Controller::determineDriveState(const ObstacleDetectionResult& obstacle_result) const {
    if (!obstacle_result.has_obstacle) {
        // 장애물이 없으면 중앙선 복귀 고려
        if (current_lane_ != 2 && !is_changing_lane_) {
            return DriveState::RETURN_TO_CENTER;
        }
        return DriveState::DRIVE;
    }
    
    // 가장 가까운 장애물이 매우 가까우면 긴급 정지
    if (obstacle_result.nearest_distance < 0.2f) {
        return DriveState::EMERGENCY_STOP;
    }
    
    // 차선 변경 중이면 차선 변경 상태 유지
    if (is_changing_lane_) {
        if (target_lane_ < current_lane_) {
            return DriveState::LANE_CHANGE_LEFT;
        } else {
            return DriveState::LANE_CHANGE_RIGHT;
        }
    }
    
    // 장애물이 가까우면 회피 모드
    if (obstacle_result.nearest_distance < 0.8f) {
        if (obstacle_result.recommended_action == -1) {
            return DriveState::AVOID_LEFT;
        } else if (obstacle_result.recommended_action == 1) {
            return DriveState::AVOID_RIGHT;
        }
    }
    
    return DriveState::DRIVE;
}

void Controller::applyEmergencyControl() {
    emergency_steering_ = 0.0f;  // 직진 유지
    emergency_throttle_ = 0.0f;  // 완전 정지
    
    steering_ = emergency_steering_;
    throttle_ = emergency_throttle_;
    
    std::cout << "[WARNING] 긴급 정지 실행!" << std::endl;
}

// ── 차선 관리 함수들 ──

void Controller::updateLaneChange() {
    lane_change_progress_ += lane_change_speed_;
    
    if (lane_change_progress_ >= 1.0f) {
        // 차선 변경 완료
        current_lane_ = target_lane_;
        lane_change_progress_ = 0.0f;
        is_changing_lane_ = false;
        
        std::cout << "[INFO] 차선 변경 완료: " << current_lane_ << "차선" << std::endl;
    }
}

float Controller::computeLaneChangeSteering() const {
    // 현재 차선과 목표 차선 사이의 중간점으로 조향
    float current_lane_center = getLaneCenterOffset(current_lane_);
    float target_lane_center = getLaneCenterOffset(target_lane_);
    
    // 진행도에 따른 중간점 계산
    float target_center = current_lane_center + 
                         (target_lane_center - current_lane_center) * lane_change_progress_;
    
    // 중앙선 기준 오프셋으로 조향 계산
    float offset = target_center - center_lane_offset_;
    return computeSteering(static_cast<int>(offset));
}

int Controller::findBestLane(const ObstacleDetectionResult& obstacle_result) const {
    // 각 차선의 안전성 평가
    std::vector<bool> lane_safety = {true, true, true}; // 1, 2, 3차선
    
    // 장애물이 있는 차선은 안전하지 않음
    for (const auto& obstacle : obstacle_result.obstacles) {
        if (obstacle.lane_number >= 1 && obstacle.lane_number <= 3) {
            lane_safety[obstacle.lane_number - 1] = false;
        }
    }
    
    // 중앙 차선(2차선) 우선
    if (lane_safety[1]) return 2;
    
    // 왼쪽 차선(1차선) 차선
    if (lane_safety[0]) return 1;
    
    // 오른쪽 차선(3차선) 차선
    if (lane_safety[2]) return 3;
    
    // 모든 차선이 위험하면 현재 차선 유지
    return current_lane_;
}

bool Controller::shouldChangeLane(const ObstacleDetectionResult& obstacle_result) const {
    if (!obstacle_result.has_obstacle) return false;
    
    // 현재 차선에 장애물이 있는지 확인
    for (const auto& obstacle : obstacle_result.obstacles) {
        if (obstacle.lane_number == current_lane_ && obstacle.is_dangerous) {
            return true;
        }
    }
    
    return false;
}

void Controller::startLaneChange(int target_lane) {
    if (target_lane < 1 || target_lane > 3) return;
    
    target_lane_ = target_lane;
    lane_change_progress_ = 0.0f;
    is_changing_lane_ = true;
    
    std::cout << "[INFO] 차선 변경 시작: " << current_lane_ 
              << "차선 → " << target_lane_ << "차선" << std::endl;
}

void Controller::returnToCenterLane() {
    if (current_lane_ != 2 && !is_changing_lane_) {
        startLaneChange(2); // 중앙 차선으로 복귀
    }
}

float Controller::getLaneCenterOffset(int lane_number) const {
    // 차선 중심점 계산 (이미지 중앙 기준)
    switch (lane_number) {
        case 1: return -lane_width_;     // 왼쪽 차선
        case 2: return 0.0f;             // 중앙 차선
        case 3: return lane_width_;      // 오른쪽 차선
        default: return 0.0f;
    }
}
