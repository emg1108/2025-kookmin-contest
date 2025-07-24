// main.cpp
#include <iostream> // 표준 입출력 스트림
#include <thread> // 멀티스레드 구현을 위한 쓰레드 라이브러리
#include <mutex> // 상호 배제(뮤텍스)
#include <condition_variable> // 조건 변수
#include <iomanip> // 출력 포맷팅
#include <atomic> // 원자성 변수
#include <memory> // 스마트 포인터
#include <string> // 문자열 처리
#include <csignal> // 시그널 처리
#include <chrono> // 시간 측정 및 sleep
#include <ctime> // 시간 변환
#include <iomanip> // 입출력 포맷 조정
#include <sstream> // 문자열 스트림 처리

#include "usb_cam.hpp" // USB 카메라 래퍼 클래스
#include "video_recorder.hpp" // 비디오 녹화 클래스
#include "lane_detector.hpp" // 차선 검출 클래스
#include "obstacle_detector.hpp" // 장애물 검출 클래스
#include "track_detector.hpp" // 트랙 구간 인식 클래스
#include "control.hpp" // 조향 제어 클래스
#include "constants.hpp" // 상수 정의 및 로드
#include "console_visualizer.hpp" // 콘솔 시각화 클래스

// 전역 변수 선언
static std::mutex frame_mutex; // 프레임 공유 시 동기화용 뮤텍스
static std::shared_ptr<cv::Mat> shared_frame = nullptr; // 최신 프레임 저장 포인터

static std::mutex lane_mutex; // 차선 오프셋 동기화용 뮤텍스
static std::atomic<int> mean_center_offset{0}; // 차선 중심 오프셋 (원자 변수)

static std::mutex obstacle_mutex; // 장애물 정보 동기화용 뮤텍스
static ObstacleDetectionResult latest_obstacle_result; // 최신 장애물 검출 결과

static std::mutex cone_mutex; // 라바콘 정보 동기화용 뮤텍스
static ConeDetectionResult latest_cone_result; // 최신 라바콘 검출 결과

static std::condition_variable control_cv; // 제어 스레드 알림용 조건 변수
static std::mutex control_mutex; // 제어 조건 변수용 뮤텍스
static bool control_ready = false; // 제어 가능 상태 플래그

static std::condition_variable first_frame_cv; // 첫 번째 프레임 대기용 조건 변수
static bool first_frame_ready = false; // 첫 번째 프레임 수신 여부
static std::atomic<bool> running{true}; // 프로그램 실행 상태 플래그

// 라이다 데이터 시뮬레이션 함수 (실제 라이다가 없을 때 사용)
std::vector<float> simulateLidarData() {
    std::vector<float> ranges(360, 10.0f); // 기본값 10m
    
    // 라바콘 시뮬레이션 (전방 90도 범위에 라바콘 배치)
    for (int i = 45; i <= 135; i++) {
        if (i % 30 == 0) { // 30도마다 라바콘 배치
            ranges[i] = 3.0f + (i % 60) * 0.1f; // 3-4m 거리에 라바콘
        }
    }
    
    // 좌우 라바콘 (차선 변경 시뮬레이션)
    ranges[90] = 2.5f;  // 정면 라바콘
    ranges[60] = 3.0f;  // 왼쪽 라바콘
    ranges[120] = 3.0f; // 오른쪽 라바콘
    
    return ranges;
}

// 실행 모드 열거형
// - DRIVE       : 차선 및 객체 검출 후 주행 제어만 수행 (녹화하지 않음)
// - RECORD      : 카메라 영상을 파일로 녹화만 수행 (주행 제어하지 않음)
// - DRIVE_RECORD: 주행 제어와 영상 녹화를 동시에 수행
enum class Mode { DRIVE, RECORD, DRIVE_RECORD };
static Mode current_mode = Mode::DRIVE; // 기본 실행 모드는 DRIVE

// SIGINT 시그널(CTRL+C) 처리 함수
void signal_handler(int) {
    running = false; // 프로그램 종료 플래그 설정
    std::cout << "\n[INFO] 종료 시그널 감지됨. 프로그램 종료 중...\n";
}

// 날짜/시간 기반 파일명 생성 함수
std::string getTimestampedFilename(const std::string& base_dir) {
    auto now = std::chrono::system_clock::now(); // 현재 시간
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&t); // 로컬 시간 변환

    std::ostringstream oss;
    oss << base_dir << "/output_"
        << std::setw(2) << std::setfill('0') << tm->tm_mday  // 일
        << std::setw(2) << std::setfill('0') << tm->tm_hour  // 시
        << std::setw(2) << std::setfill('0') << tm->tm_min   // 분
        << std::setw(2) << std::setfill('0') << tm->tm_sec   // 초
        << ".avi";
    return oss.str(); // 완성된 파일명 반환
}

int main(int argc, char** argv) {
    // 상수 파일 로드
    try {
        load_constants("constants.json"); // constants.json -> constants.hpp
        std::cout << "Steering Gain: " << STEERING_KP << "\n"; // 로드된 상수 출력
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 상수 로드 실패: " << e.what() << std::endl;
        return 1;
    }

    signal(SIGINT, signal_handler); // SIGINT 시그널 핸들러 등록

    // 실행 모드 파싱 (d, r, dr)
    if (argc < 2) {
        std::cerr << "[ERROR] 실행 인자를 지정해주세요: d, r, dr 중 하나\n";
        return 1;
    }
    std::string mode_arg = argv[1]; // 명령줄 인자
    if (mode_arg == "d") {
        current_mode = Mode::DRIVE;
    } else if (mode_arg == "r") {
        current_mode = Mode::RECORD;
    } else if (mode_arg == "dr") {
        current_mode = Mode::DRIVE_RECORD;
    } else {
        std::cerr << "[ERROR] 잘못된 모드입니다. d, r, dr 중 하나를 선택해주세요.\n";
        return 1;
    }
    std::cout << "[INFO] 선택된 모드: " << mode_arg << "\n";

    // 카메라 초기화
    USBCam cam;
    if (!cam.init()) {
        std::cerr << "[ERROR] 카메라 초기화 실패\n";
        return 1;
    }

    // 비디오 녹화 초기화 (레코드 또는 DRIVE_RECORD 모드)
    VideoRecorder recorder;
    if (current_mode == Mode::RECORD || current_mode == Mode::DRIVE_RECORD) {
        std::string filename = getTimestampedFilename("/home/orda/records/avis");
        if (!recorder.init(filename, FRAME_WIDTH, FRAME_HEIGHT, 30.0)) {
            std::cerr << "[ERROR] 비디오 저장 초기화 실패\n";
            return 1;
        }
    }

    // 카메라 캡처 스레드 (모든 모드에서 실행)
    std::thread camera_thread([&]() {
        std::cout << "[DEBUG] 카메라 스레드 시작" << std::endl;
        int frame_count = 0;
        while (running.load()) {
            cv::Mat frame = cam.getFrame(); // 프레임 읽기
            if (frame.empty()) {
                std::cout << "[DEBUG] 빈 프레임 수신" << std::endl;
                continue; // 유효 프레임 아니면 스킵
            }

            frame_count++;
            if (frame_count % 30 == 0) { // 30프레임마다 로그
                std::cout << "[DEBUG] 프레임 " << frame_count << " 처리 중..." << std::endl;
            }

            // 최신 프레임 공유
            auto ptr = std::make_shared<cv::Mat>(frame);
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                shared_frame = ptr;
                if (!first_frame_ready) {
                    first_frame_ready = true;
                    first_frame_cv.notify_all(); // 첫 프레임 수신 알림
                    std::cout << "[DEBUG] 첫 프레임 준비 완료" << std::endl;
                }
            }

            // RECORD, DRIVE_RECORD 모드에서만 녹화 수행
            if (current_mode == Mode::RECORD || current_mode == Mode::DRIVE_RECORD) {
                recorder.write(frame); // 녹화
            }

            // VIEWER 모드 화면 출력 및 ESC키 종료
            if (VIEWER) {
                // cv::imshow("Live", frame);
                if (cv::waitKey(1) == 27) {
                    running = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // CPU 과부하 방지
        }
    });

    // 첫 번째 프레임 수신 대기
    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        first_frame_cv.wait(lock, [] { return first_frame_ready; });
    }

    // DRIVE, DRIVE_RECORD 모드에서만 실행할 스레드
    std::thread lane_thread;
    std::thread obstacle_thread;
    std::thread control_thread;
    if (current_mode == Mode::DRIVE || current_mode == Mode::DRIVE_RECORD) {
        // 차선 검출 스레드
        lane_thread = std::thread([&]() {
            std::cout << "[DEBUG] 차선 검출 스레드 시작" << std::endl;
            LaneDetector lanedetector;
            int lane_count = 0;
            while (running.load()) {
                std::shared_ptr<cv::Mat> frame;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    frame = shared_frame;
                }
                if (frame && !frame->empty()) {
                    cv::Mat vis_out;
                    int offset = lanedetector.process(*frame, vis_out); // 차선 오프셋 계산
                    lane_count++;
                    if (lane_count % 30 == 0) {
                        std::cout << "[DEBUG] 차선 오프셋: " << offset << std::endl;
                    }
                    {
                        std::lock_guard<std::mutex> lock(lane_mutex);
                        mean_center_offset = offset; // 전역 오프셋 갱신
                    }
                    {
                        std::lock_guard<std::mutex> lock(control_mutex);
                        control_ready = true;
                        control_cv.notify_one(); // 제어 스레드 실행 알림
                    }
                    if (VIEWER) {
                        cv::imshow("Lane", vis_out);
                        if (cv::waitKey(1) == 27) running = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // 장애물 검출 스레드
        obstacle_thread = std::thread([&]() {
            ObstacleDetector obstacle_detector;
            if (!obstacle_detector.init()) {
                std::cerr << "[ERROR] 장애물 검출기 초기화 실패\n";
                return;
            }
            
            while (running.load()) {
                std::shared_ptr<cv::Mat> frame;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    frame = shared_frame;
                }
                if (frame && !frame->empty()) {
                    ObstacleDetectionResult result = obstacle_detector.process(*frame);
                    {
                        std::lock_guard<std::mutex> lock(obstacle_mutex);
                        latest_obstacle_result = result; // 최신 장애물 정보 갱신
                    }
                    {
                        std::lock_guard<std::mutex> lock(control_mutex);
                        control_ready = true;
                        control_cv.notify_one(); // 제어 스레드 실행 알림
                    }
                    if (VIEWER) {
                        cv::Mat debug_img = obstacle_detector.getDebugImage();
                        cv::imshow("Obstacle", debug_img);
                        if (cv::waitKey(1) == 27) running = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // 라바콘 검출 스레드
        std::thread cone_thread([&]() {
            ObstacleDetector cone_detector;
            if (!cone_detector.init()) {
                std::cerr << "[ERROR] 라바콘 검출기 초기화 실패\n";
                return;
            }
            
            // 라바콘 검출 파라미터 설정
            cone_detector.setConeDetectionRadius(4.5f);
            cone_detector.setConeMinDistance(1.0f);
            cone_detector.setConeMaxDistance(7.0f);
            cone_detector.setConeMinCount(3);
            
            while (running.load()) {
                // 라이다 데이터 시뮬레이션 (실제 라이다가 있으면 실제 데이터 사용)
                std::vector<float> lidar_ranges = simulateLidarData();
                
                ConeDetectionResult result = cone_detector.detectCones(lidar_ranges);
                {
                    std::lock_guard<std::mutex> lock(cone_mutex);
                    latest_cone_result = result; // 최신 라바콘 정보 갱신
                }
                {
                    std::lock_guard<std::mutex> lock(control_mutex);
                    control_ready = true;
                    control_cv.notify_one(); // 제어 스레드 실행 알림
                }
                
                if (VIEWER && result.has_cones) {
                    std::cout << "[CONE] 라바콘 감지: " << result.cone_count 
                              << "개, 조향각: " << result.steering_angle << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 라바콘 검출 주기
            }
        });

        // 조향 제어 스레드 (장애물 회피 포함)
        control_thread = std::thread([&]() {
            Controller controller;
            ConsoleVisualizer visualizer(80, 20);
            while (running.load()) {
                std::unique_lock<std::mutex> lock(control_mutex);
                control_cv.wait(lock, [] { return control_ready; }); // 알림 대기
                control_ready = false;
                lock.unlock();

                // 최근 검출 결과 가져오기
                int offset = 0;
                ObstacleDetectionResult obstacle_result;
                ConeDetectionResult cone_result;
                {
                    std::lock_guard<std::mutex> lock(lane_mutex);
                    offset = mean_center_offset;
                }
                {
                    std::lock_guard<std::mutex> lock(obstacle_mutex);
                    obstacle_result = latest_obstacle_result;
                }
                {
                    std::lock_guard<std::mutex> lock(cone_mutex);
                    cone_result = latest_cone_result;
                }
                
                // 라바콘 모드인지 확인
                if (cone_result.is_rubber_mode) {
                    // 라바콘 주행 모드: 라바콘 중간점을 따라 주행
                    controller.update(static_cast<int>(cone_result.steering_angle));
                } else {
                    // 일반 주행 모드: 장애물 회피를 포함한 제어
                    controller.updateWithObstacle(offset, obstacle_result);
                }
                
                // 실시간(프레임마다) 시각화
                std::vector<cv::Point2f> obstacle_points;
                for (const auto& obs : obstacle_result.obstacles) {
                    obstacle_points.push_back(obs.center);
                }
                
                // 주행 상태 문자열 생성
                std::string state_str;
                if (cone_result.is_rubber_mode) {
                    state_str = "RUBBER_MODE";
                } else {
                    switch (controller.getDriveState()) {
                        case DriveState::DRIVE: state_str = "DRIVE"; break;
                        case DriveState::HIGH_SPEED: state_str = "HIGH_SPEED"; break;
                        case DriveState::OVERTAKING: state_str = "OVERTAKING"; break;
                        case DriveState::AVOID_LEFT: state_str = "AVOID_LEFT"; break;
                        case DriveState::AVOID_RIGHT: state_str = "AVOID_RIGHT"; break;
                        case DriveState::LANE_CHANGE_LEFT: state_str = "LANE_CHANGE_LEFT"; break;
                        case DriveState::LANE_CHANGE_RIGHT: state_str = "LANE_CHANGE_RIGHT"; break;
                        case DriveState::RETURN_TO_CENTER: state_str = "RETURN_TO_CENTER"; break;
                        case DriveState::EMERGENCY_STOP: state_str = "EMERGENCY_STOP"; break;
                    }
                }
                
                // 차선 정보 추가 (라바콘 모드가 아닐 때만)
                if (!cone_result.is_rubber_mode) {
                    state_str += " L" + std::to_string(controller.getCurrentLane()) + 
                                "→" + std::to_string(controller.getTargetLane());
                }
                
                // 라바콘 정보 추가
                if (cone_result.has_cones) {
                    state_str += " CONES:" + std::to_string(cone_result.cone_count);
                }
                
                visualizer.visualizeAll(offset, controller.getSteering(), controller.getThrottle(), 
                                      state_str, obstacle_points);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 제어 주기 조절
            }
        });
    }

    // 스레드 종료 대기
    camera_thread.join();
    if (lane_thread.joinable()) lane_thread.join();
    if (obstacle_thread.joinable()) obstacle_thread.join();
    if (cone_thread.joinable()) cone_thread.join();
    if (control_thread.joinable()) control_thread.join();

    // 비디오 녹화 자원 해제
    recorder.release();

    std::cout << "[INFO] 프로그램 종료\n";
    return 0;
}
