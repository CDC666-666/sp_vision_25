#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

// OpenCV CommandLineParser 使用的参数描述。
// 该调试程序只需要一个位置参数：配置文件路径；未传入时使用 configs/sentry.yaml。
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{pitch-debug    |                        | 每秒记录pitch指令、姿态和控制状态 }"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // Exiter 统一处理退出信号；Plotter 用于把规划、云台、开火等数据实时画出来。
  tools::Exiter exiter;
  tools::Plotter plotter;

  // 解析命令行参数。如果请求 help 或配置路径为空，则只打印帮助并退出。
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool pitch_debug = cli.has("pitch-debug");

  // 硬件/算法模块均从同一份 yaml 配置初始化，保证相机、云台、识别、解算、跟踪、
  // 规划使用一致的内参、外参和调参项。
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  // 主线程负责更新目标，规划线程负责消费目标。队列容量为 1，始终保留最新目标，
  // 避免规划线程处理旧帧目标造成控制滞后。std::nullopt 表示当前没有可用目标。
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  // 诊断使用曝光时刻的姿态，不额外消费串口四元数队列。
  struct PitchObservation
  {
    bool valid = false;
    double imu_pitch = 0;
    double vision_pitch = 0;
    std::chrono::steady_clock::time_point timestamp;
  };
  PitchObservation pitch_observation;
  std::mutex pitch_observation_mutex;

  // quit 用于通知规划线程退出。规划线程独立以约 100 Hz 运行，
  // 即使视觉帧率波动，也能持续向云台发送最近一次目标对应的规划结果。
  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    // t0 是绘图时间轴的起点；last_bullet_count 用于通过弹丸计数变化判断是否真的发射。
    auto t0 = std::chrono::steady_clock::now();
    auto last_pitch_log = t0;
    uint16_t last_bullet_count = 0;

    while (!quit) {
      // 读取最新目标和当前云台状态，并根据目标和实时弹速生成控制规划。
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      const auto log_time = std::chrono::steady_clock::now();
      if (pitch_debug && log_time - last_pitch_log >= 1s) {
        last_pitch_log = log_time;
        PitchObservation observation;
        {
          std::lock_guard<std::mutex> lock(pitch_observation_mutex);
          observation = pitch_observation;
        }
        if (observation.valid) {
          constexpr double rad2deg = 180.0 / CV_PI;
          // 当前电控AUTO_AIM分支用 -plan.pitch 作为限幅前的IMU目标。
          // gs.pitch来自电机相对中位角，与四元数计算出的IMU pitch不是同一个零点。
          tools::logger()->info(
            "[DEBUG-pitch] control={} mode={} target_up_deg={:.2f} fw_request_deg={:.2f} "
            "imu_deg={:.2f} joint_deg={:.2f} zero_est_deg={:.2f} vision_deg={:.2f} "
            "pose_age_ms={:.0f}",
            plan.control, gimbal.str(gimbal.mode()), -plan.target_pitch * rad2deg,
            -plan.pitch * rad2deg, observation.imu_pitch * rad2deg, gs.pitch * rad2deg,
            (observation.imu_pitch - gs.pitch) * rad2deg, observation.vision_pitch * rad2deg,
            tools::delta_time(log_time, observation.timestamp) * 1000);
          if (plan.control) {
            tools::logger()->info(
              "[DEBUG-pitch] aim_range_m={:.3f} aim_z_m={:.3f} bullet_speed_mps={:.2f}",
              planner.debug_xyza.head<2>().norm(), planner.debug_xyza[2], gs.bullet_speed);
          }
        }
      }

      // 将规划输出下发给云台：包含控制使能、开火标志，以及 yaw/pitch 的位置、
      // 速度、加速度参考量。MPC 调试时主要观察这些量与云台反馈之间的关系。
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      // 云台反馈的 bullet_count 增加，说明实际检测到一次发射。
      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      // 组织一帧绘图数据。所有曲线共用同一个时间戳 t，便于在 plotter 中对齐观察。
      nlohmann::json data;
      const auto plot_time = std::chrono::steady_clock::now();
      data["t"] = tools::delta_time(plot_time, t0);

      // 云台反馈状态：原 gimbal_yaw 是电机相对中值的关节角，不能直接与世界系 plan_yaw 相减。
      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      // 最新有效 IMU 四元数的非消费快照。与下方规划角一起绘图时，
      // 用 q_age_ms 检查姿态样本是否过旧；该时间戳是上位机识别帧头的时刻。
      data["imu_q_valid"] = gs.imu_q_valid ? 1 : 0;
      data["yaw_error_valid"] = gs.imu_q_valid && plan.control ? 1 : 0;
      if (gs.imu_q_valid) {
        const auto imu_R = gs.imu_q.toRotationMatrix();
        const double imu_yaw_raw = std::atan2(imu_R(1, 0), imu_R(0, 0));
        const double gimbal_yaw_world = solver.gimbal_yaw_world(gs.imu_q);
        data["imu_yaw_raw"] = imu_yaw_raw;
        data["gimbal_yaw_world"] = gimbal_yaw_world;
        data["q_age_ms"] = tools::delta_time(plot_time, gs.imu_q_time) * 1000;
        if (plan.control) {
          const double delta = plan.yaw - gimbal_yaw_world;
          data["yaw_error"] = std::atan2(std::sin(delta), std::cos(delta));
        }
      }

      // planner 计算出的目标瞄准角，通常用于观察目标解算位置对应的期望指向。
      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      // planner 最终给控制器/云台的 yaw 轨迹参考：位置、速度、加速度。
      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      // planner 最终给控制器/云台的 pitch 轨迹参考：位置、速度、加速度。
      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      // fire 是规划希望开火；fired 是根据反馈计数推断出的实际开火。
      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        // ekf_x()[4]/[5] 分别是目标 z 方向位置和速度，用于观察目标高度估计稳定性。
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        // ekf_x()[7] 是目标旋转角速度 w；无目标时写 0，避免绘图曲线断裂或缺字段。
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      // 把这一周期的调试数据推给绘图器。
      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  // 主线程中的图像缓存和相机时间戳。t 会用于查询同一时刻的云台姿态，
  // 让视觉解算使用尽量准确的相机曝光时刻外参。
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_frame_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    // 读取一帧图像及其时间戳，然后查询该时刻云台姿态四元数。
    camera.read(img, t);
    auto q = gimbal.q(t);

    // 更新解算器中的云台到世界坐标系旋转，用于把图像检测结果解算到世界/云台相关坐标。
    solver.set_R_gimbal2world(q);
    if (pitch_debug) {
      std::lock_guard<std::mutex> lock(pitch_observation_mutex);
      pitch_observation.valid = true;
      pitch_observation.imu_pitch = tools::eulers(q, 2, 1, 0)[1];
      pitch_observation.vision_pitch = tools::eulers(solver.R_gimbal2world(), 2, 1, 0)[1];
      pitch_observation.timestamp = t;
    }
    // 识别装甲板，随后交给 tracker 做时序关联和 EKF 状态更新。
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    // 只把优先级最高的目标传给规划线程；没有目标时传入 nullopt，让 planner 进入无目标逻辑。
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      // 将 tracker 当前估计的各块装甲板三维位姿重新投影到图像上，绿色点用于检查解算、
      // 跟踪状态和相机/云台外参是否一致。
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // planner.debug_xyza 是规划器内部选定/预测的调试瞄准装甲板位姿，
      // 红色点用于和 tracker 的绿色重投影结果做对比。
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    const auto now = std::chrono::steady_clock::now();
    const auto fps_text = fmt::format("FPS: {:.1f}", 1.0 / tools::delta_time(now, last_frame_time));
    last_frame_time = now;
    const auto text_width = cv::getTextSize(fps_text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, nullptr).width;
    tools::draw_text(img, fps_text, {img.cols - text_width - 10, 30}, {0, 255, 0}, 0.7);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  // 退出时先通知规划线程结束，再等待线程回收，最后向云台发送一帧全 0/关闭控制，
  // 防止程序退出后仍保持上一帧控制或开火状态。
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
