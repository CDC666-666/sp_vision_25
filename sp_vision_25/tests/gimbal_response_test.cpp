#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

// OpenCV CommandLineParser 使用的参数描述。
// 该测试程序用于向云台发送指定模式的角度指令，并观察云台实际响应。
const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          2          | yaw轴delta角}"
  "{circle      c  |         0.08        | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{axis        x  |         yaw         | 发送信号的轴}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

// circle 模式下 yaw 轴的目标角度轨迹，单位为度。
// 这里使用周期为 1.6s、幅值为 1.75deg 的正弦信号。
double yaw_cal(double t)
{
  double A = 1.75;
  double T = 1.6;  // s

  return A * std::sin(2 * M_PI * t / T);  // 31是云台yaw初始角度，单位为度
}

// circle 模式下 pitch 轴的目标角度轨迹，单位为度。
// 相比 yaw 轴相位超前 pi/2，并叠加 18deg 的 pitch 初始角偏置。
double pitch_cal(double t)
{
  double A = 1.75;
  double T = 1.6;  // s

  return A * std::sin(2 * M_PI * t / T + M_PI / 2) + 18;  // 18是云台pitch初始角度，单位为度
}

int main(int argc, char * argv[])
{
  // 解析配置文件路径、输入信号幅度/周期/模式，以及需要测试的云台轴。
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  auto axis = cli.get<std::string>("axis");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // Exiter 用于统一处理退出信号；Plotter 用于实时绘制命令角和云台反馈角。
  tools::Exiter exiter;
  tools::Plotter plotter;

  // Gimbal 使用串口协议与下位机通信，和 auto_aim_debug_mpc 使用同一套收发方式。
  io::Gimbal gimbal(config_path);

  // triangle_wave 模式的参数：
  // circle 表示一个上升段持续的时间，按 100fps 折算为 slice 个切片；
  // dangle 是每个周期切片累加的角度增量，cmd_angle 是当前发送的目标角度。
  auto init_angle = 0;
  double slice = circle * 100;  //切片数=周期*帧率
  auto dangle = delta_angle / slice;
  double cmd_angle = init_angle;

  // 根据命令行参数选择测试轴：0 表示 yaw，1 表示 pitch。
  int axis_index = axis == "yaw" ? 0 : 1;  // 0 for yaw, 1 for pitch

  // count 用于统计当前信号模式下已经发送的周期步数。
  int count = 0;

  // 先发送一次 control=1 且角度为 0 的初始化指令，然后等待云台归零稳定。
  gimbal.send(true, false, 0, 0, 0, 0, 0, 0);
  std::this_thread::sleep_for(5s);  //等待云台归零

  // cmd_yaw/cmd_pitch 是本周期要发送的角度指令；last_* 用于绘图时对比上一周期命令。
  double cmd_yaw = 0;
  double cmd_pitch = 0;
  double last_cmd_yaw = 0;
  double last_cmd_pitch = 0;

  // circle 模式使用独立的模拟时间 t 生成轨迹；dt=5ms 对应 200fps 的轨迹采样。
  double t = 0;
  auto last_t = t;
  double dt = 0.005;  // 5ms, 模拟200fps

  // 绘图时间轴起点。triangle_wave 和 step 模式使用真实运行时间作为横轴。
  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    nlohmann::json data;
    // 记录当前时刻，并读取该时刻附近的云台姿态，用于和刚发送的命令角对齐观察。
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    // 串口反馈中包含四元数和云台状态；这里用四元数转欧拉角观察实际 yaw/pitch。
    auto state = gimbal.state();
    Eigen::Quaterniond q = gimbal.q(timestamp);
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    if (signal_mode == "triangle_wave") {
      // 三角波这里实现为单向斜坡：每个周期从 init_angle 线性增加到 delta_angle，
      // 到达 slice 后回到 init_angle，形成重复的锯齿/三角调试输入。
      if (count == slice) {
        cmd_angle = init_angle;
        if (axis_index == 0) {
          cmd_yaw = cmd_angle / 57.3;
        } else {
          cmd_pitch = cmd_angle / 57.3;
        }
        count = 0;

      } else {
        cmd_angle += dangle;
        if (axis_index == 0) {
          cmd_yaw = cmd_angle / 57.3;
        } else {
          cmd_pitch = cmd_angle / 57.3;
        }
        count++;
      }

      // 发送当前角度指令，并把命令角、上一帧命令角、IMU 反馈角写入绘图数据。
      gimbal.send(true, false, cmd_yaw, 0, 0, cmd_pitch, 0, 0);
      if (axis_index == 0) {
        data["cmd_yaw"] = cmd_yaw * 57.3;
        data["last_cmd_yaw"] = last_cmd_yaw * 57.3;
        data["gimbal_yaw"] = eulers[0] * 57.3;
        data["state_yaw"] = state.yaw * 57.3;
      } else {
        data["cmd_pitch"] = cmd_pitch * 57.3;
        data["last_cmd_pitch"] = last_cmd_pitch * 57.3;
        data["gimbal_pitch"] = eulers[1] * 57.3;
        data["state_pitch"] = state.pitch * 57.3;
      }
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      last_cmd_yaw = cmd_yaw;
      last_cmd_pitch = cmd_pitch;
      plotter.plot(data);
      // 模拟自瞄主循环约 100fps 的发送频率。
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "step") {
      // 阶跃模式每 300 个循环把目标角增加 delta_angle，用于观察云台阶跃响应。
      if (count == 300) {
        cmd_angle += delta_angle;
        count = 0;
      }
      // 当前 step 模式固定测试 yaw 轴，并对角度做弧度归一化。
      cmd_yaw = tools::limit_rad(cmd_angle / 57.3);
      cmd_pitch = 0;
      count++;

      // 发送阶跃指令，并绘制当前命令、上一帧命令和云台 yaw 反馈。
      gimbal.send(true, false, cmd_yaw, 0, 0, cmd_pitch, 0, 0);
      data["cmd_yaw"] = cmd_yaw * 57.3;
      data["last_cmd_yaw"] = last_cmd_yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["state_yaw"] = state.yaw * 57.3;
      last_cmd_yaw = cmd_yaw;
      last_cmd_pitch = cmd_pitch;
      plotter.plot(data);
      // 模拟自瞄主循环约 100fps 的发送频率。
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "circle") {
      // circle 模式同时给 yaw/pitch 发送正弦轨迹，方便观察二维联动响应。
      std::cout << "t: " << t << std::endl;
      cmd_yaw = yaw_cal(t) / 57.3;
      cmd_pitch = pitch_cal(t) / 57.3;
      t += dt;
      // 每运行约 2s 后跳过一段轨迹时间，制造不连续轨迹片段来观察云台跟随能力。
      if (t - last_t > 2) {
        t += 2.4;
        last_t = t;
      }
      gimbal.send(true, false, cmd_yaw, 0, 0, cmd_pitch, 0, 0);

      // circle 模式使用轨迹模拟时间 t 作为横轴，同时绘制 yaw/pitch 的命令和反馈。
      data["t"] = t;
      data["cmd_yaw"] = cmd_yaw * 57.3;
      data["cmd_pitch"] = cmd_pitch * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["gimbal_pitch"] = eulers[1] * 57.3;
      data["state_yaw"] = state.yaw * 57.3;
      data["state_pitch"] = state.pitch * 57.3;
      plotter.plot(data);
      std::this_thread::sleep_for(9ms);
    }
  }
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  return 0;
}
