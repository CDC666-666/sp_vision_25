#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "calibration/board.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(const std::string & config_path, const std::string & output_folder)
{
  auto yaml = YAML::LoadFile(config_path);
  const cv::Size pattern_size(yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>());
  const auto pattern_type = yaml["pattern_type"].as<std::string>("circles");
  const auto imu_source = yaml["imu_source"].as<std::string>("can");
  std::unique_ptr<io::CBoard> cboard;
  std::unique_ptr<io::Gimbal> gimbal;
  tools::logger()->info(
    "标定板: {}, {}列{}行, 姿态接口: {}", pattern_type, pattern_size.width, pattern_size.height,
    imu_source);
  if (imu_source == "serial")
    gimbal = std::make_unique<io::Gimbal>(config_path);
  else if (imu_source == "can")
    cboard = std::make_unique<io::CBoard>(config_path);
  else
    throw std::runtime_error("imu_source must be 'serial' or 'can'");

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = gimbal ? gimbal->q(timestamp) : cboard->imu_at(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    auto success = calibration::find_board_points(img, pattern_size, pattern_type, centers_2d);
    cv::drawChessboardCorners(img_with_ypr, pattern_size, centers_2d, success);
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    if (!success) {
      tools::logger()->warn("未识别完整标定板，请调整标定板后再保存");
      continue;
    }

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，相机和姿态接口会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 每次采集从1开始编号，避免覆盖或混入上一次的图片和四元数。
  std::filesystem::create_directories(output_folder);
  if (!std::filesystem::is_empty(output_folder)) {
    tools::logger()->error("输出目录非空，请使用新的采集目录: {}", output_folder);
    return 1;
  }

  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
