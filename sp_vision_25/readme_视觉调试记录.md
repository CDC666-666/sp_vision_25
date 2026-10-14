# 视觉流程与调试问题请教

老师您好，我最近在调试自瞄视觉和云台跟踪，想请教两个问题：`Target_yaw` 波形的稳定性，以及视觉程序帧率的提升方法。下面是当前使用的程序流程和观察到的现象。我会另外附上 `Target_yaw` 波形图。

## 一、视觉处理流程（含代码位置）

我目前使用下面的命令运行实车调试程序：

```bash
./build/auto_aim_debug_mpc configs/standard3.yaml
```

程序逐帧处理旋转小车的实时相机画面。下面的行号对应当前工作区代码，方便直接查看。

| 步骤 | 代码位置 | 具体做的事 |
| --- | --- | --- |
| 读取配置并初始化模块 | [`src/auto_aim_debug_mpc.cpp:39-56`](src/auto_aim_debug_mpc.cpp#L39) | 解析命令行中的 YAML 路径，然后用同一份配置初始化云台、相机、YOLO、Solver、Tracker 和 Planner。 |
| 确认本次实际配置 | [`configs/standard3.yaml:5-21`](configs/standard3.yaml#L5)、[`configs/standard3.yaml:54-71`](configs/standard3.yaml#L54) | 当前选择 `yolov5`、`CPU`、`use_roi: false`，相机为 `hikrobot`；这里还配置了曝光、相机内参和相机到云台的外参。 |
| 相机取帧 | [`src/auto_aim_debug_mpc.cpp:199-205`](src/auto_aim_debug_mpc.cpp#L199)、[`io/camera.cpp:11-36`](io/camera.cpp#L11)、[`io/hikrobot/hikrobot.cpp:45-52`](io/hikrobot/hikrobot.cpp#L45) | 主循环调用 `camera.read(img, t)`。`Camera` 根据配置创建 HikRobot 相机对象；HikRobot 从图像队列取出图像和时间戳。`camera_raw` 窗口使用 `img.clone()` 显示原始帧。 |
| 对齐图像和云台姿态 | [`io/gimbal/gimbal.cpp:133-169`](io/gimbal/gimbal.cpp#L133)、[`io/gimbal/gimbal.cpp:67-80`](io/gimbal/gimbal.cpp#L67)、[`src/auto_aim_debug_mpc.cpp:205-208`](src/auto_aim_debug_mpc.cpp#L205) | 云台串口线程持续读取四元数并记录接收时间；主线程用图像时间戳 `t` 查询/插值得到对应四元数，再调用 `solver.set_R_gimbal2world(q)` 更新坐标变换。 |
| 选择并运行 YOLO 模型 | [`tasks/auto_aim/yolo.cpp:11-35`](tasks/auto_aim/yolo.cpp#L11)、[`tasks/auto_aim/yolos/yolov5.cpp:57-100`](tasks/auto_aim/yolos/yolov5.cpp#L57) | 工厂类读取 `yolo_name` 并选择 YOLOv5。YOLOv5 根据 `use_roi` 选择检测区域，将图像缩放并填充到 `640×640`，调用 OpenVINO 在 CPU 上推理。 |
| 解析检测结果 | [`tasks/auto_aim/yolos/yolov5.cpp:102-192`](tasks/auto_aim/yolos/yolov5.cpp#L102) | 从网络输出解析四个角点、置信度、颜色和编号；通过置信度阈值和 NMS 筛选结果，再检查装甲板编号与类型。`use_traditional: true` 时，还会调用传统检测器尝试二次矫正角点（第 183-185 行）。 |
| 跟踪与 EKF 更新 | [`src/auto_aim_debug_mpc.cpp:216-223`](src/auto_aim_debug_mpc.cpp#L216)、[`tasks/auto_aim/tracker.cpp:31-95`](tasks/auto_aim/tracker.cpp#L31)、[`tasks/auto_aim/tracker.cpp:266-292`](tasks/auto_aim/tracker.cpp#L266) | 主循环把检测结果交给 Tracker。Tracker 过滤非敌方颜色的装甲板，按优先级选择/关联目标；首次发现目标时初始化 Target，后续帧先预测，再用当前装甲板观测更新目标状态。没有有效目标时向规划线程发送空目标。 |
| 单帧位姿解算和坐标变换 | [`tasks/auto_aim/tracker.cpp:231-263`](tasks/auto_aim/tracker.cpp#L231)、[`tasks/auto_aim/tracker.cpp:280-290`](tasks/auto_aim/tracker.cpp#L280)、[`tasks/auto_aim/solver.cpp:28-45`](tasks/auto_aim/solver.cpp#L28)、[`tasks/auto_aim/solver.cpp:64-98`](tasks/auto_aim/solver.cpp#L64) | Tracker 在初始化和更新目标时调用 `solver.solve(armor)`。Solver 使用装甲板模型点、检测角点和相机内参做 `solvePnP`，再结合相机到云台、云台到世界的变换求出目标位置和姿态。 |
| 预测并生成 `Target_yaw` | [`src/auto_aim_debug_mpc.cpp:74-88`](src/auto_aim_debug_mpc.cpp#L74)、[`tasks/auto_aim/planner/planner.cpp:27-93`](tasks/auto_aim/planner/planner.cpp#L27)、[`tasks/auto_aim/planner/planner.cpp:96-107`](tasks/auto_aim/planner/planner.cpp#L96) | 独立规划线程以约 100 Hz 读取最新目标和云台弹速，Planner 预测弹道飞行时间、生成瞄准轨迹并求解 yaw/pitch 规划。`plan.target_yaw` 在 Planner 第 76 行赋值。 |
| 下发控制和记录波形 | [`src/auto_aim_debug_mpc.cpp:117-121`](src/auto_aim_debug_mpc.cpp#L117)、[`src/auto_aim_debug_mpc.cpp:155-187`](src/auto_aim_debug_mpc.cpp#L155) | 规划结果通过 `gimbal.send(...)` 发给下位机；调试数据中的 `data["target_yaw"]` 取自 `plan.target_yaw`，然后交给 Plotter 绘图。 |
| 生成调试画面和显示 FPS | [`src/auto_aim_debug_mpc.cpp:225-254`](src/auto_aim_debug_mpc.cpp#L225) | 对目标装甲板的估计位姿做图像重投影，用绿色点画 Tracker 估计结果、红色点画规划瞄准位置；之后将画面缩小一半、绘制 FPS，并显示在 `reprojection` 窗口。 |

**关于“带检测框的视频”的说明：**当前 `auto_aim_debug_mpc` 初始化 YOLO 时传入 `false` 关闭了 YOLO 内部调试显示（[`src/auto_aim_debug_mpc.cpp:53`](src/auto_aim_debug_mpc.cpp#L53)）。YOLOv5 的检测结果绘制代码位于 [`tasks/auto_aim/yolos/yolov5.cpp:226-245`](tasks/auto_aim/yolos/yolov5.cpp#L226)，但这条调试绘制路径目前不会被该程序调用。因此，当前 `reprojection` 窗口不是检测框视频，而是带有绿色/红色重投影角点的画面。`camera_raw` 窗口则是原始相机帧。如果用录制视频离线测试，处理步骤类似，但上面的运行命令实际读取的是 HikRobot 相机实时画面。

## 二、Target_yaw 波形现象

这里的 `target_yaw` 是 Planner 根据 Tracker 的目标状态进行预测、选取瞄准装甲板后生成的目标角度，不是检测器直接测得的单帧 yaw。因此，曲线突变也可能出现在装甲板切换或角度跨越 ±π 边界时，需要结合对应画面和中间量判断，不能仅凭这条曲线归因于检测噪声。

我现在测得的 `Target_yaw` 波形不如开源效果图稳定，之前的波形还要更差一些。当前波形整体呈周期性变化，但局部有明显的不平滑和突变；我会把实际波形图发给您，方便对照判断。

我观察到，旋转小车四块装甲板中有一块的安装倾斜角度与另外三块差异较大。把装甲板安装角度调整一致后，波形看起来会好一些；在地面反光处做遮挡后，效果也有所改善。我目前只能确认这是调试中的观察，尚不能确定各因素分别造成了多大影响，也不确定波形问题主要来自检测角点、装甲板位姿解算、跟踪滤波、时间同步还是机械安装。

## 三、请教

想请教您：
- 运行时，带有处理结果的画面窗口显示帧率约为 **50 FPS**。我发现关闭原始相机画面窗口后，显示的帧率会升高。正式比赛的时候不会有这些窗口，还有哪些比较有效的方法可以提高当前程序的处理帧率？
- 从这张 `Target_yaw` 波形及对应画面看，突变更像是同一块装甲板持续可见时的估计波动，还是装甲板切换、短暂丢检后重获或角度跨越 ±π 边界造成的变化？最值得先检查哪一环？
- 除了统一装甲板安装角度和处理反光，您建议我优先记录或对比哪些中间量（例如检测角点、单帧解算 yaw、Tracker 状态或时间戳），来进一步定位波形不稳定的原因？

