# handeye_run01 候选标定记录

输入：30组1440×1080图片及wxyz四元数；棋盘格10列7行内角点，格子边长15 mm。
2026-09-20已完成handeye_run02独立数据一致性复验，五项标定参数已同步
`configs/standard3.yaml`。同步前的运行配置备份为 `standard3_before_sync.yaml`。

完整候选参数位于 `configs/handeye_standard3_candidate.yaml`。
复验/最终使用时，必须将 `camera_matrix`、`distort_coeffs`、`R_gimbal2imubody`、
`R_camera2gimbal`、`t_camera2gimbal` 作为同一组参数使用。

## 坐标约定

StandardRobotpp 的 USB 发包复制 `get_INS_quat_point()` 返回的 `INS.q`，顺序wxyz。
该四元数在 `imu_rotate()` 和 `board_rotate()` 之后解算，不能将电控安装矩阵
不加区分地复制为视觉端 `R_gimbal2imubody`。

候选映射为 `diag(-1,-1,1)`。依据：实际只有yaw/pitch运动；该映射下姿态变化
主要位于yaw/pitch；求出的相机光轴接近云台+x方向。这里采用云台前左上、相机朝前
的安装约定，轴正方向仍需实机核对。位置一致性指标本身不能验证绝对坐标轴方向。

## 计算与结果

原始输入配置备份为 `input_config.yaml`。内参由这30张图片重新拟合，结果为
`camera_intrinsics.yaml`。两种手眼算法均使用新内参和候选轴映射：

| 方法 | 棋盘位置RMS偏差 | 最大偏离均值 |
|---|---:|---:|
| calibrate_handeye | 6.07 mm | 11.82 mm |
| calibrate_robotworld_handeye | 5.87 mm | 11.98 mm |

候选配置采用 `calibrate_robotworld_handeye` 输出，t约为
`[0.0474365, 0.0189191, 0.0890791]` m；两方法的t相差约1.55 mm。
棋盘位置检查统一采用左上内角点的PnP坐标，不使用两程序各自不同的棋盘原点。

按奇/偶图片分成两组，分别只用一组拟合内参与手眼变换，再检查另一组：

| 内参 | 两次留出组RMS偏差 |
|---|---|
| 原有内参 | 27.82 / 28.45 mm |
| 训练组重新拟合内参 | 6.66 / 6.76 mm |

详细数值见 `cross_validation.json`。这是同一次采集内的分组验证，并非独立实机验证。
上述偏差衡量固定棋盘位置的一致性，不代表外参的绝对测量精度。
原内参/重估内参的误差改善支持内参不匹配这一解释，但不能排除板面不平、时序或机构模型误差。

## 独立复验

handeye_run02共18组，未参与参数拟合；重新运行检查确认结果：

- 有效图片18/18，平均PnP重投影误差0.2724 px。
- 固定棋盘位置RMS偏差4.56 mm，最大偏离均值5.93 mm。
- yaw/pitch/roll变化范围分别约27.11°、19.58°、0.48°。

独立数据的一致性检查通过；绝对坐标轴正方向和安装位置仍以实机核对为准。

后续重新采集时使用新的输出目录，操作方式如下：

固定底盘和棋盘，只改变yaw/pitch，采集10～15组不同姿态，停稳后按s保存：

```bash
./build/capture configs/handeye_standard3_candidate.yaml -o=assets/handeye_run02
python3 calibration/check_handeye.py assets/handeye_run02 -c=configs/handeye_standard3_candidate.yaml
```

检查工具不重新拟合参数。先比较新数据的RMS/最大偏差与上表，再粗测相机相对云台
原点的位置是否约为前4.74 cm、左1.89 cm、上8.91 cm。若明显不符，需要继续检查，
不能仅因图像重投影误差低就投入自瞄使用。

重现手眼计算：

```bash
./build/calibrate_robotworld_handeye assets/handeye_run01 -c=configs/handeye_standard3_candidate.yaml --no-display
```

`*_original_axes.*` 保留了计算期间使用旧轴映射的对照输出，不是推荐参数。
