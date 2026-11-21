相机外参采集与求解（camera_driver 现场接口版）
版本：20261111-camera-opencv-v3
本版自动申请机位彩色 H264 流，用现有 OpenCV 读取 Image 或解码 H264 独立关键帧。
正常退出/中断清理时释放自己的会话；不需要 PyAV，不调用 ffmpeg 命令行程序。
修复没有图像时按 s 导致 accepted_count 异常退出的问题；未收到图像时继续等待。
本地验证包括合成板、模拟 ROS 采集与会话释放、实际 OpenCV 解码以及关键帧原始时间戳保留。
尚未在真机 Ubuntu 验证。

一、上传及启动
把整个 camera_calibration_opencv_v3 目录上传到 /home/test/。
直接使用现场现有 cv2，不安装新的 Python 解码库。
H264 需要现有 OpenCV 的视频后端支持（例如已编入的 FFmpeg/GStreamer）；不能仅由 import cv2 成功推定。
在能正常执行 ros2 topic list 的厂家 ROS 2 终端运行下面相应的一条：

左腕：
bash /home/test/camera_calibration_opencv_v3/capture_camera.sh left_wrist

右腕：
bash /home/test/camera_calibration_opencv_v3/capture_camera.sh right_wrist

胸部（已确认对应接口 head；ID0 固定到左臂末端）：
bash /home/test/camera_calibration_opencv_v3/capture_camera.sh head --tag-arm left

如果本次 ID0 实际固定在右臂，最后一条改为 --tag-arm right，必须与实际一致。
每路 15 个有效姿态：前 10 个拟合，后 5 个留出验证，采满自动退出。
换一次位置和朝向，停稳至少 1 秒，输入 s 回车保存。q 回车提前结束。
H264 路径按独立 IDR 关键帧取样。若没有新鲜关键帧，会等待下一帧，期间保持停稳。
默认最多等待 10 秒，可用 --frame-wait-s 调整。没有图像时保留场次，稍后仍可再按 s。
无效帧也保留原始数据并显示原因；不计入 15 个有效姿态。
不用先填 URDF，也不用启动旧包的 RealSense 驱动。
默认申请 consumer_id=robotcal_机位_随机标识、slot_id=所选机位、stream_mask=1、codec=h264、priority=0。
priority 可用 --stream-priority 0..255 按现场控制约定指定，不自动抢占其他消费者会话。
服务返回拒绝时保留 failure_reason，不换字段重试。退出仅释放本次返回的 session_id。
会话释放未确认时会显示原因，原始请求/响应与 session_id 保存在 stream_events.jsonl。
已有其他软件开流而只需订阅时加 --no-acquire-stream。
默认 --image-source auto 优先原始 color Image；没有新鲜 Image 时使用 H264 独立关键帧。
明确使用本文档提供的 H264 主路径时加 --image-source h264。

二、现场动作和标定板
机械臂由厂家人员用原控制软件移动。本程序只申请/释放相机会话，不发机器人运动、解锁或模式指令。
使用厂家软件的视频窗口看图；需要脚本显示已保存图像时可附加 --preview，须有可用的图形/X11 会话。
本脚本不要求三路同时看见同一块板，各路分别采集。

左/右腕：把大板固定在外部稳定支架上，移动对应机械臂，使腕相机从不同位置、不同朝向看到板。
一轮 15 个姿态中，大板不能移动，腕相机的安装位置不能变化，腰部、底盘及参考基座保持不动。
完整板为 tag36h11，11×8，ID 3–90，黑边 49 mm、间距 12 mm。
完整 ID 排列及旋转规则沿用 config/board_definition.json；不要换成另一种同尺寸板。
默认每帧至少识别 25 个板内标签。板可以部分出画，但可见面积和图像清晰度要足够。

胸部/head：相机固定不动；将 tag36h11 ID0 黑边 50 mm 的标签刚性固定到一个明确的机械臂末端。
不要手拿标签，也不要把标签固定在本轮还会开合的手指上。
移动这条机械臂，让 ID0 在胸部画面中出现不同位置、距离及倾斜方向。
这一轮头/胸相机、腰部、底盘与参考基座都保持不动。
单标签正对相机时可能出现平面姿态二义性；若脚本提示 ambiguous，换一个倾斜朝向再保存。

15 个姿态必须包含绕不同轴的转动与位置变化；不能只平移、只绕同一个轴转动、
重复同一姿态，或只重复之前的 ±0.05 rad 小幅往返。请厂家在可行工作空间内安排。
前 10 个和后 5 个都应包含不同朝向；不要把全部变化集中在最后 5 个验证姿态。
实物黑边/间距若与板配置不同，应先测量并使用对应板定义，不能在采集后随意改尺度。

三、数据在哪里
默认每轮自动新建：
/home/test/robotcal_data/cameras/left_wrist_日期_时间/
/home/test/robotcal_data/cameras/right_wrist_日期_时间/
/home/test/robotcal_data/cameras/head_日期_时间/

启动时和退出时会显示本次完整目录。可用 --output /home/test/robotcal_data/cameras/本次编号 指定新目录。
已存在的目录不会被覆盖。每次启动都是一个新场次；不同场次的板/标签安装不能直接混合。

每轮包含：
samples.json               全部帧索引、内参、识别角点、板位姿、关节配对、质量标记及拟合/验证划分
samples/0001/image.png      无损保存的彩色图像（用于标定）
samples/0001/image_data.bin Image 源为原始像素；H264 源为解码 BGR 像素，来源在 sample.json 标明
samples/0001/sample.json    单帧记录和前后近邻关节反馈；原始单位另存，不覆盖原始值
feedback_raw.jsonl         程序收到的全部 JointState、可用 ArmState、CameraInfo、CameraModuleStatus
board_definition.json      本轮使用的板定义副本
latest_preview.jpg         最近一次保存图像的标记识别预览
stream_events.jsonl        彩色流申请/释放的请求、响应、session_id 和解码错误
h264_packets.bin           收到的 H264 原始包字节，按接收顺序保存
h264_packets.jsonl         每包字节偏移/长度、Header、pts_us、机位、分辨率、关键帧标记、接收时间

程序只保存用户按 s 的解码图像；关节反馈和收到的 H264 原始包持续记录。
PNG 无损保存解码像素，但不能消除 H264 传输已有的压缩损失。
ROS BEST_EFFORT 订阅不保证源端消息零丢失。
保留原始时间戳和接收时间；不通过猜测时钟偏移来伪造图像/关节同步。
OpenCV 读取包含 SPS/PPS 和一个 IDR 图片的临时 .h264 文件，读完删除临时文件。
支持 Annex B 和经过结构验证的四字节长度前缀 AVCC；其余封装原样记录并显示错误。
仅将恰好解出一张图片的原包 Header 用于图像配对，保留原始 pts_us；不使用解码完成时间冒充拍摄时间。
P/B 帧全部保存在原始包文件中，不作为此静态采集模式的测量图片，避免猜测 OpenCV 的帧重排对应关系。
无法关联、时间戳无效、内参不匹配或图像过期时保留原始数据，不冒充有效标定帧。
现场 /manipulator/joint_states 已确认反馈为度，程序另存 q_rad 供 URDF FK 使用。
只有后续现场确实改成弧度发布时才加 --joint-unit rad。
原始 CameraInfo 全字段保存，不用网上内参；使用当前图像对应的 K、D。
当前处理 plumb_bob/rational_polynomial；其他模型会保留原始数据并拒绝直接用于本求解。
失败帧不冒充有效标定帧。

四、求外参（机器人模型可后补）
现场完成数据采集后，需要真实机器人 URDF、关节对应关系和参考/末端坐标系名。
这些内容目前尚未提供，所以采集不依赖它们，求解时必须填写。
修改 config/robot_frames.json：urdf_path，左右臂 reference_frame 和 tool_frame。
joint_map 留空表示反馈关节名与 URDF 同名；不同名时写“反馈名: 对应 URDF 名”。
不能拿示例/合成 URDF 代替真机模型。URDF 结构、关节方向/零位误差也会影响手眼结果。
选择包含已记录关节的链；本包不能凭空补充未采集的腰部关节位置。

配置完成后执行：
bash /home/test/camera_calibration_opencv_v3/solve_camera.sh /home/test/robotcal_data/cameras/本轮完整目录名

结果保存在该场次 analysis/求解时间/：
candidate.json       外参 4×4、平移(m)、四元数(xyzw)、拟合/验证一致性、明确的源/目标坐标系
report.txt           可直接阅读的结果文档
samples_with_fk.json 补入机器人 FK 的计算输入副本
robot_used.urdf      本次机器人模型副本
robot_frames_used.json 本次关节/坐标系配置

腕部输出 T_tool_camera：相机彩色 optical 坐标 → 所填末端坐标。
胸部输出 T_base_camera：head 彩色 optical 坐标 → 所填 reference_frame。
reference_frame 若填机械臂基座，结果就是相对该基座；不会自动等同于胸部/整机 base_link。
相机 optical 坐标不一定等于 URDF camera_link；写入模型前必须确认转换方向和固定光学轴变换。
求解使用现有离线包的 OpenCV 手眼算法，加入姿态退化检查。
只输出候选值，不自动改 TF、URDF、USD。留出一致性不是外部绝对精度保证。

五、环境
使用厂家 ROS 2 环境的系统 Python；默认 /usr/bin/python3，不调用旧离线包的 x86_64 runtime。
需要 rclpy、sensor_msgs、rosidl_runtime_py、camera_driver、numpy、带 aruco 和视频读取后端的 cv2。
已有 ROS 2 而只缺 NumPy/OpenCV 时，Ubuntu 联网环境可安装系统包：
sudo apt-get install python3-numpy python3-opencv
离线电脑需预先准备匹配 CPU 架构及 Ubuntu 版本的系统依赖；本目录没有捆绑二进制运行时。
不使用 pyrealsense2 或 cv_bridge，也不占用相机 USB。

订阅的图像接口：
/camera_driver/slots/left_wrist/color 和 /color/camera_info
/camera_driver/slots/right_wrist/color 和 /color/camera_info
/camera_driver/slots/head/color 和 /color/camera_info
另订阅 /camera_driver/slots/<slot_id>/h264（camera_driver/msg/H264Packet，BEST_EFFORT）。
Image 直接读取；H264Packet.data 的独立关键帧用 cv2.VideoCapture 解码后用于板识别，保留原始 Header/pts_us。
CameraInfo 按接口文档使用 RELIABLE + TRANSIENT_LOCAL。
使用用户现场 ros2 interface show 确认的 AcquireStream/ReleaseStream/H264Packet 字段。
申请流成功不代表已收到图像；缺帧时会显示 H264 包计数和解码错误，按 s 不再退出。

参考：
沿用原包 config/board_definition.json 和手眼方程、用户提供的 camera_driver/manipulator 接口。
OpenCV 手眼定义：https://docs.opencv.org/4.11.0/d9/d0c/group__calib3d.html
ROS 2 QoS：https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
OpenCV VideoCapture：https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html
OpenCV 视频后端：https://docs.opencv.org/4.x/d0/da7/videoio_overview.html
