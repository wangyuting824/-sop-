# 智蜂机器人标定平台 

从仓库根目录运行命令。第一次使用可先回放一条双臂样本，检查解码、文件写入和校验清单。

```bash
python -m pip install .
robotcal collect --config configs/xuchuang/arms.yaml --session artifacts/demo_001 --replay tests/samples/arms.jsonl
```

| 目录 | 维护内容 |
|---|---|
| `adapters/`、`configs/xuchuang/` | ROS 接口、设备配置、标定板定义和验收阈值 |
| `collect/`、`control/` | 采集、设备动作、就绪检查和停止接口。现场操作使用统一入口 `robotcal` |
| `compute/` | 数据准备、求参、模型导出和指标验证 |
| `compute/isaac_tools/` | 独立 Isaac Sim 工具、项目调参脚本、场景截图脚本及带原版权声明的 NVIDIA SysID 源码 |
| `compute/cad/` | 在 FreeCAD 中执行的 STEP 查看宏。输入目录由 `CALIBRATION_STEP_DIR` 指定，宏中的文件名须与输入一致 |
| `tests/samples/` | 脱敏回放样本与动作回执 |
| `artifacts/` | 由程序按场次建立，Git 默认忽略 |

扩展工具从源码目录运行。Isaac 脚本需要相应的 Isaac Sim Python、RunSpec 和模型。历史目录可以通过 `CALIBRATION_SIM_HOME` 指定，视频输入和截图输出可分别设置 `CALIBRATION_MEDIA_DIR` 和 `CALIBRATION_OUTPUT`。CAD 宏仅用于装配查看，隐藏胸壳前须核对装配树中的零件标签。

这是从旭创现场脚本整理的第一版配置式程序。通用版和旭创版使用同一套代码，区别在配置：`configs/generic/` 的未知项保留 `null`，`configs/xuchuang/` 填入已知接口。它不是“任意厂家填张表就能控制”的万能驱动。

**验证范围：离线真实消息回放、接口解析、配置替换、合成标定数据和CLI测试。新入口尚未连接机器人或Isaac Sim验证，不保证真机无故障。**

## 1 安装与自检

Python 3.10或更新版本。ROS操作必须使用厂家已加载环境的系统Python；不要在机器人上替换已有ROS、NumPy或OpenCV安装。

在准备电脑的新虚拟环境中可安装：

```bash
python -m pip install ".[vision]"
python -m unittest discover -s tests -v
```

机器人已有依赖时，在项目根目录直接运行以下命令，无需安装本项目：

```bash
python3 -c "import yaml, numpy, rclpy, rosidl_runtime_py"
python3 -m calibration_platform check --config configs/xuchuang/arms.yaml
```

`configuration_valid`只代表配置通过检查。加`--live`才会连接ROS并试读一帧：

```bash
python3 -m calibration_platform check --config configs/xuchuang/arms.yaml --live
```

相机需现场OpenCV带aruco和视频解码支持。不能仅凭`import cv2`成功保证H264能解码。相机用`collect`采样，不以`check --live`读取标量状态。

## 2 通用配置规则

复制并填写`configs/generic/`。`adapter`只能是已实现的名称，不能填Python路径或代码。未填配置返回退出码2及缺项原因，这是预期行为。

已实现的适配器：

| 名称 | 协议与支持范围 |
|---|---|
| ros2_joint_state | 标准JointState只读采集，当前只处理转动关节 |
| xuchuang_arms_ros2 | 旭创ArmStateArray、七轴双臂及MoveJoint现场动作桥 |
| xuchuang_waist_ros2 | 旭创MotorState只读及偏航现场动作桥 |
| xuchuang_gripper_ros2 | 旭创EndEffectorStatus与固定测量服务流程 |
| xuchuang_camera_driver_ros2 | 旭创camera_driver Image/H264及会话采样 |

`robot.yaml`是设备索引和模型登记，不是可直接运行的设备配置；命令的`--config`指向arms、waist、gripper或cameras文件。`acceptance.yaml`仅供compute验证使用。

更换消息结构、SDK或单位语义需要修改适配代码。通用JointState默认按标准rad/rad/s解读，仍须和实际驱动核对。协议解析和结果码留在代码里；测点、幅度、循环留在固定控制流程中，不另建试验YAML。

## 3 双臂现场流程

以下旭创命令只能在完成现场模式准备和运动条件确认后使用。先用原控制系统关闭流式伺服并切到位置模式；程序不会自动使能或切模式。

```bash
ros2 service call /manipulator/enable_stream_servo std_srvs/srv/SetBool "{data: false}"
ros2 service call /manipulator/set_control_state manipulator/srv/SetControlState "{arm_id: 0, state: 1, wait_transition: true, timeout_sec: 10.0}"
ros2 service call /manipulator/set_control_state manipulator/srv/SetControlState "{arm_id: 1, state: 1, wait_transition: true, timeout_sec: 10.0}"
```

第一条检查success=true；后两条逐条检查result.code=0且ctrl_state_cur=1。不满足就不继续。

终端1先采集：

```bash
python3 -m calibration_platform collect --config configs/xuchuang/arms.yaml --session artifacts/xc_run_001 --duration-s 1200
```

等到`Collection ready: valid feedback saved`后，终端2先预览：

```bash
python3 -m calibration_platform control --config configs/xuchuang/arms.yaml --session artifacts/xc_run_001 --arm right --joint 1 --mode step
```

确认后执行单次+0.05rad点动：

```bash
python3 -m calibration_platform control --config configs/xuchuang/arms.yaml --session artifacts/xc_run_001 --arm right --joint 1 --mode step --direction 1 --execute
```

反向为`--direction -1`。左臂用`--arm left`。轴号1..7。往返使用`--mode oscillation`，固定±0.05rad、3循环、每段2秒；该模式可以`--arm both --joint all`。单次点动不能both/all。

采集和控制必须同目录、同设备配置、同一主机。执行前检查采集心跳，每次双臂目标发送前再次检查。采集时长必须覆盖全部动作。离线回放不会产生可供运动使用的就绪信号。

**保留现场行为边界：**动作仍由原v5逻辑执行，检查traj_plan_state与厂家结果。0.01速度/加速度原样传给接口，不重新解释。原生反馈按度制转换。实际关节范围、URDF映射、可靠停止与控制器实际目标回读未完成确认；本程序不补造这些信息。已发送目标不能通过“结束CLI”保证停止，使用原控制系统负责现场停止。`stop`命令明确报不支持，不假装成功。

## 4 腰部与夹爪

腰部终端1：

```bash
python3 -m calibration_platform collect --config configs/xuchuang/waist.yaml --session artifacts/waist_001 --duration-s 0
```

终端2先不加`--execute`预览，确认后：

```bash
python3 -m calibration_platform control --config configs/xuchuang/waist.yaml --session artifacts/waist_001 --execute
```

仅偏航，±0.05rad、3循环。v3现场仍出现反馈超时；新代码保留该检查，不承诺已修复。头部故障只记录，检查腰部四轴。无设备时间戳，不冒充设备同步。

夹爪终端1：

```bash
python3 -m calibration_platform collect --config configs/xuchuang/gripper.yaml --session artifacts/gripper_001 --duration-s 0
```

终端2：

```bash
python3 -m calibration_platform control --config configs/xuchuang/gripper.yaml --session artifacts/gripper_001 --execute
```

固定左后右，每侧CLOSE→10→20→40→60→80mm→OPEN；每点等待卡尺读数。OPEN/CLOSE可能影响两侧。v4保留TTY输入修复，仍可能遇到CLOSE后开口不符、到位超时或人工中断。不能把空读数当零，不删除失败检查。

## 5 相机

厂家人员移动机械臂；程序只申请流、记录状态并响应s/q按键。

```bash
python3 -m calibration_platform collect --config configs/xuchuang/cameras.yaml --session artifacts/camera_001 --camera left_wrist
python3 -m calibration_platform collect --config configs/xuchuang/cameras.yaml --session artifacts/camera_002 --camera right_wrist
python3 -m calibration_platform collect --config configs/xuchuang/cameras.yaml --session artifacts/camera_003 --camera head --tag-arm left
```

腕部拍固定大板；head拍固定在指定末端的ID0。停稳至少1秒后输入s回车。每路默认10拟合+5留出验证。只释放自己申请的会话，Image优先，无新鲜Image时取H264独立关键帧。相机依然保留原始H264调试流，可能较大；新关节采集则不再全量录音视频。

相机仍复用原v3安装关系和采样逻辑。换相机、板尺寸或消息协议不能只改名称；最后现场场次9/9/10个样本未达到15个目标。

## 6 结果目录

```text
artifacts/场次/
├─ session.json
├─ config/                  # 本次实际配置，变更后需新建场次
├─ health/                  # 采集心跳，不作为设备时间
├─ raw/
│  ├─ bags/                 # 现场 ROS 录包的存放位置
│  ├─ joints/{arms,waist}/  # messages.jsonl、states.jsonl及control/
│  ├─ events.jsonl          # 通用反馈采集和回放事件
│  ├─ gripper/
│  └─ cameras/
├─ derived/                 # 配对后的计算输入
├─ bundles/{urdf,usd,cameras}/
├─ reports/                 # 计算或验收时用 --output 指定输出
└─ manifest.sha256
```

要生成场次参数包，给 `compute --task export` 传入经验证的写回 JSON，并设置 `--output artifacts/场次/bundles`。成功后会生成 `calibration_bundle.yaml`，记录写入参数、模型文件及输入哈希。ROS 包、相机参数和验收报告在对应作业实际产生后归档。最后一次采集后又添加结果文件时，在所有写入结束后重新生成清单：

```bash
python -c "from calibration_platform.collect.writer import manifest; manifest('artifacts/场次')"
```

保存原始消息与SI解码行，缺失值为null；原始非有限浮点数以`nonfinite_float`标签保存并使采集失败。控制请求记录不等于实际接受目标回读。同设备同场次不覆盖已有原始日志。控制桥可能继续产生原格式CSV/YAML，暂保留作回归依据。

## 7 离线回放

使用少量真实现场消息验证解析和文件保存，不连接ROS、不运动：

```bash
python3 -m calibration_platform collect --config configs/xuchuang/arms.yaml --session artifacts/offline_001 --replay tests/samples/arms.jsonl
```

通用版发行包不带预填的运行配置，回归测试用`tests/configs/xuchuang/arms.yaml`替代上面的配置路径。测试数据只是少量接口样本，不是完整标定数据。

## 8 离线计算

```bash
python3 -m calibration_platform compute --config configs/xuchuang/acceptance.yaml --task gripper --input measured_points.json --output artifacts/gripper_fit.json
```

计算入口使用明确JSON输入，不默认把缺失外测值补成0。输出文件不覆盖。当前实际支持：

| task | 输入与输出 |
|---|---|
| static | 独立关节参考的零偏拟合，q_model=q_measured+offset；尚未实现全链路几何/TCP联合拟合 |
| dynamic | 同一时基单步目标/反馈的延迟、超调、稳定时间、误差；不伪造PD、额定上限、摩擦或惯量 |
| gripper | 指令毫米与独立卡尺值的仿射拟合及留出验证，不推断全行程/接触阈值 |
| intrinsics | 显式3D/2D角点的OpenCV内参拟合及留出验证，不推断深度比例 |
| handeye | 真实FK位姿和相机目标位姿，10拟合+5留出；或原samples.json配合`--frames robot_frames.json`从真实URDF求FK |
| scene | 同一固定变换多次独立观测的旋转/平移估计及留出误差 |
| export | 明确标记已验证的绝对URDF joint origin修改、已有USD属性的显式原生单位写回 |
| validate | 指标与acceptance阈值对比；阈值或指标缺失输出passed=null |

### 输入示例

夹爪JSON：`points`列表，每项有`slot`、`command_mm`、`measured_mm`、`split`（fit或holdout）。每侧至少3个不同fit测点和1个区间内holdout。

静态JSON：`reference_source`说明独立外测来源，`joint_names`及`samples`列表；每项`q_measured_rad`、`q_reference_rad`、`split`。至少3fit+1holdout。仅编码器数据不能拟合物理零偏。

动态JSON：`time_basis: "shared_clock_seconds"`，`samples`列表，每项`t_s`、`target_rad`、`position_rad`，时间严格递增，至少5点且只有一次目标阶跃。动作日志需正确配对后再用；不把计划创建时间自动当作真实接受时间。

内参JSON：`resolution: [宽, 高]`，`views`列表，每项`object_points_m`（N×3）、`image_points_px`（N×2）、`split`；至少8fit+3holdout。

手眼JSON：沿用vendor相机`samples.json`的`mode`、`samples`、`use_for_calibration`、`split`、`T_base_tool.matrix`和`T_camera_board.matrix`/`T_camera_tag.matrix`。无真实机器人位姿时用`--frames`指定真实URDF及轴映射。坐标约定T_A_B将B映射到A。

场景JSON：`parent_frame`、`child_frame`、`observations`，每项4×4`matrix`及`split`，至少3fit+1holdout。

验收JSON：`metrics`是指标名到非负误差的字典，对应acceptance.yaml中的同名thresholds。阈值未确认时不自动判通过。

### NVIDIA系统辨识与USD

已有项目中的官方SysID调用包装保留在`compute/nvidia_sysid.py`。它调用本机匹配的Isaac Sim 6.1官方工具，不自称实现了NVIDIA优化器：

```bash
python3 -m calibration_platform.compute.nvidia_sysid --help
```

实际调用需要`--isaac-python`、`--tools-dir`、`--run-spec`和`--output`，以及analyze/solve/apply/validate阶段。使用真实USD、实际遥测映射和UI导出的模型特定RunSpec；未知模型数据不自动填入。官方工具及Isaac运行时不随仓库分发，也未在当前电脑重新执行。

模型导出不擅自把SI增益塞入所有USD属性。`export`要求显式`validated: true`，输入`urdf.source`及`urdf.absolute_joint_origins`（xyz_m/rpy_rad），或`usd.source`及`usd.attributes`（prim_path/name/value/units=usd_native）。USD需要Isaac环境的pxr，白名单只改已有schema属性。导出的USD层引用原基础资产，基础资产和mesh引用必须保持可解析。相机USD自动写回、运行时延迟模型和真实仿真回放尚未实现；不通过自定义字段冒充生效。

## 9 来源与状态

现场标定平台来自项目的 sim2real_platform_20261112 交付版。原动作实现位于 `control/arms.py`、`control/waist.py` 和 `control/gripper.py`。相机实现保存在 `vendor/xuchuang/camera/`。Isaac 扩展工具取自项目的 zhifeng_sim2real_toolkit，NVIDIA 源码的 Apache 2.0 许可证随附在 `compute/isaac_tools/src/sysid/LICENSE`。

现场事实：左臂曾整臂运动成功，后续右臂单点成功；腰部反馈超时、夹爪CLOSE语义和相机样本不足仍未闭环。新架构未做Ubuntu/ROS/Isaac现场联调。计算候选不等于参数验收通过。

上传GitHub只包括代码、配置、说明、少量回归消息；不包含原始大包、CAD、机器人模型、账号凭据或第三方运行时。新机器人只在确有适配代码时登记，不创建空壳目录。
