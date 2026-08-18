# B2 Policy Deployment Handoff

更新日期：2026-08-11  
主仓库：`/home/zhangchi/b2_control_stack`

> **最新安全状态（2026-08-11，优先于本文后续历史记录）：** 真机实测没有走起来，后腿存在明显支撑不足；旧版控制器在测试结束/策略超时后进入 `Kp=0` 的 `SAFE_DAMPING`，导致机器人失去姿态支撑并摔倒。机器人网线现已由现场人员物理断开，控制进程已经停止。不得把此前的“链路通过”解释为“真机行走通过”，也不得直接重连后继续行走测试。正常结束失力问题已经在代码中改为 `SAFE_HOLD`，并在 unitree_mujoco 中通过 DDS 闭环验证；真机承重增益和后腿无力问题仍未解决。

## 1. 当前结论

目前已经完成：

- ONNX 策略接口和 45 维观测数学；
- MuJoCo sim-to-sim 验证；
- Unitree B2 真机 `LowState` 只读链路；
- 真机 learned-hold 与 `0.25 m/s` forward shadow 推理；
- 不连接电机发布链路的 C++ 限幅、插值和 `SAFE_DAMPING` 纯数据骨架；
- 一键真机只读验收、C++ 构建测试和 Python 回归测试。

目前仍未完成：

- 真机 `rt/lowcmd` 发布；
- `LowCmd` 字段和 CRC；
- 电机使能、原厂/自研模式切换；
- 经现场审核的 Kp/Kd、目标变化率、超时和保护参数；
- 急停、阻尼回退、失联回退；
- 吊桥支持下的零命令平滑接管和短时 `0.25 m/s` 实机测试。

因此当前状态仍是“真机只读与策略 shadow 验证通过”，不是“真机策略控制部署完成”。

## 2. 本轮做了什么

### 2.1 恢复 sim-to-sim 运行环境

系统原先缺少 `pip/venv`。本轮通过 Ubuntu 离线 wheel 组件在用户目录创建了隔离环境：

`/home/zhangchi/b2_sim2sim_env`

实际安装并验证的关键版本：

- `mujoco==3.3.6`
- `onnxruntime==1.28.0`
- `numpy==2.4.6`
- `PyYAML==6.0.3`
- `imageio==2.37.4`
- `imageio-ffmpeg==0.6.0`
- `pytest==9.1.1`

`pip check` 通过。

复现包：

`/home/zhangchi/文档/xwechat_files/wxid_zyi9sp7uwkvz22_2442/msg/file/2026-08/b2_sim2sim_repro_bundle(1) (2)/b2_sim2sim_repro_bundle`

关键 SHA-256：

- `models/policy.onnx`：`688bdbc4897c6dd9cc1cce54249131a35e9206614cfad2f383d2e907978b67eb`
- `sim2sim_b2/scenarios_oblique.yaml`：`54fe3f2db8eee8390e3b5ac6d3686e60c2d379426f1ea2cd68d4ed71c576b9fb`

本机 quick check 已实际运行成功：

- 场景：`oblique_stairs_up_12 / heading_000 / center`
- `provisional_pass=true`
- `termination_reason=course_complete`
- 仿真时间：`8.13 s`
- 前进距离：`2.94548 m`
- 最终 heading error：`0.01551 rad`
- 已生成 `summary.json`、`trajectory.csv`、场景 XML 和视频。

用户确认完整 sim-to-sim suite 已成功；复现包没有放历史视频是有意精简，不是包损坏。后续不应再把缺少 `reference_results` 视频视为部署卡点。

### 2.2 对成功复现包与主仓库做策略差分审计

对以下两套实现做了离线差分：

- 成功复现包：`sim2sim_b2/policy_adapter.py`
- 主仓库：`src/b2_control/policy_math.py`、`onnx_policy.py`

结果：

- 12 关节顺序完全一致；
- `Q_DEFAULT` 完全一致；
- action scale 完全一致；
- 1000 组随机状态的 observation 最大绝对差：`0`；
- joint target 最大绝对差：`0`；
- heading command 只存在约 `1.09e-14` 的浮点误差；
- 100 组随机 ONNX 输入的 action 最大绝对差：`0`。

结论：主仓库策略数学与成功 sim-to-sim 复现包一致。当前问题不在 45 维观测拼接、关节顺序、动作缩放或 ONNX 输出。

### 2.3 真机连接恢复和只读门禁

有线接口：

- 网卡：`enp8s0`
- 地址：`192.168.123.99/24`
- 链路：`1000 Mb/s`、full duplex
- DDS domain：`0`

机器人恢复连接后：

- `rt/lowstate` 约 `500 Hz`；
- CRC errors：`0`；
- invalid samples：`0`；
- 四元数范数正常；
- 原厂模式：`form=0, name=ai`，对应 `ai_sport`；
- 没有发布 `rt/lowcmd`；
- 没有切换原厂模式；
- 没有修改原厂遥控器按键逻辑。

### 2.4 一键真机只读验收

新增：

`scripts/run_hardware_readonly_acceptance.sh`

使用方式：

```bash
scripts/run_hardware_readonly_acceptance.sh enp8s0 \
  '/absolute/path/to/b2_policy_deployment_handoff'
```

脚本依次执行：

1. 只读 `CheckMode`；
2. 3 秒 `LowState`/CRC 门禁；
3. 10 秒 learned-hold shadow；
4. 10 秒 `0.25 m/s` forward shadow。

最近一次完整验收：

- 原厂模式：`form=0, name=ai`；
- 初始状态门禁：`1500` 帧，CRC=0，invalid=0；
- hold 状态流：`6000` 帧，CRC=0，invalid=0；
- hold shadow：`501` 样本；
- hold 最大推理时间：`1.442 ms`；
- hold 最大 action：`1.869`；
- hold 最大当前姿态到候选目标差：`0.417 rad`；
- hold 最小模型关节范围余量：`0.603 rad`；
- forward 状态流：`6000` 帧，CRC=0，invalid=0；
- forward shadow：`501` 样本；
- forward 最大推理时间：`1.397 ms`；
- forward 最大 action：`1.786`；
- forward 最大当前姿态到候选目标差：`0.396 rad`；
- forward 最小模型关节范围余量：`0.624 rad`。

结论：

- 状态传输、IMU、策略命令输入和 ONNX 推理有效；
- `0.396–0.417 rad` 的姿态跳变仍然阻止直接接管；
- 必须先做当前实测姿态到策略目标的连续插值和经审核的目标变化率限制。

记录已追加到：

`docs/hardware-readonly.md`

### 2.5 新增纯数据 C++ 安全骨架

新增文件：

- `config/b2-reference-profile.json`
- `src/utils/math_utils.hpp`
- `src/hardware_adapter.hpp`
- `src/hardware_adapter.cpp`
- `tests/cpp/test_control_data_layer.cpp`
- `tests/test_hardware_data_profile.py`

修改：

- `gateway/CMakeLists.txt`
- `docs/hardware-readonly.md`

#### 零运动配置

`config/b2-reference-profile.json` 当前明确设置：

- `status=reference_only`；
- `activation_permitted=false`；
- 12 关节 `q_min/q_max` 使用模型范围；
- `dq_max` 全零；
- `max_angle_step_rad` 全零；
- 初始硬件 `Kp/Kd` 全零；
- fallback `Kp=0`；
- fallback `Kd=3.0`；
- `Kd=3.0` 是用户指定值，仍标记为未获硬件批准。

零速度和零单帧步进保证该配置在审核前不能产生运动。

注意：仓库现在存在两个不同用途的配置文件：

1. `config/b2-reference-profile.json`：本轮新增的零运动 C++ 数据层骨架；
2. `configs/b2-reference-profile.json`：原有、完整的硬件审批门禁配置。

后续应由现场工程师将二者整合为一个经审核的权威配置，不能混用或只改其中一个。

#### `clip_command`

`src/utils/math_utils.hpp` 实现：

- 目标关节角上下限；
- `dq_max * dt` 速率限制；
- 单帧最大角度变化限制；
- 速率和单帧限制取更严格者；
- 拒绝 NaN/Inf；
- 拒绝无效上下限；
- 拒绝已经越界的上一帧目标，防止最终 hard clamp 绕过速率限制。

#### `linear_interpolate`

实现基于 `elapsed_s / duration_s` 的线性插值：

- 插值比例限制在 `[0,1]`；
- 拒绝非有限端点；
- 拒绝非正 duration；
- 可用于处理当前观测到的 `0.396–0.417 rad` 初始跳变。

插值时长和每关节速率当前没有擅自填写，必须由硬件工程师审批。

#### `HardwareAdapter`

当前是纯数据类，不连接 Unitree SDK，不创建 publisher。

实现：

- 状态：`NORMAL`、`SAFE_DAMPING`；
- 前两维安全向量绝对值大于 `0.6` 时锁存 `SAFE_DAMPING`；
- 前两维出现 NaN/Inf 时锁存 `SAFE_DAMPING`；
- action、Kp、Kd、tau 出现非有限值时锁存 `SAFE_DAMPING`；
- Kp/Kd 为负数时锁存 `SAFE_DAMPING`；
- `SAFE_DAMPING` 下强制：
  - `q_desired=0`；
  - `Kp=0`；
  - `Kd=3.0`；
  - `tau=0`；
- 安全状态不会自动恢复，避免故障消失后自动重新使能。

这只是数据转换骨架，不是 Unitree 真机阻尼命令实现。真实电机模式、CRC、publisher、失联处理和模式切换仍不存在。

### 2.6 构建和测试

CMake 新增：

- `b2_hardware_data` 静态库；
- `b2_control_data_test` 测试可执行文件；
- CTest 注册。

最终构建结果：

- `b2_hardware_data`：通过；
- `b2_sim_gateway`：通过；
- `b2_hw_probe`：通过；
- `b2_mode_probe`：通过；
- `b2_control_data_test`：通过。

测试结果：

- Python：`135 passed`；
- C++ CTest：`1/1 passed`；
- `git diff --check`：通过。

### 2.7 审计外部 ATEC 仓库

审计地址：

`https://github.com/StevenLiudw/Clear_ATEC2026_Simulation_Challenge/tree/agent/current-code-snapshot-2026-08-03`

固定快照：

`451e49efc24d7c8da1728e7ff5607348c36250ee`

审计结论：

该仓库是 Isaac Lab 仿真/比赛代码，不是真机 B2 底层部署层。

仓库中没有：

- `rt/lowcmd` publisher；
- `LowCmd` 字段初始化；
- LowCmd CRC；
- 站立目标插值；
- 真机 q/dq/Kp/Kd 命令；
- `LowState → 45维 observation`；
- Unitree SDK 模式切换；
- 真机通信超时、NaN、断线阻尼回退；
- ONNX Runtime 推理。

仓库中有、可以参考的部分：

- 从 Isaac Lab `proprio` 拼接 45 维策略观测；
- 12 腿关节顺序：`FR, FL, RR, RL × hip, thigh, calf`；
- 3 维速度命令张量或离线命令 schedule；
- 动作缩放；
- TorchScript `policy.pt` 推理；
- ONNX 导出；
- 仿真 `dt=0.005 s`、decimation=4，即 50 Hz 策略节拍。

不能把该仓库当作缺失的 Unitree 真机适配器。

## 3. 还剩什么

### 3.1 真机硬件层

以下仍未实现：

- `unitree_go::msg::dds_::LowCmd_` 完整初始化；
- `rt/lowcmd` publisher；
- 官方 CRC；
- B2 固件对应的 motor mode；
- q/dq/Kp/Kd/tau 字段发布；
- 500 Hz 或硬件批准周期的 publisher；
- 命令序号、时间戳、陈旧命令拒绝；
- publisher watchdog；
- 状态、策略、IPC、命令超时；
- 通信中断回退；
- 原厂 `ai_sport` 释放与重新获取；
- 原厂/自研模式切换确认；
- 电机使能/失能；
- 原厂 `L2+B` 阻尼行为保持；
- 物理急停和 dead-man；
- 电机温度、电池、故障码、倾角和角速度保护；
- 故障锁存后的人工确认复位。

这些必须由现场硬件工程师基于实际 B2 固件、SDK 和现场急停流程完成并审核。

### 3.2 硬件审批参数

运行：

```bash
env PYTHONPATH=src /home/zhangchi/b2_sim2sim_env/bin/python \
  -m b2_control.cli_check_parameters configs/b2-reference-profile.json
```

当前仍报告 20 个 blocker：

- `status` 未批准；
- `activation_permitted` 不是 true；
- position Kp；
- velocity Kd；
- gain ramp 时间；
- 12 关节 target slew；
- soft-limit margin；
- torque limit；
- current limit；
- 温度 trip/reset；
- tilt trip；
- angular-rate trip；
- state timeout；
- policy timeout；
- command timeout；
- publisher timeout；
- recovery timeout；
- reviewer；
- reviewed_at。

不得用以下数值直接填空：

- 仿真 `Kp=160/Kd=5`；
- URDF velocity limit；
- URDF effort limit；
- Unitree 示例的 `Kp=1000/Kd=10`；
- 用户指定但未验证的 fallback `Kd=3.0`。

### 3.3 观测映射剩余验证

已验证：

- 平放姿态四元数按 `wxyz` 解码合理；
- projected gravity 接近 `[0,0,-1]`；
- 关节槽顺序与策略顺序一致；
- 命令进入 observation；
- forward 与 hold action 有明显差异。

仍需现场夹具或支撑条件下验证：

- 正 roll 的 projected-gravity 符号；
- 负 roll；
- 正 pitch；
- 负 pitch；
- 三轴 gyro 方向；
- 每个关节正方向、单位和电机槽；
- 真实固件故障、温度、电池字段。

### 3.4 同状态 sim/real 对照

现有 sim/real 对照仍不是同一运动相位：

- sim： learned policy 行走帧；
- real：原厂 `ai_sport` 站立帧。

因此现有 RMSE 只能作基线，不能判定 sim-to-real 失败原因。

真正可比较的数据需要：

- 相同初始姿态；
- 相同 `vx,vy,wz`；
- 相同前一 action 初始化；
- 相同时间窗；
- learned policy 同时实际拥有 sim 和 real 的控制；
- 统一时间戳和逐关节记录。

这一项必须在获批硬件适配器和平滑接管之后完成。

## 4. 当前卡在哪里

1. 主策略和仿真没有发现数学错误；卡点已收敛到真机硬件适配与交接。
2. 当前原厂站姿到 learned-hold 目标最大差 `0.417 rad`，到 forward 目标最大差 `0.396 rad`；不能直接切换。
3. `configs/b2-reference-profile.json` 仍有 20 个审批 blocker。
4. 新增 `HardwareAdapter` 只是纯数据类，尚未加载 JSON，也未接入任何 Unitree `LowCmd` plugin。
5. 没有现场工程师签字的目标变化率、增益、温度、倾角、超时和回退参数。
6. 没有完成原厂 `ai_sport → custom → ai_sport` 的受控流程。
7. 没有完成吊桥/支撑、急停和短时低幅度接管测试。
8. fallback `Kd=3.0` 是用户指定值，未在当前 B2 上验证，不能因为写进 JSON 就视为安全批准值。

## 5. 踩过的坑

### 5.1 把缺少历史视频误判为包损坏

`SHA256SUMS` 列出历史 `reference_results`，但用户明确说明视频没有放进包，完整 suite 已成功。以后只校验实际交付的模型、场景、代码和资源，不应把有意省略的视频当作策略故障。

### 5.2 PyPI TLS 和系统缺 pip

最初系统没有 `pip/venv`，Python 官方引导安装遇到 `SSL: UNEXPECTED_EOF_WHILE_READING`。最终通过 Ubuntu `python3-pip-whl`、`python3-setuptools-whl`、`python3.12-venv` 创建用户级环境，再从 PyPI 安装锁定依赖。

后续应保留 wheel 缓存或准备正式离线依赖包。

### 5.3 不同运动相位不能直接比较

仿真末帧和真机原厂站立首帧直接相减会得到数值差，但不能定位 URDF、观测或策略问题。必须先对齐状态和相位。

### 5.4 原厂按键语义不能自行改写

`L2+B` 是原厂阻尼/失力相关逻辑，不等于普通的原厂 locomotion 恢复。自研程序不能拦截或重定义它。

### 5.5 仿真参数不是硬件批准参数

`Kp=160/Kd=5`、URDF velocity/effort limit 和示例 `Kp=1000/Kd=10` 都不是本机安全值。它们只能作为来源记录，不能直接进入真机 profile。

### 5.6 Release 构建会移除 `assert`

第一版 C++ 测试在 Release/`NDEBUG` 下移除了 `assert`，导致测试变量变成 unused 并被 `-Werror` 拦截。已经改为显式 `require(...)`，确保 Release 构建也真正执行检查。

### 5.7 hard clamp 可能绕过速率限制

第一版 `clip_command` 在 previous target 已经越界时，最后的 q clamp 可能产生大于单帧允许值的跳变。已经改为直接拒绝越界 previous target，而不是悄悄夹紧。

### 5.8 NaN 不能只依赖上游检查

第一版 C++ 骨架只根据倾角进入 `SAFE_DAMPING`。已经补充 action、Kp、Kd、tau 的有限值和非负增益检查；无效数据会锁存阻尼状态。

### 5.9 网线有载波不等于机器人 DDS 在线

曾出现：

- 网卡是 UP；
- 千兆链路存在；
- 但 `CheckMode=3104`；
- `LowState samples=0`；
- 邻居表为空。

机器人恢复连接后立即恢复约 500 Hz 状态流。因此网络诊断必须同时检查物理链路、地址、DDS 状态流和 mode service。

### 5.10 宿主文件隔离器故障

多次出现：

`bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted`

导致 `apply_patch` 和普通沙箱命令失败。经用户批准后在沙箱外完成读写；部分精确文件修改使用了带锚点检查的一次性 Python 写入。所有结果随后通过 `git diff`、编译和测试核对。

### 5.11 `config/` 与 `configs/` 容易混淆

用户要求的新文件位于 `config/b2-reference-profile.json`，原审批门禁文件位于 `configs/b2-reference-profile.json`。这是当前最容易造成错误配置来源的地方，后续必须统一。

## 6. 下一步怎么干

### 优先级 A：现场工程师完成审批输入

1. 确认 B2 固件版本、SDK commit、网络接口和消息/CRC 版本。
2. 在吊桥或可靠支撑下测试并批准：
   - 每关节 Kp/Kd；
   - gain ramp；
   - 每关节 target slew；
   - soft-limit margin；
   - torque/current limit；
   - 温度 trip/reset；
   - tilt/angular-rate trip；
   - 所有 watchdog timeout；
   - fallback 和恢复语义。
3. 填写 reviewer 和 reviewed_at。
4. 将 `config/` 与 `configs/` 两份配置合并为一个权威 profile。
5. 运行参数门禁直到无 blocker。

### 优先级 B：硬件工程师完成 Unitree plugin

必须实现并审核：

1. 只读 `LowState` snapshot；
2. `LowCmd` 全字段初始化；
3. CRC；
4. 固定周期 publisher；
5. stale state/policy/command 检查；
6. 当前 q 到第一个候选目标的连续插值；
7. `clip_command`；
8. gain ramp；
9. 温度、倾角、角速度、电池、故障保护；
10. 断线/超时/NaN 的锁存回退；
11. 原厂 `ai_sport` 释放、custom 接管和原厂恢复；
12. 物理急停和原厂 `L2+B` 行为保持。

现有 `HardwareAdapter` 可以作为数据层输入，但不能替代上述 plugin。

### 优先级 C：软件集成和故障注入

硬件 plugin 到位后：

1. 将 `LowState` 映射为明确的 12 关节和 IMU 数据；
2. 调用现有 `B2PolicyCore`；
3. 对候选 `q_desired` 先插值、再 q/rate/step 限幅；
4. 仅向获批 plugin 传递有限、未超时的数据；
5. 注入并验证：
   - state 超时；
   - policy 超时；
   - publisher 超时；
   - NaN/Inf；
   - 序号回退或重复；
   - DDS 中断；
   - ONNX 进程退出；
   - 倾角/角速度超限；
   - 温度/故障码；
   - 原厂恢复失败。
6. 每种故障必须锁存且不能自动重新使能。

### 优先级 D：现场接管顺序

1. 机器人吊桥或可靠支撑；
2. 原厂 `ai_sport` 稳定站立；
3. 再运行一键只读验收；
4. 验证物理急停和原厂阻尼；
5. custom armed，但不发布；
6. 以当前实测 q 为插值起点；
7. 零命令、零 tau、最低获批增益；
8. 按获批速率获取 learned-hold 目标；
9. 短时保持并人工复核日志；
10. 回到原厂 `ai_sport` 并再次 `CheckMode`；
11. 全部门禁通过后，才执行短时、低幅度 `0.25 m/s` 平地测试；
12. 首次测试不允许后退、楼梯、斜坡、无支撑或无人值守。

## 7. 常用命令

### Python 回归

```bash
env PYTHONPATH=src /home/zhangchi/b2_sim2sim_env/bin/python -m pytest -q
```

期望：`135 passed`。

### C++ 构建与测试

```bash
cmake -S gateway -B build/gateway \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/zhangchi/.local/unitree-sdk2-21d0a3b \
  -DBUILD_TESTING=ON
cmake --build build/gateway -j2
ctest --test-dir build/gateway --output-on-failure
```

### 真机只读验收

```bash
scripts/run_hardware_readonly_acceptance.sh enp8s0 \
  '/home/zhangchi/文档/xwechat_files/wxid_zyi9sp7uwkvz22_2442/msg/file/2026-08/b2_policy_deployment_handoff(1)/b2_policy_deployment_handoff'
```

该脚本没有 `LowCmd` publisher。

### 硬件参数门禁

```bash
env PYTHONPATH=src /home/zhangchi/b2_sim2sim_env/bin/python \
  -m b2_control.cli_check_parameters configs/b2-reference-profile.json
```

当前期望结果仍是 `BLOCKED`，直到现场工程师完成审核。

## 8. 关键文件

- `src/b2_control/policy_math.py`：45 维观测、命令、action scale；
- `src/b2_control/onnx_policy.py`：ONNX Runtime 包装；
- `src/b2_control/core.py`：策略状态和 previous action；
- `src/b2_control/cli_hw_dryrun.py`：真机只读 shadow；
- `gateway/b2_hw_probe.cpp`：LowState、CRC、50 Hz loopback mirror；
- `gateway/b2_mode_probe.cpp`：只读 `CheckMode`；
- `config/b2-reference-profile.json`：新零运动数据层配置；
- `configs/b2-reference-profile.json`：原硬件审批门禁配置；
- `src/utils/math_utils.hpp`：插值和限幅；
- `src/hardware_adapter.hpp`、`src/hardware_adapter.cpp`：纯数据安全状态；
- `scripts/run_hardware_readonly_acceptance.sh`：一键只读验收；
- `tests/cpp/test_control_data_layer.cpp`：C++ 数据层测试；
- `tests/test_hardware_data_profile.py`：零运动 profile 测试；
- `docs/hardware-readonly.md`：真机只读记录；
- `docs/sim-real-dryrun-comparison.md`：现有非相位对齐对照；
- `docs/b2-parameter-research.md`：参数来源边界。

## 9. 当前工作树状态

本轮变更尚未提交。当前主要变更：

- 修改：`gateway/CMakeLists.txt`；
- 修改：`docs/hardware-readonly.md`；
- 新增：`config/b2-reference-profile.json`；
- 新增：`src/utils/math_utils.hpp`；
- 新增：`src/hardware_adapter.hpp`；
- 新增：`src/hardware_adapter.cpp`；
- 新增：`tests/cpp/test_control_data_layer.cpp`；
- 新增：`tests/test_hardware_data_profile.py`；
- 新增：`scripts/run_hardware_readonly_acceptance.sh`；
- 新增/更新：`HANDOFF.md`。

接手者不要用 reset/checkout 丢弃这些未提交文件。提交前应再次运行 Python、C++ 和只读验收。

## 10. 最终交接结论

策略数学、ONNX 推理、仿真复现、真机状态流和 shadow 推理均已通过。外部 ATEC 仓库也不能补齐真机底层。

现在唯一主线不是继续改策略，而是：

1. 现场工程师提供并签字批准硬件参数；
2. 完成 Unitree `LowCmd/CRC/publisher/mode handover/watchdog` plugin；
3. 使用现有插值和限幅数据层进行吊桥下零命令平滑接管；
4. 通过故障注入和原厂恢复；
5. 最后才做短时 `0.25 m/s` 平地实机测试。

在这些条件满足前，不能把当前 C++ 骨架或 fallback `Kd=3.0` 解释为已经可安全上机。

## 16. 2026-08-11：完整 Policy → LowCmd controller 接入与 MuJoCo DDS 验证

按最新指令，真机候选控制参数先与训练仿真保持一致，不使用 B2
b2_stand_example.cpp 的站立演示增益 Kp=1000/Kd=10：

- 策略 PD：12 关节 Kp=160、Kd=5；
- 关节范围：B2 官方模型 q_min/q_max；
- 速度上限：hip/thigh 23 rad/s、calf 14 rad/s；
- LowCmd 周期：2 ms / 500 Hz；
- policy 周期：20 ms / 50 Hz；
- 初始姿态：从第一帧有效 LowState 到 policy default pose 做 1.0 s 线性插值；
- policy 目标在 500 Hz 发布侧做关节范围和单周期速度限幅；
- 状态或 policy 超过 100 ms 后进入锁存 SAFE_DAMPING：
  q=PosStopF, dq=0, Kp=0, Kd=5, tau=0。

已经接入的官方 B2 SDK 内容：

- unitree_go LowState / LowCmd；
- rt/lowstate subscriber 和 rt/lowcmd publisher；
- LowCmd 头、level、gpio 和 20 个 motor slot 默认初始化；
- B2 PMSM mode 0x0A；
- 官方 crc32_core 写入每一帧 LowCmd；
- 真机 domain 0 + 指定网卡入口；
- MotionSwitcher CheckMode/ReleaseMode；
- 仿真固定使用 domain 1 + lo，不调用 MotionSwitcher。

实际运行的 unitree_mujoco DDS 闭环：

LowState → 45维 observation → ONNX → 12维 action → q_target → 限幅/插值 → LowCmd

验证结果：

- 10 秒 SIM_PASS，502 次 policy 推理，最大推理时间 1.628 ms；
- LowState CRC errors=0，invalid LowState=0；
- controller 进入 POLICY，policy 结束约 100 ms 后进入 SAFE_DAMPING；
- 独立 DDS subscriber 5 秒收到 2500 帧 LowCmd，即 500 Hz；
- LowCmd CRC errors=0，字段错误=0；
- 20 个 motor slot 初始化正确，前 12 个 active Kp/Kd=160/5；
- C++ Release 构建通过，CTest 1/1，Python 135 passed，diff check 通过。

当前边界：

- 代码已具备真机 domain 0、MotionSwitcher、LowCmd 和 CRC 路径；
- 本轮没有在真机网卡启动 hardware 模式，没有发布真机 LowCmd；
- config/b2-reference-profile.json 保持 activation_permitted=false 和
  pending_double_check，下一步是代码 double check 后再做受控首次接管。
## 17. 2026-08-11：真机失败、摔倒根因和离线修复（最新）

### 17.1 实际结果

此前的“链路通过”只表示 LowState、ONNX、LowCmd、CRC 和控制周期可以运行，**不表示机器人已经走起来**。

- 真机 LowState 约 500 Hz，CRC error=0，invalid state=0；
- MotionSwitcher 的 ai 模式已释放，controller 完成 1 秒初始姿态插值；
- 5 秒 learned-hold：251 个 policy 样本，最大推理约 0.898 ms；
- 3 秒 vx=0.25 m/s：151 个样本，最大推理约 0.690 ms；
- 5 秒 vx=0.25 m/s：251 个样本，最大推理约 0.582 ms；
- 三次测试均未形成有效行走；后腿明显无力/下沉；
- 每次测试结束后机器人失去姿态支撑并摔倒。

结论：软件实时链路跑通，但物理 locomotion 失败。此前任何“0.25 m/s 真机测试通过”的表述均撤回。

### 17.2 已确认的摔倒根因

旧版 controller 把正常 policy 结束和 100 ms policy timeout 当成 `SAFE_DAMPING`，输出 `q=PosStopF, dq=0, Kp=0, Kd=5, tau=0`。落地承重时清零 Kp 会失去关节位置支撑，所以“测试结束就失力摔倒”是 controller 结束状态机错误，不是可接受现象。

真实 state/CRC/NaN 故障仍应锁存 SAFE_DAMPING；正常 policy 结束不能走同一路径。

### 17.3 已完成修复

`gateway/b2_sim_gateway.cpp`：

- policy 曾有效运行、随后正常停止或超时，保持 `last_target_` 并进入 `SAFE_HOLD`；
- 不再由正常结束触发 Kp=0 阻尼；
- state/CRC 等真实故障仍锁存 SAFE_DAMPING；
- 无故障的 controller 退出信号不再发送 50 ms 的 Kp=0 阻尼，而是短暂保持最后目标后停止 publisher；
- 只有已经锁存真实故障时，controller 退出阶段才继续发送 SAFE_DAMPING；
- LowCmd publisher 销毁后，尝试通过 MotionSwitcher 恢复原厂 ai 模式；
- 每秒记录 FR/FL/RR/RL 最大跟踪误差、最大 tau_est、最高电机温度和 motor lost 累计值。

`src/b2_control/cli_sim_policy.py`：

- 有限时长运动命令结束后先发 2 秒 learned-hold，再退出；
- controller 随后保持最后安全位置目标，而不是立即失力。

### 17.4 离线验证

真机网线断开期间，仅在 unitree_mujoco 中运行 DDS 闭环：2 秒 vx=0.25 m/s、2 秒 learned-hold、policy 退出。controller 从 POLICY 转入并持续保持 SAFE_HOLD，未再因正常结束进入 SAFE_DAMPING。

- SIM_PASS samples=202；
- 最大推理约 1.555 ms；
- controller 约 500 Hz；
- state CRC errors=0，invalid state=0；
- C++ Release 构建通过；Python 135 passed；CTest 1/1；diff check 通过。

这只证明“正常结束失力”路径已消除，**不证明 Kp=160/Kd=5 能在真机稳定承重或行走**。

### 17.5 当前安全状态和卡点

- 机器人网线已物理断开；没有残留 controller、policy 或仿真控制进程；
- 当前不会从本机继续发送 rt/lowcmd；
- Kp=160/Kd=5 从仿真直接搬到真机后承重或跟踪明显不足，后腿最突出；
- 已完成 MuJoCo 12关节 q_target/q_actual/error/tau_est 日志；真机逐关节日志因测试中止尚未完整保存，仍需在吊架下补采；
- Kp=1000/Kd=10 只是官方 B2 stand example 参考值，不能在无支撑条件下直接跳变使用；
- 机器人已摔倒，需要再次接管前检查机械、电池、报警和后腿执行器。

### 17.6 下一步（必须按顺序）

1. 保持网线断开，检查摔倒后的结构、关节报警、电池和后腿执行器。
2. 使用已加入 controller 的最小诊断日志，检查逐腿目标/实测误差、tau_est、最高温度和 lost 计数；如需定位到单个关节，再扩展为 CSV。
3. 使用可靠吊架或腹部支撑，确保完全失力也不会摔落。
4. 重连后先做 LowState 只读门禁，不发布 LowCmd，核对 CRC、零位、温度和故障码。
5. 只做站立承重测试，不运行 ONNX gait、不发 vx；从当前实测 q 接管并缓慢 ramp gain。
6. 分级调 Kp/Kd并记录后腿误差和 tau_est；不得从 160/5 直接跳到 1000/10。
7. 在吊架下分别验证 SAFE_HOLD、人工切回原厂 ai、真实故障阻尼，三种结束路径都不能摔倒。
8. 只有站立承重、结束保持、原厂恢复和后腿跟踪全部通过后，才允许再次做短时 vx=0.25 m/s；首次重试必须保留防跌落保护。

负责人最简结论：**真机没有走起来；结束摔倒的确定性软件原因已修复并在仿真验证，但后腿无力和真机承重参数尚未解决，因此当前禁止直接继续行走测试。**
## 18. 2026-08-11：裸 Q_DEFAULT 站立异常代码分析

新增负责人文档：

`docs/b2-stand-target-root-cause.md`

文档提取了 controller、policy target、MuJoCo PD bridge、逐关节诊断和官方 B2 example 的相关代码，并记录真机/仿真对照。最新定位是：当前状态机在首帧 policy target 到达前释放原厂 ai，并把裸 Q_DEFAULT 当作静态承重目标；而真正 learned-hold 目标是 Q_DEFAULT + ACTION_SCALE × policy action。下一步应先完成 learned-hold shadow 和首批 target 门禁，再释放 ai 并插值到 policy target，不能直接提高 Kp。



## 19. 2026-08-11：仿真接管与行走误差优化

### 19.1 接管修复

- 真机路径在释放原厂 ai 前等待 fresh LowState 和首个 fresh learned-hold policy target，未满足门禁时不发布主动 LowCmd；
- MuJoCo 没有原厂运动服务，仿真路径用第一帧 LowState 做极短预保持，收到第一帧有效 policy 后立即按关节速率限制执行；
- 仿真不再空跑 policy 或冻结第一帧 policy target，避免 observation 的 last_action 与实际执行目标不一致；
- policy 超时和正常退出继续保持 last_target，只有 CRC/NaN/LowState 故障进入 SAFE_DAMPING。

### 19.2 误差优化

`src/b2_control/core.py` 增加可配置 policy target slew。第一帧从实测 q 开始，8 rad/s 只作用在 policy 之后的安全目标；observation 的 previous_action 必须继续使用 bundle 合同要求的上一帧 raw actor output。此前短暂改成限速后动作的实现已被证明会压坏步态并已撤销。`cli_sim_policy.py` 默认使用 8 rad/s。没有修改训练动作缩放、关节顺序或官方 Kp=160/Kd=5。

`src/b2_control/onnx_policy.py` 增加控制循环前的零输入 session warmup，消除首次推理冷启动。hardware read-only dry-run 使用同一 target slew 和 ONNX warmup，但这不代表允许真机激活。

同一测试：2 秒 learned-hold、5 秒 vx=0.25 m/s、2 秒 learned-hold，随后 SAFE_HOLD。

- 基线：峰值 tracking error 0.803 rad；峰值 tau_est 125.351 Nm；最大循环内推理 33.958 ms；
- 8 rad/s + warmup：峰值 tracking error 0.625 rad，下降 22.1%；峰值 tau_est 98.924 Nm，下降 21.1%；最大循环内推理 0.462 ms；
- 6 rad/s：峰值误差回升到 0.791 rad、tau_est 127.101 Nm，说明限速过低会拖延步态相位，已经否决；
- 最终候选为 8 rad/s + ONNX warmup；Python 139 passed，diff check 通过；CRC error=0、invalid state=0、无 NaN/通信故障。

最终候选日志：

- `logs/sim-walk-slew8-warm-policy-5s-20260811.log`
- `logs/sim-walk-slew8-warm-controller-5s-20260811.log`
- `logs/sim-walk-slew8-warm-mujoco-5s-20260811.log`

### 19.3 仍然存在的限制

PD 位置误差同时承担生成承重力矩的作用，不能在保持 Kp=160/Kd=5 的同时强行压到接近零。继续降低 target slew 已被 6 rad/s 试验证明会恶化步态；提高 Kp 或加入重力/力矩前馈会改变训练控制律和真机风险，本轮没有采用。当前改进只在 MuJoCo 验证，`activation_permitted` 仍为 false，不能据此直接进入真机行走。


## 20. 2026-08-11：操作员授权后的吊架真机短时行走测试

用户明确授权真机行走后，先完成完整只读门禁：MotionSwitcher `form=0 name=ai`；LowState 约 482–500 Hz；state gate、learned-hold shadow 和 0.25 m/s forward shadow 均为 CRC error=0、invalid=0。hold shadow 最大 candidate gap 0.400 rad，forward shadow 0.373 rad；最大推理分别约 0.964 ms 和 1.009 ms。

验收期间修复了 `b2_hw_probe` 打印 PASS 后在 CycloneDDS subscriber 析构时断言退出的问题：析构函数现在显式调用 `CloseChannel()`。同时在硬件 controller 中接入投影重力倾倒门禁，body projected-gravity 的 x/y 任一绝对值超过 0.6 即锁存故障。构建、CTest 1/1、Python 139 passed。

吊架测试阶段：3 秒 learned-hold warmup、1 秒 `vx=0.25, vy=0, wz=0`、2 秒 learned-hold，然后 controller 持续 SAFE_HOLD。policy 共 242 samples，最大推理 1.924 ms。controller CRC error=0、invalid=0、motor lost=0，最高电机温度 37°C；观测到的最大逐腿 tracking error 为 0.542 rad，最大 tau_est 81.656 Nm。结束 SAFE_HOLD 稳定时逐腿误差约 `[0.230, 0.223, 0.174, 0.205]` rad，关节速度接近 0。正常结束没有发送 Kp=0。

有序退出后 MotionSwitcher 已恢复 `form=0 name=ai`。退出后 2 秒只读复检收到 997 个有效 LowState 样本（498–499 Hz），CRC/invalid 为 0，没有残留 controller/policy。

重要边界：当前 LowState/诊断没有机身世界坐标位移，因此软件只能证明 `vx=0.25` 命令进入 policy、policy/LowCmd/电机跟踪链路运行并安全收尾，不能仅靠这份日志证明机器人产生了有效前进位移。是否真正走动仍需现场观察或增加 MuJoCo/真机外部定位。禁止把本次结果写成“真机行走成功”，除非现场人员明确确认。


## 21. 2026-08-11：真机未移动确认与 raw-action history 修正

现场明确确认第 20 节的 1 秒 `vx=0.25` 真机测试没有发生移动。因此第 20 节只能算链路/承重/安全退出通过，真机 locomotion 失败。

根因检查发现：为降低 tracking error，`B2PolicyCore` 曾把 observation `33:45` 的 previous_action 改成限速后动作；但 bundle 的 `POLICY_CONTRACT.md` 明确要求这里必须是上一帧 raw actor output，关节限位/target slew 应在 actor 之后执行。错误回填会改变策略内部步态状态，真机测试使用的正是这个错误版本。

已修正：

- `previous_action` 恢复为上一帧 raw ONNX actor output；
- 8 rad/s target slew 保留，但仅作用在 policy 输出之后的 q target；
- 增加运动开始 command/action/q_target，以及整个运动阶段 action/target peak-to-peak 诊断；
- MuJoCo 增加每秒 base qpos x/y/z 和累计 dx/dy 日志；
- Python 139 passed。

修正后同一 MuJoCo 测试（2 秒 hold、5 秒 vx=0.25、2 秒 hold）：

- 运动首帧 command `[0.25, 0, 0.0023]`；
- 251 个运动 policy 样本；
- 12 关节 raw action 峰峰值约 1.33–7.29，target 峰峰值约 0.33–1.72 rad，步态振荡明确存在；
- 约 sim time 2.002 到 7.004 秒，base x 从 0.497292 m 到 1.182423 m，前进 0.685131 m；
- 同期横向位移 -0.017537 m，平均前进速度约 0.137 m/s；
- 最大推理 0.673 ms。

日志：

- `logs/sim-walk-raw-history-base-policy-5s-20260811.log`
- `logs/sim-walk-raw-history-base-controller-5s-20260811.log`
- `logs/sim-walk-raw-history-base-mujoco-5s-20260811.log`

当前真机已经恢复原厂 `form=0 name=ai`，无残留 controller/policy。修正后的版本尚未再次上真机；下一次只能在吊架下重新做短时测试，并以现场实际位移为成功标准。


## 22. 2026-08-11：raw-action history 修正后的真机 2 秒复测

已在可靠防跌落条件下执行修正版短测：3 秒 learned-hold warmup、2 秒 `vx=0.25 m/s`、2 秒 learned-hold，随后进入 SAFE_HOLD。测试使用 observation `33:45 = previous raw actor output`，8 rad/s target slew 仅位于 actor 后的关节目标安全层。

链路和输入确认：

- 运动首帧 policy command 为 `[0.2485, 0, -0.043]`；`heading=0.350 rad` 是启动时锁存的世界航向，小的 `wz` 是合同规定的航向误差修正，不是额外 0.35 rad/s 转向命令；
- 运动阶段 101 个 policy 样本，12 维 raw action 峰峰值约 `0.289..1.748`，12 关节 target 峰峰值约 `0.036..0.386 rad`；
- 最大推理时间 1.792 ms；controller 约 500 Hz；
- state CRC error=0、invalid state=0、motor lost=0，最高电机温度 36°C；
- 正常结束进入 SAFE_HOLD，没有走 Kp=0 的 SAFE_DAMPING 路径；
- 退出后只读检查为 `form=0 name=ai`，LowState 连续 2 秒 500 Hz，共 1000 个有效样本，CRC/invalid 均为 0，无残留 controller/policy。

本次暴露的主要差异：

- 真机运动阶段 action/target 变化明显小于修正版 5 秒 MuJoCo 行走：真机 action 峰峰值最大 1.748、target 最大 0.386 rad；MuJoCo action 峰峰值约 1.33..7.29、target 约 0.33..1.72 rad；测试时长不同，不能只按峰峰值直接定因，但真机策略更快收敛到近静态承重状态；
- 交接插值末段 RL 后腿峰值 tracking error 0.916 rad、估算 tau 123.844 Nm，是本轮最大瞬态，发生在正式 POLICY 行走段之前；
- POLICY 阶段逐腿峰值 tracking error 约 `[0.393, 0.246, 0.149, 0.444] rad`，估算 tau 峰值约 `[63.0, 39.3, 23.7, 69.5] Nm`；
- SAFE_HOLD 稳态仍存在约 `[0.339, 0.229, 0.152, 0.235] rad` 的逐腿最大位置误差，说明实体承重下 `Kp=160/Kd=5` 的静态偏差较大，不能把链路通过等同于有效步态。

日志：

- `logs/hw-walk-raw-history-policy-2s-20260811.log`
- `logs/hw-walk-raw-history-controller-2s-20260811.log`

当前结论：修正后的真机 policy → LowCmd 链路确实收到了非零前进命令并产生了时变动作，通信和安全收尾正常；现场随后明确确认本轮仍然没有位移，因此真机 locomotion 继续判定失败。下一步不应继续盲目延长行走时间，应先降低 learned-hold 交接瞬态，并对照真机与 MuJoCo 的 projected gravity、角速度、关节状态和 action 时间序列，定位导致真机 policy 收敛到近静态状态的 observation/执行器闭环差异。


## 23. 2026-08-11：无位移复核与仿真参数锁定

现场确认第 22 节的修正版 2 秒真机测试仍无位移。同样的 `3 秒 hold → 2 秒 vx=0.25 → 2 秒 hold` 在原始 MuJoCo `Kp=160/Kd=5` 中，100/101 个运动样本已产生明显周期动作；真机 action/target 峰峰值显著更小，且关节速度接近零并长期保留较大承重误差。当前定位从通信/ONNX 链路进一步收窄到真机受载执行器响应与训练仿真闭环不一致。

曾只在 MuJoCo 临时评估两组非训练增益：

- 官方 B2 stand example 的 `1000/10` 会破坏该按 `160/5` 训练的动态 policy 闭环，仿真机身倒下，明确否决；
- `240/6` 的首次即时接管起身明显不稳；增加 3 秒插值后起身改善并能产生步态，但 POLICY 瞬态最大 tracking error 约 0.623 rad、tau_est 约 154 Nm，不能据此直接进入真机行走。

用户随后明确要求仿真参数保持原样。最终代码已经：

- 将标准 MuJoCo 固定回训练原值 `Kp=160/Kd=5`；
- 保留原始 simulator 即时关节限速接管路径；
- 删除临时 `--sim-gains` 和 `--sim-official-b2-gains` 覆盖入口，避免后续误用非训练参数解释仿真结果；
- 重新构建通过，CTest 1/1、Python 139 passed；
- 当前无残留 MuJoCo、policy 或 controller 进程，未继续真机接管。

边界：仿真参数不再作为调参对象。下一步若继续修真机，应把硬件增益/接管策略作为独立硬件层问题处理，同时始终用原始 `160/5` MuJoCo 作为不变基线。


## 24. 2026-08-11：240/6 真机零命令接管验证

用户再次授权真机接管后，仅执行 learned-hold 承重验证，没有发送行走命令。标准 MuJoCo 仍固定原始 `Kp=160/Kd=5`；本轮 `Kp=240/Kd=6` 只属于真机硬件层候选。

接管前门禁：

- MotionSwitcher `form=0 name=ai`；
- 3 秒 LowState 收到 1497 帧，约 498–500 Hz；
- CRC error=0、invalid sample=0，无残留 controller/policy。

执行流程：

- policy 全程 `command=[0,0,0]`；
- 释放原生 `ai` 后，从实测姿态向最新 learned-hold target 做 3 秒插值；
- policy 共 442 个样本，最大推理 1.802 ms；
- 正常结束进入 SAFE_HOLD，没有进入 Kp=0 的 SAFE_DAMPING。

诊断结果：

- 插值阶段后腿最大 tracking error 约 RR 0.540 rad、RL 0.482 rad，最大 tau_est 约 RR 128.8 Nm、RL 114.2 Nm；
- learned-hold 稳态逐腿最大误差约 `[0.148, 0.116, 0.190, 0.223] rad`；
- 稳态逐腿最大 tau_est 约 `[36, 28, 46, 54] Nm`，关节速度接近 0；
- CRC error=0、invalid state=0、motor lost=0，最高电机温度 44°C；
- 与先前真机 `160/5` SAFE_HOLD 约 `[0.339, 0.229, 0.152, 0.235] rad` 相比，前腿误差明显下降，但 RR/RL 后腿没有形成一致改善，因此不能据此进入行走。

退出后只读确认已经恢复 `form=0 name=ai`；2 秒收到 1000 个有效 LowState 样本，CRC/invalid 均为 0，无残留 controller/policy。

日志：

- `logs/hw-hold-gains240-6-policy-10s-20260811.log`
- `logs/hw-hold-gains240-6-controller-10s-20260811.log`

当前结论：零命令真机接管、承重、SAFE_HOLD 和原生恢复链路正常，但 240/6 对后腿误差的改善不足，且交接阶段仍有约 129 Nm 峰值。当前不应直接进入真机行走。
现场随后明确确认本轮“站立没有问题”，因此站立承重人工门禁通过；这不自动等同于行走通过。


## 25. 2026-08-11：240/6 真机 2 秒行走测试

在第 24 节站立人工确认后，以真机硬件层 Kp=240/Kd=6 执行：5 秒 learned-hold（包含 3 秒实测姿态插值）、2 秒 vx=0.25 m/s、2 秒 learned-hold，随后 SAFE_HOLD。

- 运动首帧 command 约 [0.2479, 0, 0.0503]；
- 运动阶段 100 个样本，action 峰峰值约 0.438..2.422，target 峰峰值约 0.110..0.588 rad；
- POLICY 阶段观察到实际关节动态，典型 thigh dq 约 0.58..0.63 rad/s；
- CRC error=0、invalid state=0、motor lost=0，最高温度 44°C；
- 正常结束进入 SAFE_HOLD；随后恢复 form=0 name=ai，2 秒复检 1000 个有效样本，无残留进程。

LowState 不提供世界坐标，软件侧不能判定地面位移；现场位移结果仍待操作员确认。

日志：

- logs/hw-walk-gains240-6-policy-2s-20260811.log
- logs/hw-walk-gains240-6-controller-2s-20260811.log


## 26. 2026-08-11：切回 160/5 的真机 2 秒与 5 秒对照

按用户要求，只把真机硬件层切回 Kp=160/Kd=5；标准 MuJoCo 始终保持训练原值 160/5，3 秒 measured-pose→learned-hold 插值、安全限幅、SAFE_HOLD 和真实故障 SAFE_DAMPING 均未改变。

### 26.1 2 秒运动段

流程为 5 秒 learned-hold、2 秒 vx=0.25 m/s、2 秒 learned-hold。运动首帧 command 约 [0.2442, 0, 0.0841]；100 个运动样本，action 峰峰值约 0.633..3.286，target 峰峰值约 0.158..0.765 rad，实际关节速度峰值日志约 3.33 rad/s。插值阶段最大逐腿误差约 0.848 rad、最大 tau_est 约 130 Nm。最高温度 47°C，CRC/invalid/lost 均为 0。结束 SAFE_HOLD 后恢复 ai，复检 1000 个有效 LowState 样本，无残留进程。

日志：

- logs/hw-walk-gains160-5-policy-2s-20260811.log
- logs/hw-walk-gains160-5-controller-2s-20260811.log

### 26.2 5 秒运动段

即时门禁为 form=0 name=ai，2 秒收到 999 个有效 LowState 样本，CRC/invalid 均为 0。随后执行 5 秒 learned-hold、5 秒 vx=0.25 m/s、2 秒 learned-hold。

- 运动首帧 command 约 [0.2465, 0, 0.0654]；
- 运动阶段 250 个样本；action 峰峰值约 0.715..3.226，target 峰峰值约 0.179..0.807 rad；
- 插值阶段最大逐腿误差约 0.797 rad、最大 tau_est 约 123 Nm；
- SAFE_HOLD 稳态逐腿最大误差约 [0.386, 0.230, 0.169, 0.200] rad；
- CRC error=0、invalid state=0、motor lost=0，最高温度 48°C；
- 正常结束进入 SAFE_HOLD，随后恢复 form=0 name=ai；接管后 2 秒复检 999 个有效样本，CRC/invalid 均为 0，无残留 controller/policy。

只读探针两次在打印 PASS 后的 CycloneDDS subscriber 析构阶段触发断言；有效状态已经完整采集，且 pgrep 无残留进程，但该 teardown 回归仍需单独修复。

日志：

- logs/hw-walk-gains160-5-policy-5s-20260811.log
- logs/hw-walk-gains160-5-controller-5s-20260811.log

当前边界：两轮软件链路、真实关节动态和安全退出均通过，但没有机身世界坐标传感器，能否称为真机行走仍必须以现场实际位移确认。当前真机参数保持 160/5。

## 27. 2026-08-11：160/5 三轮真机重复性复测

按用户要求连续执行三轮同条件真机测试，每轮均为 5 秒 learned-hold、5 秒 vx=0.25 m/s、2 秒 learned-hold，使用 Kp=160/Kd=5、8 rad/s policy target slew 和 3 秒 measured-pose 接管插值。

- 第 1 轮从原厂 ai 深蹲姿态开始，四个 calf 约 -2.77..-2.80 rad；后腿 target 峰峰值仅约 0.032..0.102 rad，policy 很快收敛到近静态。
- 第 2 轮从正常站姿开始，RR target 峰峰值约 [0.249, 0.188, 0.604] rad，RL 约 [0.240, 0.196, 0.727] rad。现场确认左后腿有前进动作，右后腿只略微抬起。
- 第 3 轮从正常站姿开始，RR target 峰峰值约 [0.210, 0.204, 0.558] rad，RL 约 [0.227, 0.176, 0.544] rad。现场观察仍是左侧动作、右侧轻微抬起，但比第 2 轮更弱。
- 第 2 轮动态日志中曾同时看到 RL_calf dq 约 -1.61 rad/s、RR_calf 约 0.16 rad/s，和现场左右不对称一致；这只是 1 Hz 诊断采样点，仍需高频 CSV 量化。
- 三轮 CRC error=0、invalid state=0、motor lost=0，最高温度 38°C；全部正常进入 SAFE_HOLD，没有 SAFE_DAMPING。
- 最终恢复 form=0 name=ai；2 秒复检收到 999 个有效 LowState 样本，无残留 controller/policy/MuJoCo/录像进程。

三轮日志：

- logs/hw-repeat-r1-gains160-5-policy-5s-20260811.log
- logs/hw-repeat-r1-gains160-5-controller-5s-20260811.log
- logs/hw-repeat-r2-gains160-5-policy-5s-20260811.log
- logs/hw-repeat-r2-gains160-5-controller-5s-20260811.log
- logs/hw-repeat-r3-gains160-5-policy-5s-20260811.log
- logs/hw-repeat-r3-gains160-5-controller-5s-20260811.log

当前结论：左右后腿收到的 policy 目标幅度处于同一量级，但真实动作可重复地表现为左侧强、右侧弱，因此不能用软件 q/dq limit 或“policy 没命令后腿”解释。卡点进一步收窄到右后腿受载闭环响应、内部电流/力矩限制、机械阻力或负载分配。motor lost=0、温度正常只能排除通信丢失和明显过温，不能排除饱和或机械问题。下一步应先增加 500 Hz rear q_target/q/dq/tau_est CSV，再在可靠支撑下做小幅左右后腿逐关节响应对照，不应继续用完整 gait 盲测。


## 28. 2026-08-11：关闭两级速率限制 + action clip ±3 的六轮复测与录像诊断

### 28.1 测试配置与安全结果

六轮均使用真机 Kp=160/Kd=5、5 秒 learned-hold、4 秒前进命令、2 秒 learned-hold；policy-side target slew 与 gateway-side target rate limit 均关闭，actor applied action 限制在 [-3,+3]，机械关节角硬边界、CRC/NaN/LowState 超时、倾倒回退、结束 SAFE_HOLD 和 MotionSwitcher 恢复仍保留。

第 4–6 轮均完成，未出现 SAFE_DAMPING、CRC 或 invalid state；motor_lost_sum 从首帧到结束始终为累计值 5，没有新增。每轮结束均恢复 form=0 name=ai。全部完成后再次只读复核：LowState 约 499–500 Hz，2 秒 999 帧，CRC/invalid 均为 0；无残留 controller/policy。

录像：`/home/zhangchi/文档/xwechat_files/wxid_zyi9sp7uwkvz22_2442/msg/video/2026-08/b624ca925ffe780c6a6475a7cf1ea27f.mp4`。录像确认机器人在运动阶段身体下沉和偏摆，测试结束切回 ai 后才重新站稳；现场“表现更差”属实，不是仅由统计口径造成。

### 28.2 主要数据结论

- 运动段的 [-3,+3] clip 不是主要原因：第 4、6 轮均为 0 次，第 5 轮仅 3/2400 个关节时刻触发。
- 取消速率限制后策略目标明显增大，但真机小腿没有同步执行。第 4–6 轮四个 calf 的 target p2p 约 0.606..1.273 rad，actual/target 多数只有 0.14..0.43。
- 第 5 轮初始偏航引发 heading helper 输出瞬时 command [0.138,0,0.423]，该轮运动段平均 vx=0.232、平均 wz=0.108；第 6 轮平均 wz=-0.084。这个航向修正会放大左右不对称，但第 4 轮 wz 仅约 -0.011 仍失败，因此它不是唯一根因。
- learned-hold warmup 阶段 clip 命中数 R1..R6 分别为 10、319、89、395、702、130。训练配置 `clip_actions: null`，bundle 合同要求 previous_action 使用 raw actor output；这些大量 warmup outlier 说明多轮接管首姿态经常已离开策略训练附近状态。

### 28.3 已定位的重复测试缺口

自动循环此前只检查 MotionSwitcher=ai、LowState 频率、CRC/invalid，没有检查 ai 是否已经把机器人恢复到适合 policy 接管的稳定站姿。六轮 policy 收到的首帧相对 Q_DEFAULT 偏差如下：

| Round | 首帧 mean abs(q-Q_DEFAULT) | 最大单关节偏差 | 判定 |
|---|---:|---:|---|
| R1 | 0.603 rad | 1.303 rad | 深蹲，不应接管 |
| R2 | 0.122 rad | 0.337 rad | 可接管 |
| R3 | 0.609 rad | 1.275 rad | 深蹲/偏姿，不应接管 |
| R4 | 0.152 rad | 0.421 rad | 可接管 |
| R5 | 0.597 rad | 1.011 rad | 明显左右不对称，不应接管 |
| R6 | 0.611 rad | 1.201 rad | 深蹲/偏姿，不应接管 |

因此六轮中只有 R2、R4 起点可比。R5、R6 是在前一轮结束后机器人尚未恢复合格站姿时再次释放 ai 接管，直接解释了连续复测为什么越来越差，也说明此前的自动预门禁不足。

### 28.4 已完成修复

`gateway/b2_sim_gateway.cpp` 已在硬件 MotionSwitcher ReleaseMode 之前加入 policy-ready standing pose 门控：

- mean abs(q-Q_DEFAULT) <= 0.25 rad；
- max abs(q-Q_DEFAULT) <= 0.55 rad；
- max abs(dq) <= 0.20 rad/s；
- 三项连续满足 0.5 秒才允许释放 ai。

门控不合格时 controller 只等待并输出指标，不发布 active LowCmd；10 秒仍不合格则退出。阈值按已有六轮数据选择，会放行 R2/R4，拦截 R1/R3/R5/R6。修改已重新构建，CTest 1/1、Python 142/142 通过，尚未再次上机。

### 28.5 当前卡点与下一步

当前卡点仍是实体承重闭环跟踪：即使从合格站姿开始（R4），取消速率限制后策略发出的 calf 目标也只被执行约 14%..33%，录像表现为下沉而非稳定步态。通信、CRC、ONNX 推理和结束恢复不是当前主因；±3 motion clip 也不是主因。

下一步必须停止无姿态门控的连续 gait 盲测。先在门控通过的同一正常站姿下做 learned-hold/短运动对照，并把 gateway 已有的 q_target/q/dq/tau_est 从 1 Hz 扩展到 50–500 Hz CSV，重点比较四个 calf 和左右后腿的力矩饱和/跟踪滞后。若合格起点下仍出现 calf actual/target <0.5 和身体下沉，应回到硬件执行层/负载分配排查，而不是继续放大 policy target 或延长行走时间。

负责人一句话结论：**录像确认取消两级速率限制后真机更差；主要新发现是连续测试经常从深蹲或扭转姿态重新接管，现已在释放 ai 前加入并验证站姿门控，但合格站姿下小腿目标仍只执行约 14%..33%，所以真机步态尚未修好，当前不应继续盲跑。**


## 29. 2026-08-11：站姿门控实测与 50 Hz 零命令承重诊断

### 29.1 50 Hz controller CSV

`gateway/b2_sim_gateway.cpp` 新增可选 `--diagnostics-csv PATH`。controller 在 500 Hz LowCmd 循环中每 10 tick 记录一次，字段包含 controller mode、state/command sequence、12 关节 q_target、q、dq、tau_est、temperature 和 lost。修改重新构建通过，CTest 1/1、Python 142/142 通过。

### 29.2 门控首次正确拦截

第一次尝试时机器人虽然为 form=0 name=ai，但仍处于原生深蹲：12关节 mean abs(q-Q_DEFAULT)=0.599 rad、max=1.303 rad。新门控等待 10 秒后在 ReleaseMode 之前退出；没有发布 active LowCmd，也没有释放 ai。这验证门控能阻止此前 R1/R3/R5/R6 类型的错误连续接管。

随后使用仓库内宇树官方 B2 SportClient 示例发送原生 `BalanceStand()`，q0 从约 -0.306 rad 恢复到 -0.016 rad；3 秒只读 LowState 1495 帧，CRC/invalid=0。第二次门控稳定值为 mean=0.123 rad、max=0.338 rad、max dq<0.001 rad/s，连续 0.5 秒通过后才释放 ai。

### 29.3 8 秒 learned-hold 结果

本轮使用标准两级速率保护：policy target slew=8 rad/s、gateway 官方关节速率限制；Kp=160/Kd=5；command 全程 [0,0,0]；未启用实验 action clip；未发送 vx。controller CSV 共 417 个 50 Hz 样本：INTERPOLATE 150、POLICY 170、SAFE_HOLD 97。

插值阶段的高频峰值：

| Joint | error max | abs tau_est max | abs dq max |
|---|---:|---:|---:|
| FR calf | 0.688 rad | 102.9 Nm | 2.28 rad/s |
| FL calf | 0.406 rad | 65.0 Nm | 1.19 rad/s |
| RR calf | 1.093 rad | 172.9 Nm | 2.30 rad/s |
| RL calf | 0.682 rad | 108.1 Nm | 1.48 rad/s |

稳定 POLICY learned-hold：

| Joint | error MAE | error p95/max | abs tau_est p95/max | target/actual p2p |
|---|---:|---:|---:|---:|
| FR calf | 0.183 | 0.209/0.307 | 33.3/48.3 Nm | 0.424/0.055 rad |
| FL calf | 0.276 | 0.367/0.394 | 58.4/63.1 Nm | 0.148/0.052 rad |
| RR calf | 0.434 | 0.542/0.570 | 85.9/90.8 Nm | 0.157/0.046 rad |
| RL calf | 0.298 | 0.347/0.454 | 55.2/72.1 Nm | 0.158/0.068 rad |

SAFE_HOLD 末段四个 calf 静态 error/tau_est 约：FR 0.193 rad/31 Nm、FL 0.261/42 Nm、RR 0.415/66 Nm、RL 0.291/47 Nm；所有 dq 峰值约 0.001 rad/s。FL 三个电机的 lost 均从首帧到末帧保持 5，没有增长；其余电机保持 0。CRC/invalid=0，没有 SAFE_DAMPING。

高频数据把此前 1 Hz 日志漏掉的插值峰值从约 148 Nm修正为 172.9 Nm。当前主要软件卡点不是 motion action clip，而是 3 秒接管插值持续追踪每 20 ms 变化的 learned-hold target，导致正式行走前 RR/RL calf 已出现大误差和高力矩瞬态；进入稳态后，RR calf 仍承担最大的静态误差/力矩。

日志：

- `logs/hw-gated-hold-50hz-pass1-20260811-controller.csv`
- `logs/hw-gated-hold-50hz-pass1-20260811-controller.log`
- `logs/hw-gated-hold-50hz-pass1-20260811-policy.csv`
- `logs/hw-gated-hold-50hz-pass1-20260811-policy.log`

退出后已恢复 form=0 name=ai；2 秒 999 个有效 LowState，CRC/invalid=0，无残留 controller/policy。

### 29.4 下一步

在任何 gait 之前先修改 handover：预接管 learned-hold shadow 收敛后锁存一个固定目标，插值阶段不再追逐 live policy target；插值完成后再从固定目标平滑切到 live learned-hold。用同一个 50 Hz CSV 做零命令复测，验收目标至少是显著降低 RR calf 的 1.093 rad / 172.9 Nm 插值峰值且不引入结束失力。只有该项通过后才允许 1 秒 vx=0.25 的短测。


## 30. 2026-08-11：修复 MotionSwitcher ReleaseMode 后 1 秒无控制窗口

第 29 节 50 Hz 数据与 policy 首帧对齐后发现：policy 开始时四个 calf 仍约 [-1.444,-1.448,-1.445,-1.449] rad，为正常原生站姿；但 controller 开始发布 LowCmd 的首帧已经掉到 [-1.926,-1.928,-2.092,-2.091] rad。代码根因是 `ReleaseMode()` 成功后固定 sleep 1 秒再 CheckMode，而此时原生 ai 已释放、controller 尚未进入 LowCmd run，机器人在这 1 秒无控制窗口中先失去支撑下蹲。

已修复：ReleaseMode 返回成功后立即进入 handover，不再等待 1 秒；after-handover policy 门禁复用仍 fresh 的 command，若已过期则自然等待下一帧。重新构建，CTest 1/1、Python 142/142 通过。

同条件 A/B 零命令复测确认：

- 修复后 controller 首帧四个 calf 保持 [-1.445,-1.455,-1.453,-1.449] rad，没有在 LowCmd 前塌落；
- 站姿门控 mean=0.121 rad、max=0.339 rad、max dq<0.001 rad/s，稳定 0.5 秒后放行；
- RR calf 插值 error/tau_est max 从 1.093 rad/172.9 Nm 降至 1.004 rad/159.9 Nm；
- RL calf 插值 error/tau_est max 从 0.682 rad/108.1 Nm 升至 0.857 rad/136.2 Nm；
- 因此 1 秒无控制窗口是确定性严重缺陷且已修复，但它不是全部高力矩来源；live learned-hold target 在 3 秒插值中持续变化仍会造成后腿大误差。
- POLICY 稳态 RR/RL calf error MAE 约 0.402/0.424 rad，SAFE_HOLD 静态约 0.400/0.426 rad；CRC/invalid=0，lost_sum 全程固定 15，无新增。

新日志：

- `logs/hw-gated-hold-no-release-gap-50hz-r1-20260811-controller.csv`
- `logs/hw-gated-hold-no-release-gap-50hz-r1-20260811-controller.log`
- `logs/hw-gated-hold-no-release-gap-50hz-r1-20260811-policy.csv`
- `logs/hw-gated-hold-no-release-gap-50hz-r1-20260811-policy.log`

当前下一步：在 release 前的 shadow 阶段锁存一个固定 learned-hold target；3 秒 handover 只插值到这个固定目标，不再追 live target；完成后通过现有 gateway 关节速率限制平滑接回 live learned-hold。必须再次用同条件 50 Hz 零命令 A/B 验证 RR/RL 插值峰值后，才考虑 gait。


## 31. 2026-08-11：固定 handover target 实验失败并已撤回

按第30节计划实现了 release 前锁存单帧 learned-hold target、3秒只插值到固定目标、结束后通过现有速率限制接回 live policy，并完成同条件8秒零命令50 Hz真机A/B。

结果：固定目标降低了 INTERPOLATE mode 内部分峰值，但在约3秒切回 live policy 时，policy 的 raw-action/previous-action闭环已经继续演化，而实际执行仍停留在固定快照附近，形成状态不一致并产生更大的二次冲击。计入插值后1秒的整体 handover 窗口：

- FR calf error/tau_est max 从0.676 rad/102.5 Nm恶化到0.766 rad/122.5 Nm；
- FL calf从0.342/56.0恶化到0.449/66.6；
- RR calf从1.004/159.9恶化到1.478/228.3；
- RL calf从0.857/136.2恶化到1.278/201.0；
- 瞬时 dq 峰值还出现 FR calf 8.82 rad/s、FL calf 6.95 rad/s。

该方案验收失败，已立即从 `gateway/b2_sim_gateway.cpp` 完整撤回，并重新构建通过、CTest 1/1、Python 142/142通过。当前部署版本继续使用 live learned-hold target 插值，但保留站姿门控、ReleaseMode后无1秒空窗、50 Hz CSV和原有两级速率保护。没有运行 gait。

失败实验日志仅用于诊断，不得作为部署候选：

- `logs/hw-gated-hold-fixed-handover-50hz-r1-20260811-controller.csv`
- `logs/hw-gated-hold-fixed-handover-50hz-r1-20260811-controller.log`
- `logs/hw-gated-hold-fixed-handover-50hz-r1-20260811-policy.csv`
- `logs/hw-gated-hold-fixed-handover-50hz-r1-20260811-policy.log`

下一方案不能冻结 policy target。应保持每帧 policy/previous-action闭环同步，同时仅在 handover 阶段基于实测 q 限制 `q_command-q_actual` 或等效PD力矩，并在达到稳定条件后解除；先离线回放和MuJoCo验证，再做零命令真机，不得直接 gait。


## 32. 2026-08-11：交接 PD 力矩门控仿真失败、已撤回

按第31节设想实现过一版仅在 handover 阶段工作的等效 PD 力矩门控：继续使用实时 policy target 和正确的 raw previous_action，将 `Kp*(q_target-q)-Kd*dq` 按 hip/thigh/calf 分别限制为 40/60/80 Nm，并要求门控连续 0.5 秒不再介入后才进入 POLICY。该方案只进入 MuJoCo 验证，没有部署到真机。

仿真结果明确失败：10 秒内门控始终介入，`handover_pd_limited_ticks` 最终达到 4635，controller 一直停在 `HANDOVER_TRACKING`，从未进入 POLICY；B2 无法正常站立。原因是当前 learned-hold 本身需要超过这组门槛的瞬态等效 PD 输出，持续改写 q target 等于持续改变策略执行结果，而不是只抑制接管瞬间。

该实验代码已经从 `gateway/b2_sim_gateway.cpp` 完整撤回，没有保留在硬件路径。撤回后重新构建和 CTest 1/1 通过。失败日志仅用于诊断，不得作为部署候选：

- `logs/sim-handover-pd-guard-r1-20260811-controller.csv`
- `logs/sim-handover-pd-guard-r1-20260811-controller.log`
- `logs/sim-handover-pd-guard-r1-20260811-policy.csv`
- `logs/sim-handover-pd-guard-r1-20260811-policy.log`

随后用真实 deployment bundle 和标准 `--sim` 路径重新运行 10 秒零命令 learned-hold。结果恢复正常：controller 全程进入 POLICY，state CRC/invalid 均为0，policy `SIM_PASS`（502 samples，max inference 0.432 ms）；B2 从初始落地姿态起身，机身 z 在接管后由约0.60 m收敛到约0.51 m，并在有效控制窗口内保持站立。对应日志：

- `logs/sim-reverted-standard-r2-20260811-controller.csv`
- `logs/sim-reverted-standard-r2-20260811-controller.log`
- `logs/sim-reverted-standard-r2-20260811-policy.csv`
- `logs/sim-reverted-standard-r2-20260811-policy.log`
- `logs/sim-reverted-standard-r2-20260811-mujoco.log`

MuJoCo 日志在 controller 结束后曾继续空跑，因此后续出现高度下降；这是测试收尾残留进程在没有 LowCmd 时继续仿真，不是有效控制窗口内倒下。残留进程已清理，后续自动测试应先终止 MuJoCo，避免把控制结束后的自由落体误判为 policy 失败。

当前可部署代码仍只保留：释放 ai 前站姿门控、ReleaseMode 后无1秒失力窗口、实时 learned-hold 插值、原有两级速率保护、Kp=160/Kd=5、CRC/NaN/超时/姿态保护和50 Hz诊断。下一步不要继续增加 target/力矩门控；应先对齐同一初始 LowState 下仿真与真机的 observation、raw action、q target 三条序列，找出为何真机 learned-hold 的 target p2p 明显小于仿真，再决定是传感器/observation差异还是 policy sim-to-real 适配问题。真机 gait 在该对齐完成前暂停。


## 33. 2026-08-11：定位 shadow previous_action 不一致并加入 policy epoch reset

### 33.1 同格式 observation 采集

`cli_sim_policy.py` 和完全只读的 `cli_hw_dryrun.py` 现均可在 CSV 中记录完整45维 observation，以及 raw/applied action、q target、q、dq。真机只读 learned-hold shadow 运行5秒：B2 全程保持 form=0 name=ai，没有 LowCmd publisher；208个50 Hz policy样本，LowState 3499帧，CRC/invalid=0。日志：

- `logs/hw-readonly-observation-hold-r1-20260811.csv`
- `logs/hw-readonly-observation-hold-r1-20260811.log`
- `logs/sim-observation-hold-r1-20260811-policy.csv`

稳态对比表明：真机 ai shadow 时12关节 actual p2p均接近0，raw action p2p仅0.005..0.026；标准MuJoCo闭环中actual p2p约0.005..0.082，raw action p2p约0.046..0.677。真机shadow与仿真 observation 的主要均值差异来自 previous_action（平均绝对差0.535）和q offset（0.132），gyro差0.0007、projected gravity差0.0121，零命令完全一致。

旧的实际接管日志 `hw-gated-hold-no-release-gap-50hz-r1` 与二者不同：2秒后的 raw action p2p达到1.64..6.15，target p2p达到0.206..1.373，明显大于正常仿真，而不是 ONNX 在真机上固定地产生更小输出。真机接管的首个异常时刻为：

- 0.74秒：max abs dq 首次超过0.05 rad/s；
- 0.80秒：max target error超过0.5 rad，同时dq超过1 rad/s；
- 0.94秒：max abs raw action超过3；
- 1..3秒为最大闭环发散窗口，随后落入带0.427 rad静态误差的另一固定点。

### 33.2 根因与修复

硬件流程此前先让 policy shadow 运行约0.5..0.7秒以通过站姿门控，但 `B2PolicyCore` 每帧都会把 raw actor output写入下一帧 observation的previous_action；这些shadow action实际上没有被ai模式下的电机执行。释放ai时，policy因此不是从“上一帧实际执行动作”开始，而是从一串虚构的shadow previous_action开始。标准MuJoCo接管几乎立即开始，previous_action从0起步，所以不存在同样的不一致。

已给内部UDP协议加入 `policy_epoch`（protocol version 2）：gateway在接管前递增epoch并丢弃旧epoch命令；Python policy看到epoch变化后调用`core.reset()`，把previous_action和target slew状态清零，再回传同epoch第一帧目标。gateway确认收到新epoch目标后才调用MotionSwitcher ReleaseMode，因此没有重新引入无LowCmd等待窗口。只读hw probe同步使用version 2、epoch 0。

构建与验证：

- gateway重新构建通过；
- CTest 1/1通过；
- Python 143/143通过；
- 标准MuJoCo端到端日志出现`Requesting policy reset for handover epoch=1`和`Policy reset for handover epoch=1`，随后全程POLICY，CRC/invalid=0；`SIM_PASS samples=253, max inference=0.473 ms`；有效窗口内机身高度约0.60 m收敛至0.52 m，正常站立。

仿真验证日志：

- `logs/sim-policy-epoch-reset-r1-20260811-controller.csv`
- `logs/sim-policy-epoch-reset-r1-20260811-controller.log`
- `logs/sim-policy-epoch-reset-r1-20260811-policy.csv`
- `logs/sim-policy-epoch-reset-r1-20260811-policy.log`
- `logs/sim-policy-epoch-reset-r1-20260811-mujoco.log`

### 33.3 当前状态和下一步

新epoch-reset版本尚未发布active LowCmd到真机。当前B2只读确认仍为form=0 name=ai，无残留controller/policy/MuJoCo进程。下一步只允许做一轮短零命令 learned-hold A/B，不做gait；用50 Hz CSV确认0.74..3秒窗口的dq/raw action/target error是否显著下降。验收通过后再做1秒vx=0.25；若reset后仍发散，再用本次新增的45维observation CSV定位IMU或实际关节反馈差异，不再尝试固定target或PD门控。


## 34. 2026-08-12：epoch reset 真机 A/B、首帧支撑力矩断层与当前站姿卡点

### 34.1 epoch reset 零命令真机 A/B 已完成，但未解决交接瞬态

在正常原生站姿下完成一轮约6秒真机 learned-hold，command全程为`[0,0,0]`，没有gait。站姿门控通过：mean q error=0.121 rad、max=0.337 rad、max dq<0.001 rad/s；日志确认policy epoch从0切到1、Python执行reset、收到同epoch新目标后才ReleaseMode。controller正常经历INTERPOLATE、POLICY、SAFE_HOLD，退出后恢复form=0 name=ai；CRC/invalid=0，没有Safety trip或SAFE_DAMPING。

但50 Hz全量A/B推翻了“shadow previous_action是主要瞬态根因”的假设。reset前后首个异常时刻基本完全相同：

- max abs dq>1 rad/s：旧版0.800秒，reset版0.802秒；
- max target error>0.5 rad：旧版0.800秒，reset版0.802秒；
- max abs raw action>3：旧版0.940秒，reset版0.942秒；
- reset版全程max abs raw action=7.769、max dq=4.250 rad/s、max target error=1.863 rad，没有改善瞬态。

逐小腿峰值（error rad / abs tau_est Nm / abs dq rad/s）：

| Joint | 旧 INTERPOLATE | epoch-reset INTERPOLATE |
|---|---:|---:|
| FR calf | 0.676 / 102.5 / 2.41 | 0.873 / 134.6 / 3.43 |
| FL calf | 0.342 / 56.0 / 1.79 | 0.624 / 99.9 / 3.41 |
| RR calf | 1.004 / 159.9 / 1.45 | 1.034 / 166.7 / 1.47 |
| RL calf | 0.857 / 136.2 / 1.46 | 0.873 / 138.2 / 1.42 |

epoch reset只改善了进入POLICY后的部分后腿静态误差：RR/RL calf max error从0.448/0.427降到0.327/0.322 rad，但代价是FR/FL插值瞬态更差，因此整体不通过，不能据此开始gait。

本轮温度最高58摄氏度（RR calf）；motor lost首尾均为同一固定历史值：FL hip/thigh/calf各5、RR calf 5，总和20，本轮增量为0。日志：

- `logs/hw-policy-epoch-reset-hold-r1-20260811-controller.csv`
- `logs/hw-policy-epoch-reset-hold-r1-20260811-controller.log`
- `logs/hw-policy-epoch-reset-hold-r1-20260811-policy.csv`
- `logs/hw-policy-epoch-reset-hold-r1-20260811-policy.log`

### 34.2 新定位：ReleaseMode 前后存在首帧支撑力矩断层

epoch reset日志的controller首帧显示，释放ai前机器人靠实测非零关节力矩承重，但原handover第一帧使用`q_target=q_actual, dq_target=0, tau=0`，因此新PD命令`Kp*(q_target-q)-Kd*dq`约等于0。典型小腿：

| Joint | ai末态 tau_est | 原首帧新PD | 保持同力矩所需q target |
|---|---:|---:|---:|
| FR calf | +40.5 Nm | ~0 Nm | -1.193 rad（q=-1.446） |
| FL calf | +39.2 Nm | ~0 Nm | -1.205 rad（q=-1.450） |
| RR calf | +66.1 Nm | ~0 Nm | -1.045 rad（q=-1.458） |
| RL calf | +61.1 Nm | ~0 Nm | -1.067 rad（q=-1.449） |

也就是说，虽然已消除ReleaseMode返回后的固定1秒软件空窗，第一帧LowCmd仍会把约39..66 Nm的小腿重力支撑瞬间降到接近0；随后关节下沉、PD误差建立、policy observation变化，形成0.8..3秒闭环瞬态。这比previous_action reset更直接解释现有数据。

已实现无扰力矩接管：仅在3秒handover开始时，用
`q_start = q + (tau_est + Kd*dq)/Kp`
生成首帧虚拟位置目标，从而使新PD输出匹配切换前实测tau_est；q offset限制为绝对值不超过0.5 rad并保留硬关节限位，之后仍按原3秒live-policy插值。该机制只匹配首帧，不像失败的PD guard那样持续篡改policy target。LowState有效性检查同时增加tau_est有限值检查。

构建、CTest 1/1、Python 143/143通过。`--sim-hardware-handover` 8秒MuJoCo通过：日志出现torque-matched首目标、POLICY、CRC/invalid=0，`SIM_PASS samples=403`，模型从初始落地姿态站起后机身高度稳定约0.52 m。仿真日志：

- `logs/sim-torque-matched-handover-r1-20260812-controller.csv`
- `logs/sim-torque-matched-handover-r1-20260812-controller.log`
- `logs/sim-torque-matched-handover-r1-20260812-policy.csv`
- `logs/sim-torque-matched-handover-r1-20260812-policy.log`
- `logs/sim-torque-matched-handover-r1-20260812-mujoco.log`

### 34.3 torque-matched 真机验证被站姿门控正确拦截

准备运行真机 torque-matched 零命令A/B时，站姿门控测得mean q error=0.584 rad、max=1.281 rad，连续10秒不合格后在ReleaseMode之前退出。没有发布active LowCmd，没有释放ai，controller退出码1。这是保护机制的正确行为。

当前12关节只读值约为：

`[-0.3060, 1.2057, -2.7810, 0.2742, 1.2347, -2.7554, -0.2756, 1.2101, -2.7799, 0.2509, 1.1605, -2.7815]`

明显是深蹲而非policy-ready原生站姿。宇树官方SportClient的`BalanceStand()`和`RecoveryStand()`均完整返回code 0，但姿态没有执行变化；之后仍为form=0 name=ai、q0约-0.306 rad。2秒只读shadow的max abs action=3.961、target delta max=1.603 rad，说明绝不能在此姿态绕过门控。

失败/只读日志：

- `logs/hw-torque-matched-hold-r1-20260812-controller.log`（门控失败，无LowCmd）
- `logs/hw-torque-matched-hold-r1-20260812-policy.csv`
- `logs/hw-torque-matched-hold-r1-20260812-policy.log`
- `logs/hw-post-recovery-readonly-20260812.log`

### 34.4 当前状态、卡点与下一步

当前真机状态：form=0 name=ai；LowState约500 Hz；CRC/invalid=0；机器人仍处于深蹲；没有残留controller、policy、MuJoCo或SportClient进程。torque-matched代码已构建并通过仿真，但从未在真机发布active LowCmd。

当前唯一阻塞真机A/B的条件是：官方`BalanceStand()`/`RecoveryStand()`返回成功却没有把机器人从深蹲恢复到正常站姿。不要放宽或删除姿态门控，也不要从当前姿态继续接管。

下一步按顺序：

1. 排查宇树高层状态为何“RPC code 0但动作不执行”：检查遥控器/急停/保护状态、足端是否承重受阻、SportClient状态反馈，并尝试官方推荐的`StopMove -> RecoveryStand/BalanceStand`状态序列；每一步后只读检查12关节，不以RPC code 0单独判定成功。
2. 只有恢复到门控阈值（mean<=0.25、max<=0.55、max dq<=0.20并稳定0.5秒）后，重新运行一轮6秒torque-matched零命令A/B；不运行gait。
3. A/B验收重点：首帧命令等效PD与tau_est的差值、0.8..3秒max dq、target error、四个calf的error/tau峰值必须显著低于第34.1节epoch-reset基线，同时lost不能增长、温度不能继续异常上升。
4. 只有零命令A/B通过后，才允许1秒`vx=0.25`；否则撤回torque-matched实验并继续检查tau_est符号/时序或高层到低层切换语义。

负责人一句话结论：**epoch reset已证明不是主要修复；新数据定位到ai约39..66 Nm小腿支撑力矩在首帧LowCmd被降到近0，已实现并通过MuJoCo的torque-matched无扰切换，但真机当前深蹲且官方站立/恢复RPC虽返回成功却不动作，姿态门控已正确阻止接管，因此下一步必须先恢复真实正常站姿，不能继续真机行走。**


## 35. 2026-08-12：当前程序已切回真机实测表现最佳版本

按用户要求复核全部现场记录后，真机表现最好的可复现版本确定为第27节第2轮，而不是后续实验版。该轮使用：

- 硬件Kp=160、Kd=5；
- policy-side q target slew=8 rad/s；
- gateway-side B2关节速率限制开启；
- action clip关闭；
- 从实测q到每帧实时learned-hold target做3秒插值；
- observation previous_action使用上一帧raw actor output。

现场表现：第27节第2轮左后腿有明确前进动作，右后腿略微抬起；第3轮同类但更弱。没有任何版本取得真实位移，因此这里的“最好”仅表示截至目前腿部步态动作最明显、重复性最好。240/6没有更好的现场位移证据；关闭两级限制并clip到±3的六轮经录像确认身体下沉偏摆、表现更差。

当前代码已撤回并完全删除两项后续未获真机收益的实验：

- policy epoch reset：真机A/B中0.8..3秒瞬态没有改善，前腿峰值反而升高；
- torque-matched首帧：只通过MuJoCo，尚未在真机发布active LowCmd，不作为当前部署版本。

内部UDP协议也已从实验version 2恢复为第27节控制链使用的version 1（StatePacket 148 bytes、CommandPacket 76 bytes）。重新构建通过，CTest 1/1、Python 143/143通过；`--sim-hardware-handover` 8秒复测`SIM_PASS samples=402`，3秒插值后机身高度稳定约0.513 m，CRC/invalid=0。日志：

- `logs/sim-best-hw-r2-profile-20260812-controller.csv`
- `logs/sim-best-hw-r2-profile-20260812-controller.log`
- `logs/sim-best-hw-r2-profile-20260812-policy.csv`
- `logs/sim-best-hw-r2-profile-20260812-policy.log`
- `logs/sim-best-hw-r2-profile-20260812-mujoco.log`

保留的后续安全改进不改变第27节policy控制律：ReleaseMode后无固定1秒失力等待、接管前真实12关节站姿门控、正常结束SAFE_HOLD、故障SAFE_DAMPING、CRC/NaN/LowState超时/倾倒保护和50 Hz诊断CSV。

当前真机仍为ai深蹲，姿态不满足门控，因此本次只完成程序切换和仿真验证，没有再次发布真机LowCmd。恢复正常站姿前不得绕过门控。当前部署候选明确为：**160/5 + 两级速率限制 + 无action clip + live-policy 3秒插值 + raw previous_action**。


## 36. 2026-08-12：所有后续仿真强制保存视频

用户明确要求：从本节开始，每一次MuJoCo仿真都必须生成并保留视频；缺少视频的仿真不算完成，不能只交付console log、CSV或统计图。

公共入口`scripts/run_b2_mujoco.sh`已改为默认强制X11录屏：

- 使用项目Python环境自带的imageio-ffmpeg二进制；
- 采集当前DISPLAY（默认`:1`）完整仿真画面；
- 输出H.264 MP4，1280x720，20 fps，yuv420p；
- 默认保存到`logs/videos/sim-YYYYMMDD-HHMMSS.mp4`；
- 每轮应优先设置`B2_SIM_VIDEO`，使视频与controller/policy/MuJoCo日志使用同一实验前缀；
- ffmpeg、DISPLAY或屏幕尺寸不可用时，启动脚本直接失败，不允许静默运行无视频仿真；
- 收到INT/TERM时先停止MuJoCo、再用SIGINT结束ffmpeg以写完MP4索引，并输出最终绝对路径。

相对`B2_SIM_VIDEO`路径会在启动时解析为仓库下绝对路径，避免MuJoCo切换工作目录后错误校验。已完成两轮录屏冒烟测试；第二轮文件可被ffmpeg识别为6秒级有效H.264视频，最终复测文件：

- `logs/videos/sim-recording-smoke-r2-20260812.mp4`
- `logs/sim-recording-smoke-r2-20260812.log`

后续交付每轮仿真结果时，必须同时报告视频的绝对路径；批量仿真每一轮必须使用不同文件名，禁止覆盖上一轮证据。


## 37. 2026-08-12：12 cm 上楼梯仿真基线通过

按用户要求运行楼梯场景，未修改策略或仿真参数。使用成功复现包的
`oblique_stairs_up_12 / heading_000 / center` 场景：5级台阶，每级高
0.12 m、踏面0.35 m，策略周期50 Hz、物理周期200 Hz、前进命令0.50 m/s。

本轮结果为 `provisional_pass=true`、`termination_reason=course_complete`，
没有跌倒；8.13 s内前进2.9455 m，最终航向误差0.0155 rad，最大横向偏差
0.0943 m，最低机身高度0.5215 m，最低离地间隙0.4306 m，力矩饱和比例0。
录像已抽帧确认楼梯和B2画面正常，并用VLC打开播放。

证据目录：

- `logs/stairs/stairs-up-12cm-20260812-r1/episode.mp4`
- `logs/stairs/stairs-up-12cm-20260812-r1/summary.json`
- `logs/stairs/stairs-up-12cm-20260812-r1/trajectory.csv`
- `logs/stairs/stairs-up-12cm-20260812-r1/scene/scene.xml`

边界：这是Python MuJoCo策略直连的楼梯能力基线，不是Unitree DDS
`LowState -> policy -> LowCmd`控制器在楼梯场景上的闭环验证；当前官方
unitree_mujoco DDS入口仍使用平地`scene.xml`。若下一步要求验证完整部署链，需将
相同楼梯几何接入B2 DDS场景后，再用当前160/5候选控制器重复测试并单独录像。


## 38. 2026-08-12：平地DDS全链路5秒行走仿真与录像修复

按用户要求使用当前标准平地链路运行：官方B2 MuJoCo DDS `LowState -> 50 Hz
ONNX policy -> 500 Hz LowCmd`，Kp=160/Kd=5、policy target slew=8 rad/s、
gateway关节速率限制开启、无action clip、raw previous_action。流程为2秒
learned-hold、5秒`vx=0.25 m/s`、2秒learned-hold。

前两次编排/录像不作为通过结果：r1错误使用3秒模拟硬件接管且MuJoCo在接管前空跑；
r2虽然有效窗口产生步态，但停止顺序使录像包含长时间无控制尾段。它们的日志和录像保留
为故障证据，未覆盖或冒充最终结果。

最终有效轮次为r3：policy共452帧（warmup 101、motion 250、settle 101），运动段
4.98秒，平均command vx=0.2496 m/s，最大推理0.707 ms，所有policy数值有限；
MuJoCo base x约从sim time 2.002秒的0.4676 m移动到7.004秒的1.2122 m，
对应约0.7446 m前进。运动结束2秒hold后z=0.5377 m、最后最大关节速度约
0.0242 rad/s，姿态保持稳定。CRC/invalid/lost均未增长。controller在模拟器停止前
进入SAFE_HOLD；CSV末尾10个SAFE_DAMPING样本由测试程序主动终止MuJoCo后
LowState中断触发，不属于有效控制窗口。

用户反馈先前VLC打开录像黑屏。抽帧证明H.264文件内有画面，实际是VLC/NVIDIA
VA-API硬件视频路径初始化失败。已将`~/.config/vlc/vlcrc`设置为
`avcodec-hw=none`和`vout=xcb_x11`，最终r3录像逐帧解码通过，并已用VLC软件路径
播放验证。最终证据：

- `logs/videos/sim-flat-walk-5s-20260812-r3.mp4`
- `logs/sim-flat-walk-5s-20260812-r3-policy.csv`
- `logs/sim-flat-walk-5s-20260812-r3-policy.log`
- `logs/sim-flat-walk-5s-20260812-r3-controller.csv`
- `logs/sim-flat-walk-5s-20260812-r3-controller.log`
- `logs/sim-flat-walk-5s-20260812-r3-mujoco.log`


## 39. 2026-08-12：新设计15 cm斜向上楼梯场景通过

新增配置`configs/scenarios-stairs-15cm-oblique.yaml`。新场景名为
`stairs_up_15cm_oblique_15deg`，设计目的处于原12 cm和18 cm基线之间，同时增加
斜向穿越约束：6级台阶、每级高0.15 m、踏面0.32 m，总抬升0.90 m；接近段
1.25 m、顶层平台1.40 m、通道宽3.20 m、摩擦系数0.80；机器人初始和目标航向
均为相对台阶法线15度，nominal speed=0.50 m/s。原policy、Kp/Kd、action scale、
物理周期0.005 s和policy周期0.02 s均未修改。

首轮仿真通过：`provisional_pass=true`、`course_complete`、未跌倒；8.185 s内
前进3.2376 m，最终航向误差0.0172 rad，最大横向偏差0.0961 m；最低机身高度
0.5206 m、最低离地间隙0.4339 m、最大roll 0.0665 rad、最大pitch 0.2932 rad；
峰值力矩184.77 Nm，力矩饱和比例0，最小关节限位余量0.4370 rad。中段和终点
录像抽帧均确认B2已完成攀爬并在顶层平台直立，录像逐帧解码通过且已用VLC打开。

证据：

- `configs/scenarios-stairs-15cm-oblique.yaml`
- `logs/stairs/stairs-up-15cm-oblique15-20260812-r1/episode.mp4`
- `logs/stairs/stairs-up-15cm-oblique15-20260812-r1/summary.json`
- `logs/stairs/stairs-up-15cm-oblique15-20260812-r1/trajectory.csv`
- `logs/stairs/stairs-up-15cm-oblique15-20260812-r1/scene/scene.xml`

边界：本轮仍是Python MuJoCo policy直连的场景能力测试，不是Unitree DDS
`LowState -> policy -> LowCmd`完整部署链测试。


## 40. 2026-08-12：15 cm楼梯接入官方DDS MuJoCo并修复hardware handover

新增官方B2场景
`third_party/unitree_mujoco/unitree_robots/b2/scene_stairs_15cm_oblique.xml`，并让
`scripts/run_b2_mujoco.sh`支持通过`B2_SIM_SCENE`选择场景，默认平地入口不变。
已使用官方Unitree MuJoCo DDS完整链运行：`rt/lowstate -> 50 Hz ONNX policy ->
500 Hz rt/lowcmd`，controller参数为`--sim-hardware-handover`、Kp=160、Kd=5、
3秒初始姿态插值、两级target rate limit开启、action clip关闭。

首轮失败发生在到达楼梯前：controller从state=6的自由落体LowState开始handover，
首帧calf速度约-8.5 rad/s，3秒插值期间后腿误差达到约0.82 rad、tau_est约131 Nm，
随后姿态保护触发SAFE_DAMPING。根因不是楼梯碰撞或非零command，而是模拟器预保持
错误地把第一帧动态LowState当作稳定交接姿态。

已修改`gateway/b2_sim_gateway.cpp`：仅对带初始插值的模拟硬件handover，先等待
12关节`max_abs_dq<=0.20 rad/s`连续0.5秒；静止门控通过前不向policy转发状态，
通过后才锁存pre-hold姿态并开始policy与3秒handover。普通`--sim`和真机路径未改。
重新构建通过，CTest 1/1、Python 143/143通过。

0.50 m/s复测成功爬到顶层，但12秒持续前进走出1.40 m顶层平台；将运动窗口按课程
终点缩短为7秒后，机器人在顶层`x=3.57 m, z=1.42 m`保持稳定，运行阶段无Safety
trip。随后按真机基线将速度改为0.25 m/s，并把自定义场景默认速度永久改为0.25、
时长改为14秒。最终0.25 m/s DDS复测：运动首帧command约
`[0.2358,0,0.1330]`（同时修正15度航向），成功完成6级总高0.90 m楼梯；运动结束
后顶层learned-hold约`x=3.32 m, z=1.42 m`，无运行期Safety trip、CRC或invalid。
CSV尾部SAFE_DAMPING仅由测试主动停止MuJoCo造成的LowState中断触发。

最终0.25 m/s证据：

- `logs/videos/sim-dds-stairs-15cm-oblique-v025-handover-20260812-r4.mp4`
- `logs/sim-dds-stairs-15cm-oblique-v025-handover-20260812-r4-controller.csv`
- `logs/sim-dds-stairs-15cm-oblique-v025-handover-20260812-r4-controller.log`
- `logs/sim-dds-stairs-15cm-oblique-v025-handover-20260812-r4-policy.csv`
- `logs/sim-dds-stairs-15cm-oblique-v025-handover-20260812-r4-policy.log`
- `logs/sim-dds-stairs-15cm-oblique-v025-handover-20260812-r4-mujoco.log`


## 41. 2026-08-12：B2_sim2real 键盘遥控接入现有真机控制链

按用户要求审计并接入 `StevenLiudw/B2_sim2real`。键盘实现固定参考
`agent/document-b2-real-handoff` 分支提交
`2fe3e96cfdd10b6fa03402deda6927fce8d720b9`。没有复制该仓库完整的独立
hardware runner，也没有替换当前已经验证的 DDS gateway；只复用了其
`InteractiveVelocityCommand` 和非阻塞 TTY 输入语义。

当前完整链路为：终端键盘生成 `[vx,vy,wz]` → 写入45维 observation 的第6:9
元素 → 50 Hz ONNX policy → 12关节 q target → 本地UDP → 现有500 Hz C++
gateway → `rt/lowcmd`。因此 LowCmd 初始化、CRC、MotionSwitcher、3秒实时policy
插值、Kp=160/Kd=5、两级关节速率保护、无action clip、raw previous_action、
LowState/NaN/姿态超时和 SAFE_HOLD/SAFE_DAMPING 均保持原实现不变。

新增/修改内容：

- `src/b2_control/interactive_command.py`：W/S、A/D、Q/E、数字档位和零命令；
- `B2PolicyCore.step(..., velocity_command=...)`：验证有限3维向量并直接送入
  observation；`hold=True` 永远覆盖成 `[0,0,0]`；
- `cli_sim_policy.py --interactive-keyboard`：TTY实时按键、启动零命令warmup、
  速度步长/上限和CSV诊断；
- `b2-policy-teleop` console entry point，已安装到现有Python环境，并在
  `/home/zhangchi/.local/bin/b2-policy-teleop` 提供可直接调用入口；
- `docs/keyboard-teleop.md`：仿真和真机三终端使用说明。

安全按键：Space/Z/R/X/Esc立即清零进入learned hold，Ctrl-C退出后gateway保留
最后安全目标。方向键转义序列结尾是大写A/B/C/D，因此实现明确忽略所有大写运动
字母，避免按方向键误触发WASD。

验证结果：Python回归161/161通过，C++ CTest 1/1通过，`git diff --check`
通过。第一次DDS MuJoCo轮次r1因8秒窗口结束后才送到按键，只产生零命令，保留为
失败证据且不计通过。第二轮r2使用官方Unitree DDS MuJoCo、
`--sim-hardware-handover`、3秒handover并强制录像；policy共1002帧：warmup 151、
motion 750、settle 101。CSV确认键盘命令实际进入policy：

- `[0,0,0]`：446帧；
- `[0.05,0,0]`：188帧；
- `[0.10,0,0]`：201帧；
- `[0.10,0,0.10]`：167帧。

全部policy字段有限；controller依次进入INTERPOLATE、POLICY和SAFE_HOLD，正常
测试窗口没有触发姿态/数值故障。尾部SAFE_DAMPING发生在policy已退出、LowState
停止更新的测试拆除阶段，不属于键盘控制窗口。录像为H.264 1280×720、20 fps、
36.05秒，逐帧解码通过且抽帧确认不是黑屏，画面中B2保持直立并产生步态动作。

有效证据：

- `logs/videos/sim-keyboard-teleop-20260812-r2.mp4`
- `logs/sim-keyboard-teleop-20260812-r2-policy.csv`
- `logs/sim-keyboard-teleop-20260812-r2-controller.csv`

本节没有启动domain 0真机LowCmd。真机入口已接通，但首次使用仍必须满足当前站姿
门控并在吊架下从单次W（`vx=0.05 m/s`）开始；A/D横移、负vx和较大wz必须先逐项
完成MuJoCo验证，不能因为键盘接口已可用就直接在真机尝试。


## 42. 2026-08-12：键盘控制再次仿真与录像退出修复

按用户要求再次运行官方Unitree DDS MuJoCo平地仿真，参数仍为
`--sim-hardware-handover`、Kp=160/Kd=5、3秒handover、两级速率限制开启、
action clip关闭。键盘顺序为`[0,0,0] -> [0.05,0,0] -> [0.10,0,0] ->
[0.10,0,0.10] -> X清零`。

r3控制数据有效，但原MP4在Ctrl-C时没有写出`moov`索引，判定录像失败；r4启用
fragmented MP4后可以播放，但先停policy使gateway冻结最后一帧静态target，长时间
SAFE_HOLD后机器人倾倒，因此r4也不作为最终结果。由此确认：模拟结束时不能先退出
learned-hold policy并长时间保留静态target。

最终有效轮次r5在X清零后继续保持实时learned-hold，确认稳定后先结束MuJoCo/录像，
再停止policy和gateway。policy共1859帧，所有数值有限；命令帧数分别为零命令1192、
0.05 m/s 159、0.10 m/s 157、0.10 m/s+wz=0.10 rad/s 351。最后2秒零命令窗口
raw action与q target均已稳定（各关节最大p2p分别约8.3e-7和2.1e-7），仿真结束前
机身约`z=0.517 m`并保持直立。controller有效窗口仅经历INTERPOLATE和POLICY；
尾部SAFE_DAMPING是先停止MuJoCo后LowState中断的预期拆除行为，没有对应的仿真
跌倒窗口。

`scripts/run_b2_mujoco.sh`已改为先录制可中断的fragmented MP4，再自动丢弃损坏尾包、
转码并写入完整索引，避免以后Ctrl-C产生无法打开的录像。r5最终录像为H.264、
1280x720、20 fps、37.2秒，完整逐帧解码通过，末帧抽查机器人直立且不是黑屏。

最终证据：

- `logs/videos/sim-keyboard-teleop-20260812-r5-final.mp4`
- `logs/sim-keyboard-teleop-20260812-r5-policy.csv`
- `logs/sim-keyboard-teleop-20260812-r5-controller.csv`


## 43. 2026-08-12：手持遥控器接入自研 ONNX policy 与锁存急停

已将宇树手持遥控器接入当前 `LowState -> 45维 observation -> ONNX -> 12关节
q_target -> 500 Hz LowCmd` 链路。未修改宇树原键位和方向：`ly -> vx`、`-lx ->
vy`、`-rx -> wz`，A进入policy，R2回learned hold，L2+B作为锁存软件急停；其它
键未重新分配。默认死区0.10，速度上限仍为vx 0.90 m/s、vy 0.25 m/s、wz 0.40
rad/s。A仅在摇杆居中时生效，遥控启动强制至少5秒learned-hold，覆盖现有3秒真机
handover插值。

真机只读检查确认：enp8s0为192.168.123.99/24，domain 0 `rt/lowstate`约500 Hz、
CRC/invalid为0；该B2 EDU固件没有独立发布`rt/wirelesscontroller`，但
`LowState.wireless_remote[40]`有效，头为`0x55 0x51`，居中时keys=0且四轴为0。
因此gateway按宇树官方低层example直接解析LowState内嵌遥控数据，同时保留
`rt/wirelesscontroller`作为MuJoCo/兼容固件的备用输入。

内部UDP StatePacket升级为version 2/172 bytes，新增lx/ly/rx/ry、keys、remote
fresh和estop状态；CommandPacket仍为76 bytes，但flags新增remote-required与
emergency-stop。遥控输入超过250 ms不新鲜时，gateway锁存SAFE_DAMPING。L2+B
在C++ LowState回调直接检测，不依赖50 Hz Python是否活着；触发后12关节强制
Kp=0、Kd=3、tau=0，松键不会恢复，进程重启前禁止自动MotionSwitcher切回ai。
急停后必须先物理固定/支撑机器人，并保持gateway输出阻尼；不能先杀gateway使
LowCmd彻底中断。

验证：Python 176/176、C++构建和CTest 1/1通过。官方Unitree DDS MuJoCo完整脚本
通过：居中等待 -> A -> 约0.25 m/s前进 -> R2 learned hold -> L2+B；policy日志出现
POLICY/STAND/EMERGENCY_STOP，gateway日志明确进入SAFE_DAMPING、Kd=3、CRC和
invalid均为0。视频H.264逐帧解码通过且抽帧非黑屏：

- `logs/videos/remote-onnx-estop-sim-20260812.mp4`
- `logs/remote-onnx-estop-policy.log`
- `logs/remote-onnx-estop-gateway.log`
- `logs/remote-onnx-estop-script.log`
- `docs/hand-remote-onnx-policy.md`

本轮没有启动domain 0真机LowCmd，机器人仍由官方ai保持站立。下一步首次真机遥控
只能在吊架/防跌落条件下执行：启动gateway和`b2-policy-teleop --remote-control`，
5秒hold完成后摇杆居中短按A，从极小前推开始；R2正常回hold，L2+B只用于急停。


## 44. 2026-08-12：手持遥控器首次真机 ONNX 接管测试

首次尝试因机器人仍为蹲伏姿态被站姿门控拒绝，未释放ai：mean_q_error=0.624、
max_q_error=1.306。原生R2站立后第二次门控通过：mean=0.105、max=0.287、
max_dq约0.001。随后MotionSwitcher成功释放ai，3秒实时learned-hold插值完成，
gateway进入POLICY；LowState和遥控数据持续新鲜，CRC/invalid/lost均为0。

数据链验证成功：遥控A键值256被收到，摇杆命令实际写入45维observation并驱动
ONNX/LowCmd。但操作过程中四个摇杆轴均出现过接近满量程，policy实际命令范围为
vx [-0.893,0.900] m/s、vy [-0.250,0.250] m/s、wz [-0.400,0.394] rad/s，明显
超过计划的10%–20%轻推测试。整轮只收到A，没有收到R2键值16，也没有L2+B急停；
因此不能把本轮判为“低速遥控验证通过”，只能判定通信和policy链路通过。

高速输入窗口统计到全关节峰值：q_target跟踪误差0.974 rad、|dq| 7.417 rad/s、
|tau_est| 150.844 Nm、温度50摄氏度。现有显式保护只覆盖姿态、通信超时、CRC/NaN
和硬角度/速率限幅，跟踪误差与tau_est没有独立触发阈值，所以这些峰值没有触发
SAFE_DAMPING。这是下一轮前必须收紧测试速度或增加显式监视的重要证据。

由于现场没有产生R2，测试由软件主动结束：停止remote policy后立即启动强制
`--hold` learned-hold，稳定后停止gateway并确认MotionSwitcher恢复`form=0 name=ai`，
最后停止hold policy。退出后只读LowState约500 Hz、CRC/invalid=0、关节速度近零，
自研控制进程为0。

证据：

- `logs/remote-hardware-20260812-r2-gateway.csv`
- `logs/remote-hardware-20260812-r2-policy.csv`
- `logs/remote-hardware-20260812-r2-gateway.log`
- `logs/remote-hardware-20260812-r2-exit-hold-policy.csv`

下一次真机遥控不应继续使用0.90/0.25/0.40满量程上限。先用CLI把vx/vy/wz分别
限制到0.25/0.10/0.20，并只读确认实体R2确实输出keys=16后，再做低速移动轮次。


## 45. 2026-08-13：独立 B2 真机/仿真可视化控制中心

新增可脱离Codex运行的`B2 Sim2Real Control Center`。桌面和应用菜单均已有启动
图标，启动脚本为`scripts/start_b2_control_center.sh`，Web服务只监听127.0.0.1并
使用随机token；程序启动保持IDLE，不自动连接真机、不发布LowCmd、不释放ai。

控制中心已统一接入：verified policy注册与每次启动SHA-256校验、官方Unitree DDS
MuJoCo、LowState→45维observation→ONNX 50 Hz→LowCmd 500 Hz链路、W/S/A/D按住
移动和松开停止、数字键1–9选择W/S的0.1–0.9 m/s档位、Q/E每次±0.10 rad/s、
原生/自研locomotion双确认切换、网页键盘/
宇树手持遥控器单命令源选择、A/R2/L2+B原遥控键位、LowState/CRC/MotionSwitcher
真机只读预检、分进程日志、全量CSV、强制仿真录像和自动诊断图表。真机“一键
连接”只做只读检查；连接成功后仍为官方ai，必须在嵌入操作台再次确认自研模式才
会发生接管。

新增9个可加载场景：平地、草地近似、泥路近似、确定性石子路、10/15/20 cm楼梯、
12度斜坡和综合路线。所有场景复用官方B2模型、0.002秒步长；草地/泥路通过MuJoCo
friction/solref/solimp近似，不宣称为现场土壤标定。policy注册表把bundle和adapter
分离；当前只允许已验证的`b2_45x12_v1`，未来不同observation/action合同应新增
adapter，接口不匹配时fail-closed。

最终草地验收实际发出0.6 m/s前进命令4秒，松开后回learned hold；链路、录像封装
和统计生成全部成功。证据：

- `logs/videos/sim-20260813-181329-grass.mp4`
- `logs/control-center/sim-20260813-181329/policy.csv`
- `logs/control-center/sim-20260813-181329/gateway.csv`
- `logs/control-center/sim-20260813-181329/policy-tracking-tracking.svg`
- `logs/control-center/sim-20260813-181329/policy-tracking-raw-action.svg`
- `logs/control-center/sim-20260813-181329/policy-tracking-stats.csv`

验收结果：Python全量`204 passed`；C++ gateway、probe、remote script和data test全部
构建成功；9个地形均已用MuJoCo Python实际加载，nq=19、dt=0.002；桌面和移动宽度
界面无横向溢出。完整说明见`docs/control-center.md`。

本轮没有连接或操作真机；验收时`enp8s0`为DOWN。后续真机使用时先接网线、现场
吊架/防跌落和物理急停就绪，再由操作者在控制中心明确选择网卡、命令源并完成两次
确认。当前不需要Codex常驻。


## 46. 2026-08-14：控制台真机安全闭环、全量遥测与故障恢复

按“用户权限与审计除外，其它全部加入”的要求扩展了独立控制中心。本轮只修改代码、
编译、测试和运行 Unitree DDS MuJoCo，没有连接、接管或操作真机。

### 本轮完成

- 真机连接改为强制预检：操作者在场、区域清空、机型/固件确认、物理急停可用、
  命令源为零；连接阶段仍只读 LowState/CRC/MotionSwitcher，不发布 LowCmd。
- 建立唯一规范配置 `configs/b2-reference-profile.json`。真机自研接管要求配置状态
  `approved`、`activation_permitted=true`、12 关节参数完整，并绑定 policy SHA-256、
  机身序列号、固件、复核人和复核时间。当前随附配置是 `reference_only`，因此默认
  fail-closed；旧 `config/b2-reference-profile.json` 不再作为运行时真机参数源。
- C++ gateway 在释放官方 `ai` 前增加本机独占锁，并按官方 B2 部署入口先订阅
  `rt/lowcmd` 检测已有发布者；检测到并行低层控制器时拒绝接管。
- 真机控制参数完全来自已审批配置：逐关节 Kp/Kd、增益爬坡、目标速率限制、软边界、
  硬边界和超时。仿真的既有 Kp=160/Kd=5、200 Hz 力矩计算和模型限制没有被修改。
- 安全监控覆盖 LowState CRC/NaN/超时、policy/命令超时、发布心跳、关节硬角度、dq、
  tau_est、持续跟踪误差、电机温度、lost 计数、机身倾角/角速度、BMS SOC/原始电流/
  温度。任一保护触发后锁存 SAFE_DAMPING，强制 Kp/tau=0、Kd=审批阻尼，并禁止自动
  归还原生 locomotion。
- 增加 500 Hz 进程内发布 watchdog、连续 gateway 黑匣子 CSV，以及包含首个故障原因、
  配置/策略摘要和恢复说明的 `.fault.json` sidecar。
- UDP 状态协议升级到 version 3/326 bytes，补齐 12 关节 tau_est、温度、lost、BMS 和
  安全锁存字段；Python/C++ 两端已对齐。
- 操作台新增 12 关节 q_target/q_actual/error/dq/tau_est/温度/lost 表格、电池和安全
  状态；键盘/遥控器输入源切换默认清零，并只允许在已验证的原生模式下执行。
- 增加原生 B2 站立、起身、趴下、恢复站立、阻尼、低功耗操作入口。按钮仅在
  HARDWARE_CONNECTED 可用，危险动作额外确认，自研 LowCmd 活跃时拒绝执行。
- 增加锁存故障恢复：必须确认机器人已物理支撑、急停可用、故障证据已检查；只向
  核对过可执行路径/接口的记录 PID 发送 SIGTERM，绝不 SIGKILL，gateway 正常退出后
  才显式请求恢复官方 `ai`。失败则继续保持 FAULT。
- 增加平地 0.25/0.60、草地、石子路、10/15 cm 楼梯、斜坡等测试模板；模板只预填
  场景和速度，不会自动启动或运动。已有九类场景、每轮强制录像和跟踪图表继续保留。
- 未增加用户帐号、角色权限或操作人员审计；现有事件日志仅是控制系统故障诊断证据。

### 仿真与验证

第一次 version 3 联调在 `sim-20260814-160529` 触发了假急停。原因不是机器人或
policy，而是 Python 把 keys/flags 错排在 47 个 float 之后，C++ 实际布局是在 35 个
float 之后，导致后续字段整体错位。已把 Python 格式修正为
`<IIQQ35fII12f12B12IBBHi4B15HI`，包长 326 bytes。

协议修正后的 `sim-20260814-160616` 已证明运动链路可用：policy 以 vx=0.25 m/s
前进约 2 秒后松键回 learned hold，base x 约从 0.188 m 移动到 0.58 m。但停止时又
发现第二个问题：控制中心原来先停 policy，gateway 将正常退出误判为 policy timeout，
切到 SAFE_DAMPING 并让录像末尾失去承重。该轮不能再标为完整通过。

已把仿真停止顺序改为：先请求 MuJoCo/录像冻结结束（policy 仍持续发命令），再正常
停止 gateway，等待录像封装，最后停止 policy。最终 `sim-20260814-161622` 完整通过：
0.25 m/s 前进 1 秒后回 learned hold，最后四秒 base z 为 0.5208/0.5244/0.5220/
0.5199 m；末帧机器人直立，不是黑屏，gateway 无 SAFE_DAMPING、policy timeout 或
fault sidecar。录像和图表均已保存：

- `logs/videos/sim-20260814-161622-flat.mp4`
- `logs/control-center/sim-20260814-161622/policy.csv`
- `logs/control-center/sim-20260814-161622/gateway.csv`
- `logs/control-center/sim-20260814-161622/policy-tracking-tracking.svg`
- `logs/control-center/sim-20260814-161622/policy-tracking-raw-action.svg`

最终验收：C++ 全量构建成功，CTest 1/1，Python `207 passed`，`git diff --check`
通过。

### 当前卡点和限制

1. 真机自研激活目前是有意锁住的：规范配置没有真实 B2 的审批参数、序列号、固件和
   当前 policy SHA。不能把仿真参数自动标成真机已审批参数，也不能为了通过界面删除
   这层门禁。
2. B2 LowState 没有逐电机电流字段。现阶段能直接保护的是 tau_est 和 BMS 总电流
   原始值；`current_limit` 只作为未来硬件/固件适配的逐关节审批占位，尚未形成逐电机
   电流闭环。
3. 进程内 watchdog 能处理控制线程停滞，但进程崩溃、主机断电或通信设备彻底失效
   仍必须依赖电机固件超时行为、物理急停、机械支撑和现场保护，软件不能替代这些措施。
4. 本轮没有在真机验证新增原生动作按钮、故障锁存或恢复路径；这些功能第一次上机必须
   在吊架/防跌落和急停就绪条件下逐项做静态验收，不能直接开始楼梯行走。

### 下一步

硬件复核人员应针对指定 B2 和指定 ONNX 文件逐项填写并复核所选配置（默认模板为
`configs/b2-reference-profile.json`，附加配置放在 `configs/hardware-profiles/`），
特别是关节 Kp/Kd、slew、软边界、dq/tau_est、
跟踪误差、温度、BMS 原始电流、超时和恢复时间，并记录 policy SHA、机身序列号和
固件版本。审批完成后先做“只读连接→原生动作按钮静态检查→零命令 learned hold→
人为制造非运动通信超时验证 SAFE_DAMPING/黑匣子→人工锁存恢复”，全部通过后才进行
低速平地真机测试。任何真机动作都必须由现场操作者明确发起。


## 47. 2026-08-14：取消固定单一安全参数文件

控制中心已从“只认 `configs/b2-reference-profile.json`”改为多配置选择。运行时自动
发现默认参考文件和 `configs/hardware-profiles/*.json`，Hardware Activation Gate
新增配置下拉框；每次真机只读连接会把用户选中的仓库内配置路径固定到该会话，并将
其传给 policy console 和 C++ gateway。网页不能传入目录外任意路径，活动会话期间
不能更换配置。

此次取消的是“单一文件来源”限制，不是取消安全校验。每套配置仍独立校验 schema、
完整参数、approved/activation_permitted、Policy SHA、机身/固件和复核信息。现有
参考配置仍为 reference_only，因此真机自研接管保持 LOCKED。本轮未连接或操作真机。

## 2026-08-18：安装 Stack10 resume policy（model_31997）

已将用户提供的 bundle 复制到：

`policies/b2_stack10_resume_s42_model31997/`

该策略不是旧版 45 维 actor 的替换文件，而是独立的 `b2_stack10_v1`：

- ONNX 输入 `[1, 423]`，输出 `[1, 12]`；
- 50 Hz policy tick；
- 10 帧 term-major 历史；
- reset 时清空 previous action，并用首个有效帧填充历史；
- previous action 保存未变换的 raw actor output；
- joint order、Q_DEFAULT 和 action scale 按 bundle contract 保留。

控制台已新增 `src/b2_control/stack10_adapter.py`，并将 bundle、ONNX runtime、
policy core、CLI 和策略注册器接入可选 adapter。原有 `b2_45x12_v1` 保持不变，可回滚。

离线验证：

- bundle SHA-256 校验通过；
- `cli_verify` 的 `423 -> 12` golden reset inference 通过；
- 生产 `B2PolicyCore` 两个 tick 输出 `[1,423]`，tick-1/tick-2 action 与 contract 一致；
- Python 全量回归：`243 passed`。

当前仍不能直接真机部署：现有硬件 profile 审批的 Policy SHA 是旧策略
`688bdbc...`，新策略 SHA 是 `bbb474e...`，且 Stack10 尚未经过该 B2 的硬件审批、
shadow replay 和现场安全复核。控制台会显示该策略可用，但真机 activation gate 仍保持
LOCKED；不得通过修改 SHA 或删除门禁伪造批准。当前正在运行的真机会话没有被重启、切换或停止。
