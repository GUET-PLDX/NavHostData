# HostDataLink

## LibXR 2.0 静态装配

使用 xrobot 1.0.1 的 `module` / 有序 `args`；构造函数显式注入外设与服务，实例按依赖顺序创建。参数名和顺序以头文件中的构造函数为准，固件示例见 `User/RobotConfig/`。

保留 PLDX 控制、协议、超时与离线保护。

导航包保留 Fx/Fy/Mz 力协议，写入 `CMD::Data::chassis` 的 `fx_n/fy_n/mz_nm/ff_valid`；运动目标 `chassis.vx/vy/wz` 默认零。CMD 仲裁后发布复合底盘命令，Omni 将其叠加到速度反馈。非零速度轨迹需要上游补充速度目标。

控制输入回调在模块锁内缓存导航数据，行为回调继续分发模式候选及决策 Topic；这些接收路径不读写 CMD。构造时注册的输入回调由 `CMD::UpdateInputs()` 执行，在锁内聚合、释放锁后提交，通过 `FeedAIChassis` 只更新 AI 底盘分片。导航力按 150 ms 判定新鲜度。没有命令软件定时器。导航模式作为待选值发布，实际是否采用由 DualBoard 的一致控制结果决定。

视觉目标与开火改由 TalosLink 通过 `FeedAIGimbal` 提交；本模块不再订阅 `target_euler` / `fire_notify`。
