// 关节运动脚本（保存后自动热重载）
//
// 每帧调用 tick(tick, dt)，返回 [joint0_angle, joint1_angle]
//   tick: 帧计数  dt: 时间步长（秒）
// 可用常量：PI（π）
// 单位：弧度

fn tick(tick, dt) {
    let t = tick * dt;

    // 关节 0：正弦摆动，振幅 0.5 rad，周期 3 秒
    let a0 = 0.5 * sin(t * 2.0 * PI / 3.0);

    // 关节 1：方波，每 2 秒翻转
    let cycle = t / 2.0;
    let phase = cycle - floor(cycle);
    let a1 = if phase < 0.5 { 0.8 } else { -0.8 };

    [a0, a1]
}
