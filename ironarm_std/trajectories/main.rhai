// 主入口——修改 import 行即可切换轨迹。
// 支持热重载：保存此文件后约 1 秒，Copper 运行时即自动更新轨迹。
import "trajectories/lissajous" as traj;

fn sample(t) {
    traj::sample(t)
}
