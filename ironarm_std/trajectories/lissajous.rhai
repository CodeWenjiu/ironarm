// 利萨如 ∞ 字曲线 — 可直接 import 或在 main.rhai 中 import。
// 参数在 sample 函数内部定义，避免 Rhai 模块导入时的变量作用域问题。
fn sample(t) {
    let CX = -0.4;
    let CY = 0.0;
    let CZ = 0.35;
    let A = 0.15;
    let B = 0.10;
    let C = 0.05;
    let PERIOD = 6.0;

    let w = t * 2.0 * 3.141592653589793 / PERIOD;
    #{
        x: CX + A * sin(w),
        y: CY + B * sin(2.0 * w) * 0.5,
        z: CZ + C * cos(0.5 * w),
        rx: -10.0,
        ry: -10.0,
        rz: -10.0,
    }
}
