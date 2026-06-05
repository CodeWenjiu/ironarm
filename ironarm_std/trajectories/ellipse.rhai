// 三维空间椭圆 — 可直接 import 或在 main.rhai 中 import。
// 参数在 sample 函数内部定义，避免 Rhai 模块导入时的变量作用域问题。
fn sample(t) {
    let CX = -0.4;
    let CY = 0.0;
    let CZ = 0.35;
    let NX = 0.5;
    let NY = 0.3;
    let NZ = 0.8;
    let R = 0.12;
    let PERIOD = 5.0;

    let mag = (NX * NX + NY * NY + NZ * NZ).sqrt();
    let nx = NX / mag;
    let ny = NY / mag;
    let nz = NZ / mag;

    let rx = 0.0; let ry = 0.0; let rz = 1.0;
    if nx.abs() < 0.9 { rx = 1.0; rz = 0.0; }

    let ux = ry * nz - rz * ny;
    let uy = rz * nx - rx * nz;
    let uz = rx * ny - ry * nx;
    let um = (ux * ux + uy * uy + uz * uz).sqrt();
    let ux = ux / um; let uy = uy / um; let uz = uz / um;

    let vx = ny * uz - nz * uy;
    let vy = nz * ux - nx * uz;
    let vz = nx * uy - ny * ux;

    let w = t * 2.0 * 3.141592653589793 / PERIOD;
    let cw = cos(w); let sw = sin(w);
    #{
        x: CX + R * (cw * ux + sw * vx),
        y: CY + R * (cw * uy + sw * vy),
        z: CZ + R * (cw * uz + sw * vz),
        rx: 0.0, ry: 0.0, rz: 0.0,
    }
}
