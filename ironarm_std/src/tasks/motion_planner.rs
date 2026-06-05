//! 运动规划器——加载 Rhai 脚本，按配置参数生成笛卡尔路径点。
//!
//! 支持热重载：运行时修改 .rhai 脚本文件，规划器自动检测并重编译，
//! 无需重启 Copper 运行时。
//!
//! 配置键：
//! - `"script"`: 轨迹脚本路径（相对于 ironarm_std 目录）
//! - `"wp_rate_hz"`: 路径点输出频率（默认 10）

use std::path::PathBuf;
use std::time::SystemTime;

use cu29::prelude::*;
use ironarm_core::clock;
use ironarm_core::messages::CartesianWaypoint;
use log;
use rhai::{AST, Engine, Scope};

#[derive(Reflect)]
pub struct MotionPlanner {
    start: f32,
    wp_interval: f32,
    last_wp_time: f32,
    last_wp: CartesianWaypoint,
    #[reflect(ignore)]
    engine: Engine,
    #[reflect(ignore)]
    ast: AST,
    #[reflect(ignore)]
    scope: Scope<'static>,
    #[reflect(ignore)]
    script_path: PathBuf,
    /// 轨迹目录（监视其中所有 .rhai 文件的变更）。
    #[reflect(ignore)]
    trajectories_dir: PathBuf,
    #[reflect(ignore)]
    last_mtime: Option<SystemTime>,
    #[reflect(ignore)]
    next_reload_check: f32,
}

impl Freezable for MotionPlanner {}

impl CuSrcTask for MotionPlanner {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CartesianWaypoint);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.unwrap_or_else(|| panic!("MotionPlanner 需要 config"));

        let script_rel = cfg
            .get::<String>("script")
            .ok()
            .flatten()
            .unwrap_or_else(|| "trajectories/ellipse.rhai".into());

        let wp_rate_hz = cfg.get::<f64>("wp_rate_hz").ok().flatten().unwrap_or(10.0) as f32;
        let wp_interval = if wp_rate_hz > 0.0 {
            1.0 / wp_rate_hz
        } else {
            0.0
        };

        let script_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(&script_rel);
        let trajectories_dir = script_path
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."))
            .to_path_buf();

        let engine = {
            let mut e = Engine::new();
            let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
            let resolver = rhai::module_resolvers::FileModuleResolver::new_with_path(manifest);
            e.set_module_resolver(resolver);
            e
        };
        let (ast, scope, _) = compile_script(&engine, &script_path)?;

        let mut scope = scope;
        let map = engine
            .call_fn::<rhai::Map>(&mut scope, &ast, "sample", (0.0f64,))
            .unwrap_or_default();
        let last_wp = map_to_wp(&map);

        let now = clock::now_secs();
        let initial_mtime = latest_rhai_mtime(&trajectories_dir);
        Ok(Self {
            start: now,
            wp_interval,
            last_wp_time: now - wp_interval,
            last_wp,
            engine,
            ast,
            scope,
            script_path,
            trajectories_dir,
            last_mtime: initial_mtime,
            next_reload_check: 0.0,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = clock::now_secs() - self.start;

        if t >= self.next_reload_check {
            self.next_reload_check = t + 1.0;
            self.try_reload();
        }

        if t - self.last_wp_time >= self.wp_interval {
            let map = self
                .engine
                .call_fn::<rhai::Map>(&mut self.scope, &self.ast, "sample", (t as f64,))
                .unwrap_or_default();
            self.last_wp = map_to_wp(&map);
            self.last_wp_time = t;
        }
        output.set_payload(self.last_wp);
        Ok(())
    }
}

impl MotionPlanner {
    /// 扫描 trajectories_dir 下所有 .rhai 文件，若任一变更新于上次编译时间则重载。
    fn try_reload(&mut self) {
        let latest = match latest_rhai_mtime(&self.trajectories_dir) {
            Some(t) => t,
            None => return,
        };
        if Some(latest) == self.last_mtime {
            return;
        }

        let script = match std::fs::read_to_string(&self.script_path) {
            Ok(s) => s,
            Err(e) => {
                log::error!("读取脚本文件失败: {e}");
                return;
            }
        };

        // 创建全新 Engine 以清空模块缓存，否则 import 的子模块不会重新读取
        let mut engine = Engine::new();
        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let resolver = rhai::module_resolvers::FileModuleResolver::new_with_path(manifest);
        engine.set_module_resolver(resolver);

        match engine.compile(&script) {
            Ok(new_ast) => {
                self.engine = engine;
                self.ast = new_ast;
                self.scope = Scope::new();
                self.last_mtime = Some(latest);
                log::info!("Rhai 脚本已热重载: {}", self.script_path.display());
            }
            Err(e) => {
                log::error!("Rhai 脚本编译失败，保持旧版本: {e}");
            }
        }
    }
}

/// 返回目录下所有 .rhai 文件的最新修改时间。
fn latest_rhai_mtime(dir: &PathBuf) -> Option<SystemTime> {
    let entries = std::fs::read_dir(dir).ok()?;
    let mut latest: Option<SystemTime> = None;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) == Some("rhai") {
            if let Ok(meta) = path.metadata() {
                if let Ok(mtime) = meta.modified() {
                    if latest.map_or(true, |prev| mtime > prev) {
                        latest = Some(mtime);
                    }
                }
            }
        }
    }
    latest
}

fn compile_script(engine: &Engine, path: &PathBuf) -> CuResult<(AST, Scope<'static>, SystemTime)> {
    let script = std::fs::read_to_string(path)
        .map_err(|e| CuError::from(format!("读取脚本文件失败 {}: {e}", path.display())))?;
    let ast = engine
        .compile(&script)
        .map_err(|e| CuError::from(format!("Rhai 脚本编译失败: {e}")))?;
    let mtime = std::fs::metadata(path)
        .and_then(|m| m.modified())
        .unwrap_or(SystemTime::UNIX_EPOCH);
    Ok((ast, Scope::new(), mtime))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_rhai_map() {
        let engine = Engine::new();
        let script = r#"fn sample(t) { #{ x: -0.4, y: 0.0, z: 0.35, rx: 0.1, ry: 0.2, rz: 0.3 } }"#;
        let ast = engine.compile(script).expect("compile");
        let mut scope = Scope::new();
        let map = engine
            .call_fn::<rhai::Map>(&mut scope, &ast, "sample", (0.0f64,))
            .expect("call_fn");
        let wp = map_to_wp(&map);
        assert!((wp.x + 0.4).abs() < 0.001, "x={}", wp.x);
        assert!((wp.y).abs() < 0.001, "y={}", wp.y);
        assert!((wp.z - 0.35).abs() < 0.001, "z={}", wp.z);
        assert!((wp.rx - 0.1).abs() < 0.001, "rx={}", wp.rx);
    }

    #[test]
    fn test_load_main_rhai() {
        let script_path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("trajectories/main.rhai");
        let engine = {
            let mut e = Engine::new();
            let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
            let resolver = rhai::module_resolvers::FileModuleResolver::new_with_path(dir);
            e.set_module_resolver(resolver);
            e
        };
        let (ast, mut scope, _mtime) =
            compile_script(&engine, &script_path).expect("compile_script");
        let map = engine
            .call_fn::<rhai::Map>(&mut scope, &ast, "sample", (0.0f64,))
            .expect("call_fn");
        let wp = map_to_wp(&map);
        assert!((wp.x + 0.4).abs() < 0.01, "x={}, expected ≈-0.4", wp.x);
        assert!(wp.z > 0.3, "z={}, expected >0.3", wp.z);
    }

    #[test]
    fn test_latest_rhai_mtime_finds_files() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("trajectories");
        let mtime = latest_rhai_mtime(&dir);
        assert!(mtime.is_some(), "应在 trajectories/ 中找到 .rhai 文件");
    }

    #[test]
    fn test_latest_rhai_mtime_returns_max() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("trajectories");
        let before = latest_rhai_mtime(&dir).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let tmp = dir.join("_test_tmp.rhai");
        std::fs::write(&tmp, b"// temp").unwrap();
        let after = latest_rhai_mtime(&dir).unwrap();
        std::fs::remove_file(&tmp).ok();
        assert!(
            after > before,
            "新建文件后 mtime 应该变大: {before:?} → {after:?}"
        );
    }

    /// 复现：修改 import 的模块文件后，重新编译应该反映模块内容变更
    #[test]
    fn test_recompile_picks_up_module_change() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("trajectories");
        // 创建临时主脚本和模块
        let main_path = dir.join("_test_main.rhai");
        let mod_path = dir.join("_test_mod.rhai");
        std::fs::write(
            &mod_path,
            "fn sample(t) { #{ x: 1.0, y: 0.0, z: 0.0, rx: 0.0, ry: 0.0, rz: 0.0 } }",
        )
        .unwrap();
        std::fs::write(
            &main_path,
            "import \"trajectories/_test_mod\" as m; fn sample(t) { m::sample(t) }",
        )
        .unwrap();

        // 编译
        let engine = {
            let mut e = Engine::new();
            let resolver = rhai::module_resolvers::FileModuleResolver::new_with_path(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")),
            );
            e.set_module_resolver(resolver);
            e
        };
        let (ast, mut scope, _) = compile_script(&engine, &main_path).unwrap();
        let map = engine
            .call_fn::<rhai::Map>(&mut scope, &ast, "sample", (0.0f64,))
            .unwrap();
        let wp = map_to_wp(&map);
        assert!((wp.x - 1.0).abs() < 0.001, "初始 x 应为 1.0, got {}", wp.x);

        // 修改模块文件
        std::fs::write(
            &mod_path,
            "fn sample(t) { #{ x: 2.0, y: 0.0, z: 0.0, rx: 0.0, ry: 0.0, rz: 0.0 } }",
        )
        .unwrap();

        // 重新编译主脚本——用新 Engine（模拟 try_reload 修复后的行为）
        let script = std::fs::read_to_string(&main_path).unwrap();
        let engine2 = {
            let mut e = Engine::new();
            let resolver = rhai::module_resolvers::FileModuleResolver::new_with_path(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")),
            );
            e.set_module_resolver(resolver);
            e
        };
        let new_ast = engine2.compile(&script).unwrap();
        let mut scope2 = Scope::new();
        let map2 = engine2
            .call_fn::<rhai::Map>(&mut scope2, &new_ast, "sample", (0.0f64,))
            .unwrap();
        let wp2 = map_to_wp(&map2);

        std::fs::remove_file(&main_path).ok();
        std::fs::remove_file(&mod_path).ok();

        assert!(
            (wp2.x - 2.0).abs() < 0.001,
            "重编译后 x 应为 2.0（反映模块修改），但 got {}。Rhai 缓存了旧模块！",
            wp2.x
        );
    }
}

fn map_to_wp(m: &rhai::Map) -> CartesianWaypoint {
    fn get_f32(m: &rhai::Map, key: &str) -> f32 {
        m.get(key)
            .and_then(|v| v.as_float().ok())
            .map(|f| f as f32)
            .unwrap_or(0.0)
    }
    CartesianWaypoint {
        x: get_f32(m, "x"),
        y: get_f32(m, "y"),
        z: get_f32(m, "z"),
        rx: get_f32(m, "rx"),
        ry: get_f32(m, "ry"),
        rz: get_f32(m, "rz"),
    }
}
