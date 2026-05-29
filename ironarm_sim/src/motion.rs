//! Rhai 脚本驱动的关节运动引擎。
//!
//! 从 `assets/motion.rhai` 加载脚本，每帧调用 `tick(tick, dt) -> [f32; 2]`，
//! 返回两个关节的目标角度。脚本文件保存后自动热重载。

use bevy::log;
use bevy::prelude::*;
use rhai::{AST, Engine, Scope};
use std::fs;
use std::path::PathBuf;
use std::time::SystemTime;

/// 管理 Rhai 脚本的编译与执行。
#[derive(Resource)]
pub struct RhaiMotion {
    engine: Engine,
    ast: AST,
    script_path: PathBuf,
    last_modified: SystemTime,
}

impl RhaiMotion {
    /// 从默认路径加载脚本，编译并返回引擎。
    /// 路径相对于 crate 根目录（CARGO_MANIFEST_DIR）。
    pub fn load() -> Result<Self, String> {
        let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_else(|_| ".".to_string());
        let script_path = PathBuf::from(&manifest).join("assets/motion.rhai");
        let source = fs::read_to_string(&script_path)
            .map_err(|e| format!("Failed to read {:?}: {}", script_path, e))?;
        let last_modified = fs::metadata(&script_path)
            .and_then(|m| m.modified())
            .unwrap_or(SystemTime::UNIX_EPOCH);

        let mut engine = Engine::new();
        engine.set_optimization_level(rhai::OptimizationLevel::None);
        // 注册标准数学函数
        engine.register_fn("sin", |x: f32| x.sin());
        engine.register_fn("cos", |x: f32| x.cos());
        engine.register_fn("tan", |x: f32| x.tan());
        engine.register_fn("abs", |x: f32| x.abs());
        engine.register_fn("floor", |x: f32| x.floor());
        engine.register_fn("ceil", |x: f32| x.ceil());
        engine.register_fn("sqrt", |x: f32| x.sqrt());
        engine.register_fn("pow", |x: f32, y: f32| x.powf(y));
        engine.register_fn("clamp", |x: f32, min: f32, max: f32| x.clamp(min, max));
        engine.register_fn("lerp", |a: f32, b: f32, t: f32| a + (b - a) * t);

        let ast = engine
            .compile(&source)
            .map_err(|e| format!("Script compile error: {}", e))?;

        Ok(Self {
            engine,
            ast,
            script_path,
            last_modified,
        })
    }

    /// 检查脚本文件是否被修改，如果是则重新编译。
    /// 应在 `Update` 中每帧调用。
    pub fn try_reload(&mut self) {
        if let Ok(meta) = fs::metadata(&self.script_path) {
            if let Ok(modified) = meta.modified() {
                if modified > self.last_modified {
                    match fs::read_to_string(&self.script_path) {
                        Ok(source) => match self.engine.compile(&source) {
                            Ok(ast) => {
                                self.ast = ast;
                                self.last_modified = modified;
                            }
                            Err(e) => log::error!("[RhaiMotion] compile error: {}", e),
                        },
                        Err(e) => log::error!("[RhaiMotion] read error: {}", e),
                    }
                }
            }
        }
    }

    /// 调用脚本的 `tick(tick, dt)` 函数，返回 `[joint0_angle, joint1_angle]`。
    pub fn compute_angles(&self, tick: u64, dt: f32) -> Option<[f32; 2]> {
        let mut scope = Scope::new();
        scope.push_constant("PI", std::f32::consts::PI);
        match self
            .engine
            .call_fn::<rhai::Dynamic>(&mut scope, &self.ast, "tick", (tick as f32, dt))
        {
            Ok(result) => match result.into_typed_array::<f32>() {
                Ok(arr) if arr.len() >= 2 => Some([arr[0], arr[1]]),
                Ok(arr) => {
                    log::error!("[RhaiMotion] tick() returned {} values (need 2)", arr.len());
                    None
                }
                Err(e) => {
                    log::error!("[RhaiMotion] tick() result is not a float array: {}", e);
                    None
                }
            },
            Err(e) => {
                log::error!("[RhaiMotion] tick() call failed: {}", e);
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motion_script() {
        let motion = RhaiMotion::load().expect("Failed to load motion.rhai");
        let a = motion.compute_angles(0, 0.016).expect("tick(0) failed");
        println!("tick=0:   [{:.4}, {:.4}]", a[0], a[1]);
        let b = motion.compute_angles(50, 0.016).expect("tick(50) failed");
        println!("tick=50:  [{:.4}, {:.4}]", b[0], b[1]);
        let c = motion.compute_angles(100, 0.016).expect("tick(100) failed");
        println!("tick=100: [{:.4}, {:.4}]", c[0], c[1]);
    }

    #[test]
    fn test_angles_diverge() {
        let motion = RhaiMotion::load().unwrap();
        for tick in 0..300 {
            let a = motion.compute_angles(tick, 0.016).unwrap();
            let diff = (a[0] - a[1]).abs();
            if diff > 0.1 {
                println!(
                    "tick={}: j0={:.4} j1={:.4} diff={:.4}",
                    tick, a[0], a[1], diff
                );
                return;
            }
        }
        panic!("j0 and j1 never diverged over 300 ticks");
    }
}
