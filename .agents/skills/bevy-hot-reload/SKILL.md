---
name: bevy-hot-reload
description: Implement asset hot-reload in Bevy 0.18 for custom asset types (RON, JSON, etc.). Use when the user wants to detect file changes and rebuild entities at runtime without restart.
---

# Bevy 0.18 自定义 Asset 热重载

## 核心原则

Bevy 的热重载对**内置 asset 类型**（Mesh、Scene 等）是全自动的——修改文件后渲染自动更新。但对**自定义 asset 类型**，需要手动实现 6 个步骤，缺一不可。

## 必须的 6 个步骤

### 1. 启用 `file_watcher` feature（Cargo.toml）

```toml
[workspace.dependencies]
bevy = { version = "0.18.0", default-features = false, features = [
    # ... 其他 features ...
    "file_watcher",  # 不加这行，Bevy 根本不监视文件
] }
```

**验证**：`cargo tree -e features | grep file_watcher` 必须有输出。

### 2. 启用 `AssetPlugin` 的文件监听（main.rs）

`file_watcher` feature 只是把代码编译进去，`watch_for_changes_override` 才是真正打开监听：

```rust
use bevy::asset::AssetPlugin;

app.add_plugins(
    DefaultPlugins
        .set(AssetPlugin {
            watch_for_changes_override: Some(true),
            ..Default::default()
        })
);
```

**验证**：启动后不应出现 `AssetWatcher configured` 的 WARN。

### 3. `init_asset` 注册类型（main.rs）

```rust
use bevy::asset::AssetApp;

app.init_asset::<MyConfig>();
```

Bevy 0.18 必须显式调用，否则 `asset_server.load()` 直接 panic："asset type has not been initialized"。

### 4. 实现 `AssetLoader` trait（config 定义文件）

```rust
use bevy::asset::{AssetLoader, io::Reader, LoadContext};
use bevy::prelude::*;
use serde::Deserialize;

#[derive(TypePath)]              // AssetLoader 要求 TypePath
pub struct MyConfigLoader;

impl AssetLoader for MyConfigLoader {
    type Asset = MyConfig;
    type Settings = ();
    type Error = Box<dyn std::error::Error + Send + Sync + 'static>;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let cfg: MyConfig = ron::de::from_bytes(&bytes)?;
        Ok(cfg)
    }

    fn extensions(&self) -> &[&str] {
        &["ron"]
    }
}

#[derive(Asset, Deserialize, TypePath, Clone)]
pub struct MyConfig { /* ... */ }
```

关键点：
- `Asset` derive 是必须的
- `TypePath` derive 在 `AssetLoader` 和 `Asset` 上都需要
- `Clone` derive 在热重载比对时需要

### 5. 注册 loader（main.rs）

```rust
app.register_asset_loader(MyConfigLoader);
```

没有 loader，Bevy 不知道如何把文件字节转成你的类型。`asset_server.load()` 返回的 handle 永远不会 resolve。

### 6. 检测变更并重建实体

**不要依赖** `Res::is_changed()`、`AssetChanged` filter、或 `MessageReader<AssetEvent>`——这些在 Bevy 0.18 的自定义 asset 场景下行为不可靠。

**正确做法：数据快照比对**。每帧读取 asset 内容，和上次快照比较，不同则重建：

```rust
use bevy::prelude::*;

fn spawn_from_config(
    mut commands: Commands,
    configs: Res<Assets<MyConfig>>,
    handle: Res<MyConfigHandle>,
    mut last_cfg: Local<Option<MyConfig>>,  // 快照
    mut entities: Local<Option<MyEntities>>,  // 追踪已创建的实体
) {
    let Some(cfg) = configs.get(&handle.0) else {
        return;  // asset 还没加载完
    };

    // 比对新旧配置
    if let Some(ref last) = *last_cfg {
        if last.field1 == cfg.field1
            && last.field2 == cfg.field2
            // ... 逐字段比对 ...
        {
            return;  // 无变更
        }
    }

    // 有变更或首次加载 → 保存快照
    *last_cfg = Some(cfg.clone());

    // 销毁旧实体
    if let Some(e) = entities.take() {
        commands.entity(e.child).despawn();
        commands.entity(e.parent).despawn();
    }

    // 创建新实体
    let parent = commands.spawn((...)).id();
    let child = commands.spawn((...)).id();
    *entities = Some(MyEntities { parent, child });

    // 如果需要其他 system 访问这些实体，insert 为 resource
    commands.insert_resource(MyEntities { parent, child });
}

#[derive(Resource, Clone)]
struct MyConfigHandle(pub Handle<MyConfig>);

#[derive(Resource, Clone)]
struct MyEntities {
    parent: Entity,
    child: Entity,
}
```

**为什么不用 `is_changed()`？**
- `Assets<T>` 的变更由 `AssetEventSystems`（在 `PostUpdate`）处理
- `Update` 中的 system 读取 `Res<Assets<T>>::is_changed()` 有帧延迟
- 实际测试中经常返回 false，原因与 tick 机制有关

**为什么不用 `MessageReader<AssetEvent>`？**
- `AssetEvent` 类型是 `bevy::asset::AssetEvent`（不是 `EventReader`，Bevy 0.18 已重命名为 `MessageReader`）
- 事件在 `PostUpdate` 的 `AssetEventSystems` 中发出
- 在 `Update` 中读取需要跨帧，容易丢事件

## 完整检查清单

在实现热重载后，按顺序检查：

- [ ] `cargo tree -e features | grep file_watcher` 有输出
- [ ] 启动日志**没有** `AssetWatcher configured` 的 WARN
- [ ] 启动日志**有** asset loader 的加载日志（手动添加的 diagnostic log）
- [ ] 修改 RON 文件保存后，loader 的加载日志**再次出现**
- [ ] 实体重建日志出现，模型在 GUI 中更新

## 诊断日志模板

在 `AssetLoader::load()` 和重建 system 中加 `bevy::log::warn!`——选 `warn` 级别确保可见：

```rust
// 在 loader 中
bevy::log::warn!("[MyLoader] loaded: field={:?}", cfg.field);

// 在检测 system 中
bevy::log::warn!("[spawn] CHANGE: old={:?} new={:?}", last.field, cfg.field);
bevy::log::warn!("[spawn] FIRST LOAD: field={:?}", cfg.field);
```

## 常见陷阱

1. **`file_watcher` feature 没加到 Cargo.toml**——`cargo tree` 验证，不要信任编辑器中的文件内容
2. **`watch_for_changes_override` 字段名**——Bevy 0.18 是 `watch_for_changes_override: Some(true)`，不是 `watch_for_changes: true`
3. **`AssetLoader` 缺少 `TypePath` derive**——编译器会报错，但容易漏看
4. **忘记 `register_asset_loader`**——能编译但 asset 永远不会加载
5. **`EventReader` 已改名为 `MessageReader`**——Bevy 0.18 的 breaking change
6. **直接用 `cargo run` 而不是 `just` 启动**——确保工作目录正确，asset 路径才能找到
