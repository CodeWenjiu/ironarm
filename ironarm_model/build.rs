//! Build script: parse ur5e.xml and emit PoE kinematics constants.
//!
//! Walks the MuJoCo body tree at q=0 (all joints at zero), collecting:
//!   - Joint origins in world frame → link offsets p[i]
//!   - Joint axes transformed to world frame → screw axes h[i]
//!
//! The output is included via `include!(concat!(env!("OUT_DIR"), "/poe_params.rs"))`.

use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// 3-D vector helpers
// ---------------------------------------------------------------------------

type Vec3 = [f32; 3];

fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

// ---------------------------------------------------------------------------
// Quaternion helpers (w, x, y, z)
// ---------------------------------------------------------------------------

type Quat = [f32; 4];

/// Parse "w x y z" string → [w, x, y, z].
fn parse_quat(s: &str) -> Quat {
    let v: Vec<f32> = s
        .split_whitespace()
        .filter_map(|t| t.parse().ok())
        .collect();
    if v.len() == 4 {
        [v[0], v[1], v[2], v[3]]
    } else {
        [1.0, 0.0, 0.0, 0.0] // identity
    }
}

/// Parse "x y z" string → [x, y, z].
fn parse_vec3(s: &str) -> Vec3 {
    let v: Vec<f32> = s
        .split_whitespace()
        .filter_map(|t| t.parse().ok())
        .collect();
    if v.len() == 3 {
        [v[0], v[1], v[2]]
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Normalise a quaternion.
fn quat_normalise(q: Quat) -> Quat {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-10 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// Multiply two quaternions: q1 * q2  (applies q2 then q1).
fn quat_mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Rotate vector v by unit quaternion q = (w, x, y, z).
fn quat_rotate(q: Quat, v: Vec3) -> Vec3 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    [
        v[0] * (1.0 - 2.0 * (yy + zz)) + v[1] * (2.0 * (xy - wz)) + v[2] * (2.0 * (xz + wy)),
        v[0] * (2.0 * (xy + wz)) + v[1] * (1.0 - 2.0 * (xx + zz)) + v[2] * (2.0 * (yz - wx)),
        v[0] * (2.0 * (xz - wy)) + v[1] * (2.0 * (yz + wx)) + v[2] * (1.0 - 2.0 * (xx + yy)),
    ]
}

// ---------------------------------------------------------------------------
// Default-class resolution
// ---------------------------------------------------------------------------

/// Build a map from class name → default joint axis (Vec3).
fn build_default_axes(doc: &roxmltree::Document) -> HashMap<String, Vec3> {
    let mut map: HashMap<String, Vec3> = HashMap::new();

    for node in doc.descendants().filter(|n| n.has_tag_name("default")) {
        let class = node.attribute("class").unwrap_or("");
        // Inherit from parent class first
        let parent_class = node
            .parent()
            .and_then(|p| p.attribute("class"))
            .unwrap_or("");
        if !parent_class.is_empty() {
            if let Some(&parent_axis) = map.get(parent_class) {
                map.insert(class.to_string(), parent_axis);
            }
        }
        // Then check for joint axis override in this class
        if let Some(joint) = node.children().find(|c| c.has_tag_name("joint")) {
            if let Some(axis_str) = joint.attribute("axis") {
                let axis = parse_vec3(axis_str);
                if axis != [0.0, 0.0, 0.0] {
                    map.insert(class.to_string(), axis);
                }
            }
        }
    }

    map
}

/// Resolve the effective joint axis for a joint node, given class defaults.
fn resolve_joint_axis(
    joint_node: &roxmltree::Node,
    body_childclass: &str,
    defaults: &HashMap<String, Vec3>,
) -> Vec3 {
    // 1. Explicit axis on the joint itself
    if let Some(axis_str) = joint_node.attribute("axis") {
        let a = parse_vec3(axis_str);
        if a != [0.0, 0.0, 0.0] {
            return a;
        }
    }
    // 2. Joint's own class
    let class = joint_node.attribute("class").unwrap_or("");
    if !class.is_empty() {
        if let Some(&axis) = defaults.get(class) {
            return axis;
        }
    }
    // 3. Body's childclass
    if !body_childclass.is_empty() {
        if let Some(&axis) = defaults.get(body_childclass) {
            return axis;
        }
    }
    // 4. MuJoCo global default for hinge joints
    [0.0, 1.0, 0.0]
}

// ---------------------------------------------------------------------------
// Body tree walk — collect joints and site
// ---------------------------------------------------------------------------

struct BodyState {
    pos: Vec3,
    joint_axis: Option<Vec3>,
    joint_name: String,
}

fn walk_body(
    node: &roxmltree::Node,
    world_pos: Vec3,
    world_quat: Quat,
    inherited_childclass: &str,
    defaults: &HashMap<String, Vec3>,
    joints: &mut Vec<BodyState>,
    site: &mut Option<BodyState>,
) {
    // Compute this body's world frame
    let body_pos_str = node.attribute("pos").unwrap_or("0 0 0");
    let body_pos_local = parse_vec3(body_pos_str);
    let body_pos = vec3_add(world_pos, quat_rotate(world_quat, body_pos_local));

    let body_quat_str = node.attribute("quat").unwrap_or("1 0 0 0");
    let body_quat_local = quat_normalise(parse_quat(body_quat_str));
    let body_quat = quat_mul(world_quat, body_quat_local);

    let childclass = node.attribute("childclass").unwrap_or(inherited_childclass);

    // Collect joint if present
    if let Some(joint_node) = node.children().find(|c| c.has_tag_name("joint")) {
        let joint_name = joint_node.attribute("name").unwrap_or("").to_string();
        if !joint_name.is_empty() {
            let local_axis = resolve_joint_axis(&joint_node, childclass, defaults);
            let world_axis = quat_rotate(body_quat, local_axis);
            joints.push(BodyState {
                pos: body_pos,
                joint_axis: Some(world_axis),
                joint_name,
            });
        }
    }

    // Collect attachment site (tool frame)
    if let Some(site_node) = node
        .children()
        .find(|c| c.has_tag_name("site") && c.attribute("name") == Some("attachment_site"))
    {
        let sp = site_node.attribute("pos").unwrap_or("0 0 0");
        let site_pos_local = parse_vec3(sp);
        let site_world_pos = vec3_add(body_pos, quat_rotate(body_quat, site_pos_local));
        *site = Some(BodyState {
            pos: site_world_pos,
            joint_axis: None,
            joint_name: "attachment_site".into(),
        });
    }

    // Recurse
    for child in node.children().filter(|c| c.has_tag_name("body")) {
        walk_body(
            &child, body_pos, body_quat, childclass, defaults, joints, site,
        );
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let xml_path = manifest.join("ur5e.xml");

    println!("cargo:rerun-if-changed=ur5e.xml");

    let xml = fs::read_to_string(&xml_path).expect("failed to read ur5e.xml");
    let doc = roxmltree::Document::parse(&xml).expect("invalid XML");

    let defaults = build_default_axes(&doc);

    // Find the worldbody
    let wb = doc
        .descendants()
        .find(|n| n.has_tag_name("worldbody"))
        .expect("no worldbody in ur5e.xml");

    let mut joints: Vec<BodyState> = Vec::new();
    let mut site: Option<BodyState> = None;

    // Walk all top-level bodies in worldbody
    for child in wb.children().filter(|c| c.has_tag_name("body")) {
        walk_body(
            &child,
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            "",
            &defaults,
            &mut joints,
            &mut site,
        );
    }

    let attachment = site.expect("attachment_site not found in ur5e.xml");
    let n = joints.len();
    assert!(n >= 6, "expected >= 6 joints, found {n}");

    // Build link offsets p[0..n] and p_tool
    // p[0] = first joint position
    // p[i] = joints[i].pos - joints[i-1].pos  for i=1..5
    // p_tool = attachment.pos - joints[last].pos
    let mut p: Vec<Vec3> = Vec::new();
    p.push(joints[0].pos); // p[0]
    for i in 1..n {
        p.push(vec3_sub(joints[i].pos, joints[i - 1].pos));
    }
    let p_tool = vec3_sub(attachment.pos, joints[n - 1].pos);

    // Screw axes
    let h: Vec<Vec3> = joints
        .iter()
        .map(|j| j.joint_axis.unwrap_or([0.0, 1.0, 0.0]))
        .collect();

    // -------------------------------------------------------------------
    // Emit Rust constants
    // -------------------------------------------------------------------
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let dest = out_dir.join("poe_params.rs");

    let mut code = String::new();
    code.push_str("// Auto-generated from ur5e.xml — do not edit.\n\n");

    code.push_str(&format!("pub const N_JOINTS: usize = {n};\n\n"));

    // Joint names
    code.push_str("pub const JOINT_NAMES: &[&str] = &[");
    for (i, j) in joints.iter().enumerate() {
        if i > 0 {
            code.push_str(", ");
        }
        code.push_str(&format!("\"{}\"", j.joint_name));
    }
    code.push_str("];\n\n");

    // Screw axes
    code.push_str(&format!("pub const SCREW_AXES: [glam::Vec3; {n}] = [\n"));
    for a in &h {
        code.push_str(&format!(
            "    glam::Vec3::new({:.6}, {:.6}, {:.6}),\n",
            a[0], a[1], a[2]
        ));
    }
    code.push_str("];\n\n");

    // Link offsets (joint-to-joint)
    code.push_str(&format!(
        "pub const LINK_OFFSETS: [glam::Vec3; {}] = [\n",
        n + 1
    ));
    for o in &p {
        code.push_str(&format!(
            "    glam::Vec3::new({:.6}, {:.6}, {:.6}),\n",
            o[0], o[1], o[2]
        ));
    }
    // Tool offset
    code.push_str(&format!(
        "    glam::Vec3::new({:.6}, {:.6}, {:.6}),\n",
        p_tool[0], p_tool[1], p_tool[2]
    ));
    code.push_str("];\n");

    fs::write(&dest, code).expect("failed to write poe_params.rs");
}
