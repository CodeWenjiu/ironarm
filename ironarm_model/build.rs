//! Build script: parse ironarm.xml and emit a Rust constants module.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let xml_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("ironarm.xml");
    let xml = fs::read_to_string(&xml_path).expect("failed to read ironarm.xml");
    let doc = roxmltree::Document::parse(&xml).expect("invalid XML");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let dest = out_dir.join("model_params.rs");

    let mut joints = Vec::new();
    let mut links = Vec::new();
    let mut base_z = 0.0f32;

    // Walk <worldbody> children for joints and geometry
    if let Some(wb) = doc.descendants().find(|n| n.has_tag_name("worldbody")) {
        for node in wb.descendants() {
            if node.has_tag_name("joint") {
                let name = node.attribute("name").unwrap_or("");
                let axis_str = node.attribute("axis").unwrap_or("0 0 1");
                let axis: Vec<f32> = axis_str
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                if axis.len() == 3 {
                    joints.push((name.to_string(), axis[0], axis[1], axis[2]));
                }
            }
            // Measure links from capsule "fromto" attributes
            if node.has_tag_name("geom") && node.attribute("type") == Some("capsule") {
                if let Some(fromto) = node.attribute("fromto") {
                    let v: Vec<f32> = fromto
                        .split_whitespace()
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    if v.len() == 6 {
                        let dx = v[3] - v[0];
                        let dy = v[4] - v[1];
                        let dz = v[5] - v[2];
                        let len = (dx * dx + dy * dy + dz * dz).sqrt();
                        links.push(len);
                    }
                }
            }
            // Extract shoulder height from the "shoulder" body position
            if node.has_tag_name("body") && node.attribute("name") == Some("shoulder") {
                if let Some(pos) = node.attribute("pos") {
                    let v: Vec<f32> = pos
                        .split_whitespace()
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    if v.len() == 3 {
                        base_z = v[2]; // Z is up in MuJoCo
                    }
                }
            }
        }
    }

    let mut code = String::new();
    code.push_str("// Auto-generated from ironarm.xml — do not edit.\n\n");

    code.push_str(&format!(
        "pub const LINK_LENGTHS: &[f32] = &[{}];\n",
        links
            .iter()
            .map(|l| format!("{:.6}", l))
            .collect::<Vec<_>>()
            .join(", ")
    ));

    code.push_str(&format!("pub const BASE_Z: f32 = {base_z:.2}f32;\n"));

    code.push_str("pub const JOINT_NAMES: &[&str] = &[");
    for (i, (name, _, _, _)) in joints.iter().enumerate() {
        if i > 0 {
            code.push_str(", ");
        }
        code.push_str(&format!("\"{name}\""));
    }
    code.push_str("];\n");

    code.push_str("pub const JOINT_AXES: &[(f32, f32, f32)] = &[");
    for (i, (_, x, y, z)) in joints.iter().enumerate() {
        if i > 0 {
            code.push_str(", ");
        }
        code.push_str(&format!("({x:.1}, {y:.1}, {z:.1})"));
    }
    code.push_str("];\n");

    fs::write(&dest, code).expect("failed to write model_params.rs");
    println!("cargo:rerun-if-changed=ironarm.xml");
}
