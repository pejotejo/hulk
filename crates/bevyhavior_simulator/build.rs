use color_eyre::eyre::{Result, WrapErr};

use code_generation::{ExecutionMode, generate, write_to_file::WriteToFile};
use source_analyzer::{
    cyclers::{CyclerKind, Cyclers},
    manifest::{CyclerManifest, FrameworkManifest},
    pretty::to_string_pretty,
    structs::Structs,
};

fn main() -> Result<()> {
    let manifest = FrameworkManifest {
        cyclers: vec![CyclerManifest {
            name: "WorldState",
            kind: CyclerKind::RealTime,
            instances: vec![""],
            setup_nodes: vec!["crate::fake_data"],
            nodes: vec!["world_state::behavior::node"],
            execution_time_warning_threshold: None,
        }],
    };
    let root = "../../crates/";

    let cyclers = Cyclers::try_from_manifest(manifest, root)?;
    for path in cyclers.watch_paths() {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    println!();
    println!("{}", to_string_pretty(&cyclers)?);

    let structs = Structs::try_from_cyclers(&cyclers)?;
    generate(&cyclers, &structs, ExecutionMode::Run)
        .write_to_file("generated_code.rs")
        .wrap_err("failed to write generated code to file")
}
