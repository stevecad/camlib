//! Every integration test, as one test binary (#194).
//!
//! The engine is generic over its cancel signal, so each crate that calls it
//! compiles the engine again for its own signal types. As 55 separate test
//! binaries that was about 10,000 CPU-seconds of building (37 of the Windows
//! CI job's 48 minutes); as one binary it happens once.
//!
//! Each file in this directory is a module here. Add a new file to the list
//! below: `every_test_file_is_a_module` fails if one is missing, so a test
//! file can never be silently left out of the run.
//!
//! Run one file's tests with `cargo test --test integration <file>::`.

macro_rules! modules {
    ($($name:ident),* $(,)?) => {
        $(mod $name;)*
        const MODULES: &[&str] = &[$(stringify!($name)),*];
    };
}

modules! {
    adaptive,
    adaptive_connected,
    adaptive_front_complete,
    assembly_reach,
    chamfer,
    contour,
    contracts,
    curved_surfaces,
    deburr,
    dexel,
    dexel_stream,
    dynamics,
    engagement,
    facing,
    finishing_material,
    finishing_stock,
    geometry,
    helical_entry,
    holes,
    machine,
    material,
    material_budgets,
    material_resume,
    material_surface,
    nesting,
    parallel,
    patterns,
    planar,
    pocket,
    pocket_top,
    post_rounding,
    preparation,
    probe,
    progress,
    protocol,
    ramp_entry,
    ramp_entry_front,
    recessed_finishing,
    regeneration,
    roughing,
    rust_carvera_post,
    rust_haas_post,
    rust_post,
    shared_geometry,
    silhouette,
    slot,
    stock,
    stock_contact,
    stock_mesh,
    stream,
    supplied,
    surface_validation,
    tapping,
    thermal,
    thermal_common_line,
    thermal_mark,
    thread_mill,
    trace,
    undercut,
    verification,
    waterline,
    waterline_flat_shadow,
    waterline_reach,
    waterline_support,
}

#[test]
fn every_test_file_is_a_module() {
    let directory = concat!(env!("CARGO_MANIFEST_DIR"), "/tests");
    let mut files: Vec<String> = std::fs::read_dir(directory)
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .filter_map(|name| name.strip_suffix(".rs").map(str::to_string))
        .filter(|name| name != "main")
        .collect();
    files.sort();
    let mut modules: Vec<String> = MODULES.iter().map(|m| m.to_string()).collect();
    modules.sort();
    assert_eq!(
        files, modules,
        "tests/*.rs and the modules! list in tests/main.rs differ"
    );
}
