extern crate bindgen;

use std::{env, path::PathBuf};

const INCLUDE_PACKAGES: &[&str] = &[
    "rmw",
    "rcutils",
    "rosidl_runtime_c",
    "rosidl_typesupport_interface",
    "fastcdr",
];

fn main() {
    // Get AMENT_PREFIX_PATH from the environment
    let ament_prefix =
        env::var("AMENT_PREFIX_PATH").unwrap_or_else(|_| "/opt/ros/humble".to_string());

    let bindgen_out_path = PathBuf::from("src");

    // Collect all include paths for bindgen
    let mut include_args = Vec::new();
    for pkg in INCLUDE_PACKAGES {
        for prefix in ament_prefix.split(':') {
            let pkg_path = PathBuf::from(prefix).join(format!("include/{}", pkg));
            if pkg_path.exists() {
                include_args.push(format!("-I{}", pkg_path.display()));
                break;
            }
        }
    }

    // Generate bindings
    let bindings = bindgen::Builder::default()
        .header("binding.hpp")
        .clang_args(include_args.clone())
        // Allow utility functions
        .allowlist_function("rcutils_.*")
        .allowlist_function("rmw_get_zero_initialized_.*")
        .allowlist_function("rmw_names_and_types_.*")
        .allowlist_function("rmw_check_zero_rmw_string_array")
        .allowlist_function("rmw_security_options_.*")
        .allowlist_function("rmw_discovery_options_.*")
        .allowlist_function("rmw_validate_.*")
        .allowlist_function("rmw_event_fini")
        .allowlist_function("rmw_topic_endpoint_info_.*")
        .allowlist_function("rmw_service_endpoint_info_.*")
        // Allow types and constants
        .allowlist_type("rmw_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_type("rosidl_.*")
        .allowlist_var("RMW_.*")
        // Blocklist problematic functions that require unavailable headers
        .blocklist_function("rmw_take_dynamic_message")
        .blocklist_function("rmw_take_dynamic_message_with_info")
        .blocklist_function("rmw_serialization_support_init")
        .blocklist_type("rosidl_dynamic_typesupport_serialization_support_t")
        .blocklist_type("rosidl_dynamic_typesupport_serialization_support_impl_t")
        .derive_default(true)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(bindgen_out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    // Build CXX bridge for type support serialization
    let include_pkgs = [
        "rmw",
        "fastcdr",
        "rcutils",
        "rosidl_runtime_c",
        "rosidl_typesupport_interface",
        "rosidl_typesupport_fastrtps_c",
        "rosidl_typesupport_fastrtps_cpp",
    ];
    let mut include_dirs = Vec::new();
    for pkg in include_pkgs {
        for prefix in ament_prefix.split(':') {
            let pkg_path = PathBuf::from(prefix).join(format!("include/{}", pkg));
            if pkg_path.exists() {
                include_dirs.push(pkg_path.display().to_string());
                break;
            }
        }
    }

    cxx_build::bridge("src/type_support.rs")
        .file("src/serde_bridge.cc")
        .include("include")
        .includes(include_dirs)
        .std("c++17")
        .compile("serde_bridge");

    // Link libraries
    let lib_path = format!("{}/lib", ament_prefix.split(':').next().unwrap());
    println!("cargo:rustc-link-search=native=/usr/local/lib");
    println!("cargo:rustc-link-search=native={}", lib_path);
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");
    println!("cargo:rustc-link-lib=dylib=fastcdr");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_fastrtps_cpp");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/type_support.rs");
    println!("cargo:rerun-if-changed=src/serde_bridge.cc");
    println!("cargo:rerun-if-changed=include/serde_bridge.h");
    println!("cargo:rerun-if-changed=binding.hpp");
}
