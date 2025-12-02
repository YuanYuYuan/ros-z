fn main() {
    // rmw-z implements RMW directly in Rust using Zenoh
    // No C++ compilation needed
    println!("cargo:rerun-if-changed=src/lib.rs");
}
