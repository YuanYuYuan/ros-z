#!/usr/bin/env nu

# ROS 2 package test runner for rmw-zenoh-rs
# Single source of truth for both local development and CI
#
# Usage:
#   nu scripts/test-ros-packages.nu                              # Run rcl and rclcpp tests (default)
#   nu scripts/test-ros-packages.nu --include-filter "test_pub"  # Filter tests
#   nu scripts/test-ros-packages.nu --build-only                 # Only build
#   nu scripts/test-ros-packages.nu --test-only                  # Only test (skip build)
#   nu scripts/test-ros-packages.nu --rmw-impl rmw_zenoh_cpp     # Test with rmw_zenoh_cpp
#   nu scripts/test-ros-packages.nu --packages rcl               # Test only rcl
#   nu scripts/test-ros-packages.nu --packages "rcl,rclcpp,rcl_action" # Test specific packages
#   nu scripts/test-ros-packages.nu --rmw-path /path/to/rmw-zenoh-rs  # Custom rmw path
#   nu scripts/test-ros-packages.nu --log                             # Save output to _tmp/
#
# Workspace convention (user manages packages in src/):
#   ws-dir/
#   ├── src/              # ROS 2 packages (e.g., rcl/)
#   ├── build/
#   ├── install/
#   └── log/
#
# Note: rmw-zenoh-rs is passed via --base-paths, not symlinked into workspace

def main [
    --ws-dir: string = ""                  # Workspace directory (default: ../ws)
    --rmw-path: string = ""                # Path to rmw-zenoh-rs (default: auto-detect from script)
    --rmw-impl: string = "rmw_zenoh_rs"    # RMW implementation to test (rmw_zenoh_rs or rmw_zenoh_cpp)
    --build-only                           # Only build, skip tests
    --test-only                            # Only test, skip build
    --include-filter: string = ""          # CTest include regex filter (e.g., "test_publisher|test_node")
    --exclude-filter: string = ""          # CTest exclude regex filter (e.g., "test_count_matched|test_events")
    --packages: string = ""                # ROS 2 packages to test (comma-separated, e.g., "rcl,rclcpp"; default: "rcl,rclcpp")
    --src-paths: list<string> = []         # Additional source paths to scan
    --verbose                              # Verbose test output
    --clean                                # Clean build before building
    --no-daemon                            # Skip spawning rmw_zenohd (if already running)
    --timeout: int = 0                     # Test timeout in seconds (0 = no timeout)
    --log                                  # Save output to _tmp/test-<timestamp>.log
] {
    # Validate conflicting flags
    if $build_only and $test_only {
        print "Error: Cannot specify both --build-only and --test-only"
        exit 1
    }

    # Resolve paths relative to script location (ros-z root)
    let ros_z_root = ($env.FILE_PWD | path dirname)

    let ws = if $ws_dir == "" {
        $ros_z_root | path join "../ws" | path expand
    } else {
        $ws_dir | path expand
    }

    # Resolve rmw-zenoh-rs path (absolute path passed to colcon --paths)
    let rmw_zenoh_rs = if $rmw_path == "" {
        $ros_z_root | path join "rmw-zenoh-rs" | path expand
    } else {
        $rmw_path | path expand
    }

    # Validate rmw-zenoh-rs exists
    if not ($rmw_zenoh_rs | path exists) {
        print $"Error: rmw-zenoh-rs not found: ($rmw_zenoh_rs)"
        exit 1
    }

    # Validate workspace src directory exists
    let ws_src = $ws | path join "src"
    if not ($ws_src | path exists) {
        print $"Error: Workspace src directory not found: ($ws_src)"
        print "Create the workspace and add your ROS 2 packages to src/"
        exit 1
    }

    # Parse packages parameter (comma-separated string or default)
    let pkg_list = if $packages == "" {
        ["rcl", "rclcpp"]
    } else {
        $packages | split row ","
    }

    # Setup logging if --log is specified
    let log_file = if $log {
        let tmp_dir = $ros_z_root | path join "_tmp"
        mkdir $tmp_dir
        let timestamp = date now | format date "%Y%m%d-%H%M%S"
        let filter_suffix = if $include_filter != "" {
            $"-($include_filter | str replace -a '[^a-zA-Z0-9_]' '_')"
        } else {
            ""
        }
        let log_path = $tmp_dir | path join $"test-($timestamp)($filter_suffix).log"
        print $"Logging to: ($log_path)"
        $log_path
    } else {
        null
    }

    print $"=== ROS 2 Package Test Runner for rmw-zenoh-rs ==="
    print $"  Workspace: ($ws)"
    print $"  Packages: ($pkg_list | str join ', ')"
    print ""

    # Determine source paths to scan
    # Use absolute path for rmw-zenoh-rs (no symlink needed)
    # This preserves Cargo workspace resolution
    let scan_paths = if ($src_paths | is-empty) {
        mut paths = [$rmw_zenoh_rs]
        for pkg in $pkg_list {
            let pkg_path = $ws_src | path join $pkg
            if ($pkg_path | path exists) {
                $paths = ($paths | append ($pkg_path | path expand))
            }
        }
        $paths
    } else {
        # Prepend rmw-zenoh-rs to user-specified paths
        [$rmw_zenoh_rs] | append $src_paths
    }

    cd $ws

    # Clean if requested (skip if test-only)
    if $clean and not $test_only {
        print "Cleaning build directories..."
        rm -rf build install log
    }

    # Build using --paths with absolute paths (no symlinks needed)
    # This preserves Cargo workspace resolution for rmw-zenoh-rs
    if not $test_only {
        print "Building rmw_zenoh_rs and dependencies..."
        colcon build --base-paths ...($scan_paths) --packages-up-to rmw_zenoh_rs --symlink-install

        print $"Building test packages: ($pkg_list | str join ', ')..."
        colcon build --base-paths ...($scan_paths) --packages-select ...($pkg_list) --symlink-install

        if $build_only {
            print "Build complete (--build-only specified, skipping tests)"
            return
        }
    } else {
        print "Skipping build (--test-only specified)"

        # Validate that build directory exists
        let build_dir = $ws | path join "build"
        let install_dir = $ws | path join "install"
        if not ($build_dir | path exists) or not ($install_dir | path exists) {
            print "Error: Build directory not found. Run without --test-only to build first."
            exit 1
        }
    }

    # Spawn rmw_zenohd in background
    let daemon_job = if not $no_daemon {
        # Kill any existing zenohd processes first
        print "Killing any existing zenohd processes..."
        pkill -9 zenohd | ignore
        sleep 500ms

        print "Spawning rmw_zenohd daemon in background..."

        # Source setup and spawn daemon
        let job_id = job spawn {
            bash -c "source install/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"
        }

        print $"  Daemon job ID: ($job_id)"

        # Wait for daemon to be ready
        print "  Waiting for daemon to start..."
        sleep 2sec

        $job_id
    } else {
        null
    }

    # Prepare test command
    let packages_arg = $pkg_list | str join " "
    let paths_arg = $scan_paths | str join " "

    let verbose_arg = if $verbose { " --event-handlers console_direct+" } else { "" }
    let filter_arg = if $include_filter != "" {
        print $"Include filter: ($include_filter)"
        $" --ctest-args -R '($include_filter)'"
    } else {
        ""
    }
    let exclude_arg = if $exclude_filter != "" {
        print $"Exclude filter: ($exclude_filter)"
        $" -E '($exclude_filter)'"
    } else {
        ""
    }

    # Set RUST_LOG to show only warnings and errors from most modules, unless verbose mode is enabled
    let rust_log = if $verbose {
        "debug"
    } else {
        "warn,ros_z=warn,rmw_zenoh_rs=warn,zenoh=warn"
    }

    let test_cmd = $"source ($ws)/install/setup.bash && RUST_LOG=($rust_log) RMW_IMPLEMENTATION=($rmw_impl) colcon test --base-paths ($paths_arg) --packages-select ($packages_arg) --return-code-on-test-failure($verbose_arg)($filter_arg)($exclude_arg)"

    print $"Testing packages: ($packages_arg) with RMW: ($rmw_impl)"
    print ""

    # Run tests with optional timeout and logging
    let test_exit_code = if $log_file != null {
        # Use tee to capture output to log file
        let tee_cmd = if $timeout > 0 {
            print $"Using timeout: ($timeout)s"
            $"timeout ($timeout)s bash -c '($test_cmd)' 2>&1 | tee ($log_file)"
        } else {
            $"bash -c '($test_cmd)' 2>&1 | tee ($log_file)"
        }
        bash -c $tee_cmd
        $env.LAST_EXIT_CODE
    } else if $timeout > 0 {
        print $"Using timeout: ($timeout)s"
        bash -c $"timeout ($timeout)s bash -c '($test_cmd)'"
        $env.LAST_EXIT_CODE
    } else {
        bash -c $test_cmd
        $env.LAST_EXIT_CODE
    }

    # Kill daemon if we spawned it
    if $daemon_job != null {
        print ""
        print $"Stopping rmw_zenohd daemon \(job ($daemon_job)\)..."
        job kill $daemon_job
    }

    # Show results
    print ""
    if $test_exit_code == 0 {
        print "=== All tests passed ==="
    } else {
        print $"=== Tests failed with exit code ($test_exit_code) ==="
        print $"View detailed results: colcon test-result --test-result-base ($ws)/build"
    }

    if $log_file != null {
        print $"Log saved to: ($log_file)"
    }

    exit $test_exit_code
}
