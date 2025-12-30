//! Demo nodes module
//!
//! This module re-exports the functions from the example files,
//! allowing tests to import and use the exact same code that users run.
pub mod add_two_ints_client;
pub mod add_two_ints_server;
pub mod fibonacci_action_client;
pub mod fibonacci_action_server;
pub mod listener;
pub mod talker;

// Re-export the main functions for easy access
pub use add_two_ints_client::run_add_two_ints_client;
pub use add_two_ints_server::run_add_two_ints_server;
#[cfg(all(feature = "external_msgs", has_action_tutorials_interfaces))]
pub use fibonacci_action_client::run_fibonacci_action_client;
#[cfg(all(feature = "external_msgs", has_action_tutorials_interfaces))]
pub use fibonacci_action_server::run_fibonacci_action_server;
pub use listener::run_listener;
pub use talker::run_talker;
