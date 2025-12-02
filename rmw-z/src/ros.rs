#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, unused)]

use std::ffi::c_void;

// RMW return codes
pub const RMW_RET_OK: u32 = 0;
pub const RMW_RET_ERROR: u32 = 1;
pub const RMW_RET_TIMEOUT: u32 = 2;
pub const RMW_RET_BAD_ALLOC: u32 = 10;
pub const RMW_RET_INVALID_ARGUMENT: u32 = 11;
pub const RMW_RET_UNSUPPORTED: u32 = 3;
pub const RMW_RET_ALREADY_INIT: u32 = 12;

// RCL return codes
pub const RCL_RET_OK: i32 = 0;
pub const RCL_RET_ERROR: i32 = 1;
pub const RCL_RET_BAD_ALLOC: i32 = 10;
pub const RCL_RET_INVALID_ARGUMENT: i32 = 11;
pub const RCL_RET_UNSUPPORTED: i32 = 3;
pub const RCL_RET_NODE_INVALID: i32 = 100;
pub const RCL_RET_NODE_INVALID_NAME: i32 = 101;
pub const RCL_RET_NODE_INVALID_NAMESPACE: i32 = 102;
pub const RCL_RET_NODE_NAME_NON_EXISTENT: i32 = 103;
pub const RCL_RET_PUBLISHER_INVALID: i32 = 200;
pub const RCL_RET_SUBSCRIPTION_INVALID: i32 = 201;
pub const RCL_RET_SUBSCRIPTION_TAKE_FAILED: i32 = 202;
pub const RCL_RET_CLIENT_INVALID: i32 = 300;
pub const RCL_RET_CLIENT_TAKE_FAILED: i32 = 301;
pub const RCL_RET_SERVICE_INVALID: i32 = 400;
pub const RCL_RET_SERVICE_TAKE_FAILED: i32 = 401;
pub const RCL_RET_ALREADY_INIT: i32 = 500;
pub const RCL_RET_NOT_INIT: i32 = 501;

// QoS policies
pub const RMW_QOS_POLICY_HISTORY_KEEP_LAST: u32 = 1;
pub const RMW_QOS_POLICY_HISTORY_KEEP_ALL: u32 = 2;
pub const RMW_QOS_POLICY_RELIABILITY_RELIABLE: u32 = 1;
pub const RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT: u32 = 2;
pub const RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: u32 = 1;
pub const RMW_QOS_POLICY_DURABILITY_VOLATILE: u32 = 2;

// Basic types
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_time_t {
    pub sec: i64,
    pub nsec: u64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_qos_profile_t {
    pub history: u32,
    pub depth: usize,
    pub reliability: u32,
    pub durability: u32,
    pub liveliness: u32,
    pub liveliness_lease_duration: rmw_time_t,
    pub deadline: rmw_time_t,
    pub lifespan: rmw_time_t,
    pub avoid_ros_namespace_conventions: bool,
}

impl Default for rmw_qos_profile_t {
    fn default() -> Self {
        Self {
            history: RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth: 10,
            reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability: RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness: 0,
            liveliness_lease_duration: rmw_time_t { sec: 0, nsec: 0 },
            deadline: rmw_time_t { sec: 0, nsec: 0 },
            lifespan: rmw_time_t { sec: 0, nsec: 0 },
            avoid_ros_namespace_conventions: false,
        }
    }
}

// RMW structs
#[repr(C)]
#[derive(Debug)]
pub struct rmw_context_t {
    pub instance_id: u64,
    pub impl_: *mut rmw_context_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_node_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
    pub name: *const ::std::os::raw::c_char,
    pub namespace_: *const ::std::os::raw::c_char,
    pub context: *mut rmw_context_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_publisher_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
    pub topic_name: *const ::std::os::raw::c_char,
    pub options: *const rmw_publisher_options_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_subscription_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
    pub topic_name: *const ::std::os::raw::c_char,
    pub options: *const rmw_subscription_options_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_client_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
    pub service_name: *const ::std::os::raw::c_char,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_service_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
    pub service_name: *const ::std::os::raw::c_char,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_wait_set_t {
    pub impl_: *mut rmw_wait_set_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_guard_condition_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
    pub context: *mut rmw_context_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_event_t {
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub data: *mut ::std::os::raw::c_void,
}

// Opaque impl types
#[repr(C)]
pub struct rmw_context_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_node_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_publisher_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_subscription_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_client_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_service_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_wait_set_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_guard_condition_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct rmw_event_impl_t {
    _private: [u8; 0],
}

// Options structs
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_publisher_options_t {
    pub rmw_specific_publisher_payload: *mut c_void,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_subscription_options_t {
    pub rmw_specific_subscription_payload: *mut c_void,
    pub ignore_local_publications: bool,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_client_options_t {
    pub qos: rmw_qos_profile_t,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_service_options_t {
    pub qos: rmw_qos_profile_t,
}

// Allocation structs
#[repr(C)]
#[derive(Debug)]
pub struct rmw_publisher_allocation_t {
    pub impl_: *mut c_void,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_subscription_allocation_t {
    pub impl_: *mut c_void,
}

// Wait set collections
#[repr(C)]
#[derive(Debug)]
pub struct rmw_subscriptions_t {
    pub subscriber_count: usize,
    pub subscribers: *mut *mut rmw_subscription_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_guard_conditions_t {
    pub guard_condition_count: usize,
    pub guard_conditions: *mut *mut rmw_guard_condition_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_services_t {
    pub service_count: usize,
    pub services: *mut *mut rmw_service_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_clients_t {
    pub client_count: usize,
    pub clients: *mut *mut rmw_client_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_events_t {
    pub event_count: usize,
    pub events: *mut *mut rmw_event_t,
}

// Request/Response IDs
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_request_id_t {
    pub writer_guid: [u8; 16],
    pub sequence_number: i64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_service_info_t {
    pub request_id: rmw_request_id_t,
    pub source_timestamp: rmw_time_t,
    pub received_timestamp: rmw_time_t,
}

// Message info
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rmw_message_info_t {
    pub source_timestamp: rmw_time_t,
    pub received_timestamp: rmw_time_t,
    pub publication_sequence_number: u64,
    pub reception_sequence_number: u64,
    pub publisher_gid: [u8; 24],
    pub from_intra_process: bool,
}

// Serialized message
#[repr(C)]
#[derive(Debug)]
pub struct rcl_serialized_message_t {
    pub buffer: *mut u8,
    pub buffer_length: usize,
    pub buffer_capacity: usize,
    pub allocator: rcl_allocator_t,
}

// String array
#[repr(C)]
#[derive(Debug)]
pub struct rcutils_string_array_t {
    pub size: usize,
    pub data: *mut *mut ::std::os::raw::c_char,
    pub allocator: rcl_allocator_t,
}

// Names and types
#[repr(C)]
#[derive(Debug)]
pub struct rmw_names_and_types_t {
    pub names: rcutils_string_array_t,
    pub types: *mut rcutils_string_array_t,
}

// Allocator
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rcl_allocator_t {
    pub allocate: Option<unsafe extern "C" fn(size: usize, state: *mut c_void) -> *mut c_void>,
    pub deallocate: Option<unsafe extern "C" fn(pointer: *mut c_void, state: *mut c_void)>,
    pub realloc: Option<unsafe extern "C" fn(pointer: *mut c_void, size: usize, state: *mut c_void) -> *mut c_void>,
    pub zero_allocate: Option<unsafe extern "C" fn(number_of_elements: usize, size_of_element: usize, state: *mut c_void) -> *mut c_void>,
    pub state: *mut c_void,
}

// ROS IDL types (from rcl_z)
pub use rcl_z::ros::rosidl_message_type_support_t;
pub use rcl_z::ros::rosidl_service_type_support_t;

// Additional RMW types needed - these will be defined locally since they're not available in rcl_z

// RCL types (needed for compatibility)
#[repr(C)]
#[derive(Debug)]
pub struct rcl_context_t {
    pub impl_: *mut rcl_context_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_context_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_t {
    pub context: *mut rcl_context_t,
    pub impl_: *mut rcl_node_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_publisher_t {
    pub impl_: *mut rcl_publisher_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_publisher_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_subscription_t {
    pub impl_: *mut rcl_subscription_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_subscription_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_client_t {
    pub impl_: *mut rcl_client_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_client_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_service_t {
    pub impl_: *mut rcl_service_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_service_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_wait_set_t {
    pub impl_: *mut rcl_wait_set_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_wait_set_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_guard_condition_t {
    pub impl_: *mut rcl_guard_condition_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_guard_condition_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_event_t {
    pub impl_: *mut rcl_event_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_event_impl_t {
    _private: [u8; 0],
}

// RCL options
#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_options_t {
    pub domain_id: usize,
    pub allocator: rcl_allocator_t,
    pub use_global_arguments: bool,
    pub arguments: rcl_arguments_t,
    pub enable_rosout: bool,
    pub rosout_qos: rmw_qos_profile_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_publisher_options_t {
    pub qos: rmw_qos_profile_t,
    pub allocator: rcl_allocator_t,
    pub disable_loaned_message: bool,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_subscription_options_t {
    pub qos: rmw_qos_profile_t,
    pub allocator: rcl_allocator_t,
    pub disable_loaned_message: bool,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_client_options_t {
    pub qos: rmw_qos_profile_t,
    pub allocator: rcl_allocator_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_service_options_t {
    pub qos: rmw_qos_profile_t,
    pub allocator: rcl_allocator_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_arguments_t {
    pub impl_: *mut rcl_arguments_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_arguments_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_init_options_t {
    pub impl_: *mut rcl_init_options_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_init_options_impl_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_timer_t {
    pub impl_: *mut rcl_timer_impl_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rcl_timer_impl_t {
    _private: [u8; 0],
}

// Init options
#[repr(C)]
#[derive(Debug)]
pub struct rmw_init_options_t {
    pub instance_id: u64,
    pub implementation_identifier: *const ::std::os::raw::c_char,
    pub domain_id: usize,
    pub security_options: *mut c_void,
    pub localhost_only: u8,
    pub allocator: rcl_allocator_t,
    pub impl_: *mut c_void,
}

// Return types
pub type rmw_ret_t = i32;
pub type rcl_ret_t = i32;

// Additional types needed for RMW implementation
#[repr(C)]
#[derive(Debug)]
pub struct rmw_network_flow_endpoint_array_t {
    pub size: usize,
    pub data: *mut rmw_network_flow_endpoint_t,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_network_flow_endpoint_t {
    pub transport_protocol: *mut ::std::os::raw::c_char,
    pub internet_address: *mut ::std::os::raw::c_char,
    pub transport_port: u32,
    pub flow_label: *mut ::std::os::raw::c_char,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_subscription_content_filter_options_t {
    pub filter_expression: *mut ::std::os::raw::c_char,
    pub expression_parameters: *mut rcutils_string_array_t,
}

// Additional missing types - defined locally since they're not available in rcl_z
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum rmw_qos_compatibility_type_t {
    RMW_QOS_COMPATIBILITY_OK = 0,
    RMW_QOS_COMPATIBILITY_WARNING = 1,
    RMW_QOS_COMPATIBILITY_ERROR = 2,
}

#[repr(C)]
#[derive(Debug)]
pub struct rmw_serialization_support_t {
    pub serialize_message: Option<unsafe extern "C" fn(
        *const ::std::os::raw::c_void,
        *mut rcl_serialized_message_t,
    ) -> rmw_ret_t>,
    pub deserialize_message: Option<unsafe extern "C" fn(
        *const rcl_serialized_message_t,
        *mut ::std::os::raw::c_void,
    ) -> rmw_ret_t>,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rosidl_message_bounds_t {
    pub n_members: usize,
    pub members: *const rosidl_message_member_t,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rosidl_message_member_t {
    pub name: *const ::std::os::raw::c_char,
    pub type_id: u32,
    pub is_array: bool,
    pub array_size: usize,
    pub string_upper_bound: usize,
    pub is_primitive: bool,
}

pub type rmw_subscription_new_message_callback_t = Option<unsafe extern "C" fn(
    *const ::std::os::raw::c_void,
    usize,
) -> ::std::os::raw::c_int>;

pub type rmw_service_new_request_callback_t = Option<unsafe extern "C" fn(
    *const ::std::os::raw::c_void,
    usize,
) -> ::std::os::raw::c_int>;

pub type rmw_client_new_response_callback_t = Option<unsafe extern "C" fn(
    *const ::std::os::raw::c_void,
    usize,
) -> ::std::os::raw::c_int>;

#[repr(C)]
#[derive(Debug)]
pub struct rcldynamic_message_t {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum rmw_feature_t {
    RMW_FEATURE_MESSAGE_INFO = 0,
    RMW_FEATURE_PUBLISHER_LOAN = 1,
    RMW_FEATURE_SUBSCRIPTION_LOAN = 2,
    RMW_FEATURE_SERVICE_LOAN = 3,
    RMW_FEATURE_CLIENT_LOAN = 4,
    RMW_FEATURE_EVENT_MESSAGE = 5,
    RMW_FEATURE_EVENT_SERVICE = 6,
    RMW_FEATURE_EVENT_CLIENT = 7,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum rmw_log_severity_t {
    RMW_LOG_SEVERITY_DEBUG = 0,
    RMW_LOG_SEVERITY_INFO = 1,
    RMW_LOG_SEVERITY_WARN = 2,
    RMW_LOG_SEVERITY_ERROR = 3,
    RMW_LOG_SEVERITY_FATAL = 4,
}