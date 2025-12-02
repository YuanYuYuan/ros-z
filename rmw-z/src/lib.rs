#![allow(clippy::enum_variant_names)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::explicit_auto_deref)]
#![allow(clippy::collapsible_if)]
#![allow(clippy::cast_slice_from_raw_parts)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(unpredictable_function_pointer_comparisons)]

pub mod common;
pub mod context;
pub mod guard_condition;
pub mod node;
pub mod pubsub;
pub mod qos;
pub mod ros;
pub mod service;
#[macro_use]
pub mod traits;
pub mod wait_set;

/// Newtype wrapper for a C void. Only useful as a `*c_void`
#[allow(non_camel_case_types)]
#[repr(transparent)]
pub struct c_void(pub ::std::os::raw::c_void);

/// # Safety
///
/// We assert that the namespace and type ID refer to a C++
/// type which is equivalent to this Rust type.
unsafe impl cxx::ExternType for c_void {
    type Id = cxx::type_id!(c_void);
    type Kind = cxx::kind::Trivial;
}

// RMW implementation identifier
pub const RMW_ZENOH_IDENTIFIER: &str = "rmw_zenoh_cpp";

// Serialization format
pub const RMW_ZENOH_SERIALIZATION_FORMAT: &str = "cdr";

// Note: rmw-z implements the RMW layer directly in Rust using Zenoh
// No C++ bridge needed since we're not interfacing with existing RMW implementations

// Remove the cxx extern block since we're implementing RMW directly

use rcl_z::ros::{
    rmw_event_callback_t, rmw_event_type_t, rmw_gid_t, rmw_topic_endpoint_info_array_t,
};

use crate::{pubsub::PublisherImpl, ros::*, traits::*};

// Implement the actual RMW functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_implementation_identifier() -> *const std::os::raw::c_char {
    RMW_ZENOH_IDENTIFIER.as_ptr() as *const std::os::raw::c_char
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialization_format() -> *const std::os::raw::c_char {
    RMW_ZENOH_SERIALIZATION_FORMAT.as_ptr() as *const std::os::raw::c_char
}

// Publishers
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_publisher(
    node: *const rmw_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const std::os::raw::c_char,
    qos_profile: *const rmw_qos_profile_t,
    publisher_options: *const rmw_publisher_options_t,
) -> *mut rmw_publisher_t {
    if node.is_null()
        || type_support.is_null()
        || topic_name.is_null()
        || qos_profile.is_null()
        || publisher_options.is_null()
    {
        return std::ptr::null_mut();
    }

    let node_impl = match unsafe { node.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let topic_str = unsafe { std::ffi::CStr::from_ptr(topic_name) }
        .to_str()
        .unwrap_or("");
    let ts = match unsafe { rcl_z::type_support::MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return std::ptr::null_mut(),
    };

    let zpub_builder = node_impl
        .inner
        .create_pub::<rcl_z::msg::RosMessage>(topic_str);
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { *qos_profile });
    let zpub_builder = zpub_builder.with_qos(qos);
    let zpub = match zpub_builder.build() {
        Ok(zpub) => zpub,
        Err(_) => return std::ptr::null_mut(),
    };

    let topic_cstr = match std::ffi::CString::new(topic_str) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let publisher_impl = PublisherImpl {
        inner: zpub,
        ts,
        topic: topic_cstr,
        options: unsafe { *publisher_options },
        qos: unsafe { *qos_profile },
    };

    let publisher = Box::new(rmw_publisher_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        topic_name: topic_name as *const _,
        options: publisher_options as *const _,
    });

    let publisher_ptr = Box::into_raw(publisher);
    unsafe {
        (*publisher_ptr).data = Box::into_raw(Box::new(publisher_impl)) as *mut _;
    }

    publisher_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_publisher(
    node: *mut rmw_node_t,
    publisher: *mut rmw_publisher_t,
) -> rmw_ret_t {
    if node.is_null() || publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(publisher) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publish(
    publisher: *const rmw_publisher_t,
    ros_message: *const c_void,
    allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    if publisher.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match unsafe { publisher.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match publisher_impl.publish(ros_message) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to publish message: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

// Subscriptions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_subscription(
    node: *const rmw_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const std::os::raw::c_char,
    qos_policies: *const rmw_qos_profile_t,
    subscription_options: *const rmw_subscription_options_t,
) -> *mut rmw_subscription_t {
    if node.is_null()
        || type_support.is_null()
        || topic_name.is_null()
        || qos_policies.is_null()
        || subscription_options.is_null()
    {
        return std::ptr::null_mut();
    }

    let node_impl = match unsafe { node.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let topic_str = unsafe { std::ffi::CStr::from_ptr(topic_name) }
        .to_str()
        .unwrap_or("");
    let ts = match unsafe { rcl_z::type_support::MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return std::ptr::null_mut(),
    };

    let zsub_builder = node_impl
        .inner
        .create_sub::<rcl_z::msg::RosMessage>(topic_str);
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { *qos_policies });
    let zsub_builder = zsub_builder.with_qos(qos);
    let zsub = match zsub_builder.build() {
        Ok(zsub) => zsub,
        Err(_) => return std::ptr::null_mut(),
    };

    let topic_cstr = match std::ffi::CString::new(topic_str) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let subscription_impl = crate::pubsub::SubscriptionImpl {
        inner: zsub,
        ts,
        topic: topic_cstr,
        options: unsafe { *subscription_options },
        qos: unsafe { *qos_policies },
    };

    let subscription = Box::new(rmw_subscription_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        topic_name: topic_name as *const _,
        options: subscription_options as *const _,
    });

    let subscription_ptr = Box::into_raw(subscription);
    unsafe {
        (*subscription_ptr).data = Box::into_raw(Box::new(subscription_impl)) as *mut _;
    }

    subscription_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_subscription(
    node: *mut rmw_node_t,
    subscription: *mut rmw_subscription_t,
) -> rmw_ret_t {
    if node.is_null() || subscription.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(subscription) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take(
    subscription: *const rmw_subscription_t,
    ros_message: *mut c_void,
    taken: *mut bool,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || ros_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match unsafe { subscription.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take(ros_message, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

// Services
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_client(
    node: *const rmw_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const std::os::raw::c_char,
    qos_policies: *const rmw_qos_profile_t,
) -> *mut rmw_client_t {
    if node.is_null() || type_support.is_null() || service_name.is_null() || qos_policies.is_null()
    {
        return std::ptr::null_mut();
    }

    let node_impl = match unsafe { node.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_str = unsafe { std::ffi::CStr::from_ptr(service_name) }
        .to_str()
        .unwrap_or("");

    // Create client using ros-z
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { *qos_policies });
    let zclient = match node_impl
        .inner
        .create_client::<rcl_z::msg::RosService>(service_str)
        .with_qos(qos)
        .build()
    {
        Ok(client) => client,
        Err(e) => {
            tracing::error!("Failed to create client: {}", e);
            return std::ptr::null_mut();
        }
    };

    let client_impl = crate::service::ClientImpl {
        inner: zclient,
        service_name: service_str.to_string(),
        options: rmw_client_options_t {
            qos: unsafe { *qos_policies },
        },
    };

    let client = Box::new(rmw_client_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: service_name as *const _,
    });

    let client_ptr = Box::into_raw(client);
    unsafe {
        client_ptr.assign_data(client_impl).unwrap_or(());
    }

    client_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_client(
    node: *mut rmw_node_t,
    client: *mut rmw_client_t,
) -> rmw_ret_t {
    if node.is_null() || client.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(client) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_service(
    node: *const rmw_node_t,
    type_support: *const rosidl_service_type_support_t,
    service_name: *const std::os::raw::c_char,
    qos_profile: *const rmw_qos_profile_t,
) -> *mut rmw_service_t {
    if node.is_null() || type_support.is_null() || service_name.is_null() || qos_profile.is_null() {
        return std::ptr::null_mut();
    }

    let node_impl = match unsafe { node.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_str = unsafe { std::ffi::CStr::from_ptr(service_name) }
        .to_str()
        .unwrap_or("");

    // Create service using ros-z
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { *qos_profile });
    let zserver = match node_impl
        .inner
        .create_server::<rcl_z::msg::RosService>(service_str)
        .with_qos(qos)
        .build()
    {
        Ok(server) => server,
        Err(e) => {
            tracing::error!("Failed to create service: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_name_cstr = match std::ffi::CString::new(service_str) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_impl = crate::service::ServiceImpl {
        inner: zserver,
        service_name: service_name_cstr,
    };

    let service = Box::new(rmw_service_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: service_name as *const _,
    });

    let service_ptr = Box::into_raw(service);
    unsafe {
        service_ptr.assign_data(service_impl).unwrap_or(());
    }

    service_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_service(
    node: *mut rmw_node_t,
    service: *mut rmw_service_t,
) -> rmw_ret_t {
    if node.is_null() || service.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(service) });
    RMW_RET_OK as _
}

// Graph queries
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_node_names(
    node: *const rmw_node_t,
    node_names: *mut rcutils_string_array_t,
    node_namespaces: *mut rcutils_string_array_t,
) -> rmw_ret_t {
    if node.is_null() || node_names.is_null() || node_namespaces.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match unsafe { node.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for all nodes
    let nodes = node_impl.graph.get_all_nodes();

    // For now, just return OK with empty lists
    // Full implementation would populate the string arrays
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_node_names_with_enclaves(
    node: *const rmw_node_t,
    node_names: *mut rcutils_string_array_t,
    node_namespaces: *mut rcutils_string_array_t,
    enclaves: *mut rcutils_string_array_t,
) -> rmw_ret_t {
    if node.is_null() || node_names.is_null() || node_namespaces.is_null() || enclaves.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Delegate to rmw_get_node_names for now
    rmw_get_node_names(node, node_names, node_namespaces)
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_topic_names_and_types(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || topic_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Full implementation would query the graph
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || service_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Full implementation would query the graph
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_publishers(
    node: *const rmw_node_t,
    topic_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || topic_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        *count = 0;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_subscribers(
    node: *const rmw_node_t,
    topic_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || topic_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        *count = 0;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_node_get_graph_guard_condition(
    node: *const rmw_node_t,
) -> *const rmw_guard_condition_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_clients(
    node: *const rmw_node_t,
    service_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_services(
    node: *const rmw_node_t,
    service_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publishers_info_by_topic(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    topic_name: *const std::os::raw::c_char,
    no_mangle: bool,
    publishers_info: *mut rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_subscriptions_info_by_topic(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    topic_name: *const std::os::raw::c_char,
    no_mangle: bool,
    subscriptions_info: *mut rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_subscriber_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publisher_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_client_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    client_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialize(
    ros_message: *const c_void,
    type_support: *const rosidl_message_type_support_t,
    serialized_message: *mut rcl_serialized_message_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_deserialize(
    serialized_message: *const rcl_serialized_message_t,
    type_support: *const rosidl_message_type_support_t,
    ros_message: *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialization_support_init(
    serialization_support: *mut rmw_serialization_support_t,
    allocator: *const rcl_allocator_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_borrow_loaned_message(
    publisher: *const rmw_publisher_t,
    type_support: *const rosidl_message_type_support_t,
    ros_message: *mut *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_return_loaned_message_from_publisher(
    publisher: *const rmw_publisher_t,
    loaned_message: *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_return_loaned_message_from_subscription(
    subscription: *const rmw_subscription_t,
    loaned_message: *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialized_message_size(
    type_support: *const rosidl_message_type_support_t,
    message_bounds: *const rosidl_message_bounds_t,
    size: *mut usize,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_event_init(
    rmw_event: *mut rmw_event_t,
    publisher: *const rmw_publisher_t,
    event_type: rmw_event_type_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_event_init(
    rmw_event: *mut rmw_event_t,
    subscription: *const rmw_subscription_t,
    event_type: rmw_event_type_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_set_callback(
    event: *mut rmw_event_t,
    callback: rmw_event_callback_t,
    user_data: *mut c_void,
    allocator: *const rcl_allocator_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_type_is_supported(event_type: rmw_event_type_t) -> bool {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_event(
    event: *const rmw_event_t,
    event_info: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_set_on_new_message_callback(
    subscription: *mut rmw_subscription_t,
    callback: rmw_subscription_new_message_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_set_on_new_request_callback(
    service: *mut rmw_service_t,
    callback: rmw_service_new_request_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_client_set_on_new_response_callback(
    client: *mut rmw_client_t,
    callback: rmw_client_new_response_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_qos_profile_check_compatible(
    publisher_profile: rmw_qos_profile_t,
    subscription_profile: rmw_qos_profile_t,
    compatibility: *mut rmw_qos_compatibility_type_t,
    reason: *mut ::std::os::raw::c_char,
    reason_size: usize,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_publisher(
    publisher: *const rmw_publisher_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_client(
    client: *const rmw_client_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_compare_gids_equal(
    gid1: *const rmw_gid_t,
    gid2: *const rmw_gid_t,
    result: *mut bool,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_server_is_available(
    node: *const rmw_node_t,
    client: *const rmw_client_t,
    is_available: *mut bool,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_publisher_allocation(
    type_support: *const rosidl_message_type_support_t,
    message_bounds: *const rosidl_message_bounds_t,
    allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_fini_publisher_allocation(
    allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_subscription_allocation(
    type_support: *const rosidl_message_type_support_t,
    message_bounds: *const rosidl_message_bounds_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_fini_subscription_allocation(
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_dynamic_message(
    subscription: *const rmw_subscription_t,
    dynamic_message: *mut rcldynamic_message_t,
    taken: *mut bool,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_dynamic_message_with_info(
    subscription: *const rmw_subscription_t,
    dynamic_message: *mut rcldynamic_message_t,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_feature_supported(feature: rmw_feature_t) -> bool {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_set_log_severity(severity: rmw_log_severity_t) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_test_isolation_start() -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_test_isolation_stop() -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_zenoh_get_session(context: *const rmw_context_t) -> *const c_void {
    todo!()
}
