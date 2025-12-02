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

use crate::{
    pubsub::PublisherImpl,
    ros::*,
    traits::{BorrowData, BorrowImpl, OwnData, OwnImpl},
};

// Implement the actual RMW functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_implementation_identifier() -> *const std::os::raw::c_char {
    RMW_ZENOH_IDENTIFIER.as_ptr() as *const std::os::raw::c_char
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialization_format() -> *const std::os::raw::c_char {
    RMW_ZENOH_SERIALIZATION_FORMAT.as_ptr() as *const std::os::raw::c_char
}

// Context initialization
#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_options_init(
    init_options: *mut rmw_init_options_t,
    domain_id: usize,
    allocator: rcl_allocator_t,
) -> rmw_ret_t {
    if init_options.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Initialize options structure
    // For now, just mark as initialized
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_options_copy(
    src: *const rmw_init_options_t,
    dst: *mut rmw_init_options_t,
) -> rmw_ret_t {
    if src.is_null() || dst.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_options_fini(init_options: *mut rmw_init_options_t) -> rmw_ret_t {
    if init_options.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init(
    options: *const rmw_init_options_t,
    context: *mut rmw_context_t,
) -> rmw_ret_t {
    if options.is_null() || context.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Check if already initialized
    if !unsafe { (*context).impl_.is_null() } {
        return RMW_RET_ALREADY_INIT as _;
    }

    // Create context implementation
    // TODO: Extract domain_id from options
    let domain_id = 0;
    let context_impl = match context::ContextImpl::new(domain_id) {
        Ok(impl_) => impl_,
        Err(e) => {
            tracing::error!("Failed to create context: {}", e);
            return RMW_RET_ERROR as _;
        }
    };

    // Assign implementation
    match (context as *mut rmw_context_t).assign_impl(context_impl) {
        Ok(_) => {
            unsafe { (*context).instance_id = 1 }; // TODO: proper instance ID
            RMW_RET_OK as _
        }
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_shutdown(context: *mut rmw_context_t) -> rmw_ret_t {
    if context.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Shutdown happens implicitly when context is finalized
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_context_fini(context: *mut rmw_context_t) -> rmw_ret_t {
    if context.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    if unsafe { (*context).impl_.is_null() } {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Own and drop the implementation
    match (context as *mut rmw_context_t).own_impl() {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_node(
    context: *mut rmw_context_t,
    name: *const std::os::raw::c_char,
    namespace_: *const std::os::raw::c_char,
) -> *mut rmw_node_t {
    if context.is_null() || name.is_null() {
        return std::ptr::null_mut();
    }

    let context_impl = match context.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let name_str = unsafe { std::ffi::CStr::from_ptr(name) }
        .to_str()
        .unwrap_or("");
    let namespace_str = unsafe { std::ffi::CStr::from_ptr(namespace_) }
        .to_str()
        .unwrap_or("");

    let node_impl = match context_impl.new_node(name, namespace_, context, std::ptr::null()) {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let node = Box::new(rmw_node_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        name: name as *const _,
        namespace_: namespace_ as *const _,
        context,
    });

    let node_ptr = Box::into_raw(node);
    unsafe {
        (*node_ptr).assign_data(node_impl).unwrap_or(());
    }

    node_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_node(node: *mut rmw_node_t) -> rmw_ret_t {
    if node.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(node) });
    RMW_RET_OK as _
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

    let node_impl = match unsafe { node.borrow_impl() } {
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

    let type_info = ts.get_type_info();
    let zpub_builder = node_impl.inner.create_pub_impl(topic_str, Some(type_info));
    let zpub_builder = zpub_builder.with_serdes::<rcl_z::msg::RosSerdes>();
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
    };

    let publisher = Box::new(rmw_publisher_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        topic_name: topic_name as *const _,
        options: publisher_options as *const _,
    });

    let publisher_ptr = Box::into_raw(publisher);
    unsafe {
        (*publisher_ptr).assign_data(publisher_impl).unwrap_or(());
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
    };

    let subscription = Box::new(rmw_subscription_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        topic_name: topic_name as *const _,
        options: subscription_options as *const _,
    });

    let subscription_ptr = Box::into_raw(subscription);
    unsafe {
        (*subscription_ptr)
            .assign_data(subscription_impl)
            .unwrap_or(());
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
        (*client_ptr).assign_data(client_impl).unwrap_or(());
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
        (*service_ptr).assign_data(service_impl).unwrap_or(());
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

// Wait sets
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_wait_set(
    context: *mut rmw_context_t,
    max_conditions: usize,
) -> *mut rmw_wait_set_t {
    if context.is_null() {
        return std::ptr::null_mut();
    }

    let wait_set_impl = crate::wait_set::WaitSetImpl::new(max_conditions);
    let wait_set = Box::new(rmw_wait_set_t {
        impl_: std::ptr::null_mut(),
    });

    let wait_set_ptr = Box::into_raw(wait_set);
    unsafe {
        (*wait_set_ptr).assign_impl(wait_set_impl).unwrap_or(());
    }

    wait_set_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_wait_set(wait_set: *mut rmw_wait_set_t) -> rmw_ret_t {
    if wait_set.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(wait_set) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_wait(
    subscriptions: *mut rmw_subscriptions_t,
    guard_conditions: *mut rmw_guard_conditions_t,
    services: *mut rmw_services_t,
    clients: *mut rmw_clients_t,
    events: *mut rmw_events_t,
    wait_set: *mut rmw_wait_set_t,
    wait_timeout: *const rmw_time_t,
) -> rmw_ret_t {
    if wait_set.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let wait_set_impl = match wait_set.borrow_mut_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Clear the wait set
    wait_set_impl.subscriptions.clear();
    wait_set_impl.guard_conditions.clear();
    wait_set_impl.services.clear();
    wait_set_impl.clients.clear();
    wait_set_impl.events.clear();

    // Add subscriptions to wait set
    if !subscriptions.is_null() {
        let sub_array = unsafe { &*subscriptions };
        for i in 0..sub_array.subscriber_count {
            let sub = unsafe { *sub_array.subscribers.add(i) };
            if !sub.is_null() {
                wait_set_impl.subscriptions.push(sub);
            }
        }
    }

    // Add guard conditions
    if !guard_conditions.is_null() {
        let gc_array = unsafe { &*guard_conditions };
        for i in 0..gc_array.guard_condition_count {
            let gc = unsafe { *gc_array.guard_conditions.add(i) };
            if !gc.is_null() {
                wait_set_impl.guard_conditions.push(gc);
            }
        }
    }

    // Add services
    if !services.is_null() {
        let srv_array = unsafe { &*services };
        for i in 0..srv_array.service_count {
            let srv = unsafe { *srv_array.services.add(i) };
            if !srv.is_null() {
                wait_set_impl.services.push(srv);
            }
        }
    }

    // Add clients
    if !clients.is_null() {
        let cli_array = unsafe { &*clients };
        for i in 0..cli_array.client_count {
            let cli = unsafe { *cli_array.clients.add(i) };
            if !cli.is_null() {
                wait_set_impl.clients.push(cli);
            }
        }
    }

    // Wait for ready entities
    let timeout = if wait_timeout.is_null() {
        rmw_time_t { sec: -1, nsec: 0 }
    } else {
        unsafe { *wait_timeout }
    };

    let ready = wait_set_impl.wait(&timeout);

    if ready {
        // Update arrays to only contain ready entities
        if !subscriptions.is_null() {
            let sub_array = unsafe { &mut *subscriptions };
            let mut ready_count = 0;
            for i in 0..sub_array.subscriber_count {
                let sub = unsafe { *sub_array.subscribers.add(i) };
                if !sub.is_null() {
                    if let Ok(sub_impl) = sub.borrow_data() {
                        if sub_impl.is_ready() {
                            unsafe {
                                *sub_array.subscribers.add(ready_count) = sub;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            // Null out the rest
            for i in ready_count..sub_array.subscriber_count {
                unsafe {
                    *sub_array.subscribers.add(i) = std::ptr::null_mut();
                }
            }
        }

        // Similar for services
        if !services.is_null() {
            let srv_array = unsafe { &mut *services };
            let mut ready_count = 0;
            for i in 0..srv_array.service_count {
                let srv = unsafe { *srv_array.services.add(i) };
                if !srv.is_null() {
                    if let Ok(srv_impl) = srv.borrow_data() {
                        if srv_impl.is_ready() {
                            unsafe {
                                *srv_array.services.add(ready_count) = srv;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            for i in ready_count..srv_array.service_count {
                unsafe {
                    *srv_array.services.add(i) = std::ptr::null_mut();
                }
            }
        }

        // Similar for clients
        if !clients.is_null() {
            let cli_array = unsafe { &mut *clients };
            let mut ready_count = 0;
            for i in 0..cli_array.client_count {
                let cli = unsafe { *cli_array.clients.add(i) };
                if !cli.is_null() {
                    if let Ok(cli_impl) = cli.borrow_data() {
                        if cli_impl.is_ready() {
                            unsafe {
                                *cli_array.clients.add(ready_count) = cli;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            for i in ready_count..cli_array.client_count {
                unsafe {
                    *cli_array.clients.add(i) = std::ptr::null_mut();
                }
            }
        }

        RMW_RET_OK as _
    } else {
        RMW_RET_TIMEOUT as _
    }
}

// Guard conditions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_guard_condition(
    context: *mut rmw_context_t,
) -> *mut rmw_guard_condition_t {
    if context.is_null() {
        return std::ptr::null_mut();
    }

    let gc_impl = crate::guard_condition::GuardConditionImpl::new();
    let gc = Box::new(rmw_guard_condition_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        context,
    });

    let gc_ptr = Box::into_raw(gc);
    unsafe {
        (*gc_ptr).assign_data(gc_impl).unwrap_or(());
    }

    gc_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_guard_condition(
    guard_condition: *mut rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(guard_condition) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_trigger_guard_condition(
    guard_condition: *const rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    if let Ok(mut gc_impl) = (guard_condition as *mut rmw_guard_condition_t).borrow_mut_data() {
        gc_impl.trigger();
    }

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
