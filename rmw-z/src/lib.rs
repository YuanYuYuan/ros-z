#![allow(clippy::enum_variant_names)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::explicit_auto_deref)]
#![allow(clippy::collapsible_if)]
#![allow(clippy::cast_slice_from_raw_parts)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::useless_transmute)]
#![allow(clippy::missing_transmute_annotations)]
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
use crate::ros::rmw_feature_t;

use rcl_z::type_support::MessageTypeSupport;
use ros_z::{event::{RmEventHandle, ZenohEventType}, Builder};

use crate::{pubsub::PublisherImpl, ros::*, traits::*};

/// Wait set implementation for RMW
pub struct WaitSetImpl {
    pub subscriptions: Vec<*mut rmw_subscription_t>,
    pub guard_conditions: Vec<*mut rmw_guard_condition_t>,
    pub services: Vec<*mut rmw_service_t>,
    pub clients: Vec<*mut rmw_client_t>,
    pub events: Vec<*mut rmw_event_t>,
}

impl WaitSetImpl {
    pub fn new(max_conditions: usize) -> Self {
        Self {
            subscriptions: Vec::with_capacity(max_conditions),
            guard_conditions: Vec::with_capacity(max_conditions),
            services: Vec::with_capacity(max_conditions),
            clients: Vec::with_capacity(max_conditions),
            events: Vec::with_capacity(max_conditions),
        }
    }

    pub fn wait(&self, _timeout: &rmw_time_t) -> bool {
        // Simple implementation - check if any waitable is ready
        for sub in &self.subscriptions {
            if let Ok(sub_impl) = (*sub).borrow_data() {
                if sub_impl.is_ready() {
                    return true;
                }
            }
        }
        for gc in &self.guard_conditions {
            if let Ok(gc_impl) = (*gc).borrow_data() {
                if gc_impl.is_ready() {
                    return true;
                }
            }
        }
        for srv in &self.services {
            if let Ok(srv_impl) = (*srv).borrow_data() {
                if srv_impl.is_ready() {
                    return true;
                }
            }
        }
        for cli in &self.clients {
            if let Ok(cli_impl) = (*cli).borrow_data() {
                if cli_impl.is_ready() {
                    return true;
                }
            }
        }
        false
    }
}

rmw_impl_has_impl_ptr!(rmw_wait_set_t, rmw_wait_set_impl_t, WaitSetImpl);

// RMW init
#[unsafe(no_mangle)]
pub extern "C" fn rmw_init(
    options: *const crate::ros::rmw_init_options_t,
    context: *mut crate::ros::rmw_context_t,
) -> rmw_ret_t {
    if options.is_null() || context.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    // For rmw-z, minimal implementation
    unsafe {
        (*context).instance_id = (*options).instance_id;
        (*context).implementation_identifier = RMW_ZENOH_IDENTIFIER.as_ptr() as *const _;
        (*context).actual_domain_id = (*options).domain_id;
        (*context).impl_ = std::ptr::null_mut();
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_options_init(
    init_options: *mut crate::ros::rmw_init_options_t,
    allocator: crate::ros::rcl_allocator_t,
) -> rmw_ret_t {
    if init_options.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    unsafe {
        if !(*init_options).implementation_identifier.is_null() {
            return RMW_RET_INVALID_ARGUMENT as _;
        }
        (*init_options).instance_id = 0;
        (*init_options).implementation_identifier = RMW_ZENOH_IDENTIFIER.as_ptr() as *const _;
        (*init_options).domain_id = 0; // RMW_DEFAULT_DOMAIN_ID equivalent
        (*init_options).security_options = std::ptr::null_mut();
        (*init_options).discovery_options = std::ptr::null_mut();
        (*init_options).allocator = allocator;
        (*init_options).enclave = std::ptr::null();
        (*init_options).impl_ = std::ptr::null_mut();
    }
    RMW_RET_OK as _
}

// RMW Context functions
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
    match context.own_impl() {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

// RMW Init Options functions
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

// RMW Node functions
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
        (*node_ptr).data = Box::into_raw(Box::new(node_impl)) as *mut _;
    }

    node_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_node(node: *mut rmw_node_t) -> rmw_ret_t {
    if node.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Drop the implementation data
    let _ = node.own_data();

    drop(unsafe { Box::from_raw(node) });
    RMW_RET_OK as _
}

// RMW Wait Set functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_wait_set(
    context: *mut rmw_context_t,
    max_conditions: usize,
) -> *mut rmw_wait_set_t {
    if context.is_null() {
        return std::ptr::null_mut();
    }

    let wait_set_impl = WaitSetImpl::new(max_conditions);
    let wait_set = Box::new(rmw_wait_set_t {
        impl_: std::ptr::null_mut(),
    });

    let wait_set_ptr = Box::into_raw(wait_set);
    wait_set_ptr.assign_impl(wait_set_impl).unwrap_or(());

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
    _events: *mut rmw_events_t,
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

        // Similar for guard conditions
        if !guard_conditions.is_null() {
            let gc_array = unsafe { &mut *guard_conditions };
            let mut ready_count = 0;
            for i in 0..gc_array.guard_condition_count {
                let gc = unsafe { *gc_array.guard_conditions.add(i) };
                if !gc.is_null() {
                    if let Ok(gc_impl) = gc.borrow_data() {
                        if gc_impl.is_ready() {
                            unsafe {
                                *gc_array.guard_conditions.add(ready_count) = gc;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            for i in ready_count..gc_array.guard_condition_count {
                unsafe {
                    *gc_array.guard_conditions.add(i) = std::ptr::null_mut();
                }
            }
        }

        RMW_RET_OK as _
    } else {
        RMW_RET_TIMEOUT as _
    }
}

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

    let node_impl = match node.borrow_data() {
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
        .create_pub::<rcl_z::msg::RosMessage>(topic_str)
        .with_serdes::<rcl_z::msg::RosSerdes>();
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_profile });
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
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    if publisher.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match publisher.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match publisher_impl.publish(ros_message as *const std::os::raw::c_void) {
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

    let node_impl = match node.borrow_data() {
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
        .create_sub::<rcl_z::msg::RosMessage>(topic_str)
        .with_serdes::<rcl_z::msg::RosSerdes>();
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_policies });
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
        callback: std::sync::Mutex::new(None),
        callback_user_data: std::sync::Mutex::new(std::ptr::null()),
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
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || ros_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take(ros_message as *mut std::os::raw::c_void, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_event(
    _event: *const rmw_event_t,
    _event_info: *mut c_void,
    _taken: *mut bool,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_loaned_message(
    _subscription: *const rmw_subscription_t,
    _loaned_message: *mut *mut c_void,
    _taken: *mut bool,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_loaned_message_with_info(
    _subscription: *const rmw_subscription_t,
    _loaned_message: *mut *mut c_void,
    _taken: *mut bool,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_sequence(
    _subscription: *const rmw_subscription_t,
    _count: usize,
    _ros_message_sequence: *mut *mut c_void,
    _ros_message_info_sequence: *mut *mut rmw_message_info_t,
    _taken: *mut usize,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_serialized_message(
    _subscription: *const rmw_subscription_t,
    _serialized_message: *mut rcl_serialized_message_t,
    _taken: *mut bool,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_serialized_message_with_info(
    _subscription: *const rmw_subscription_t,
    _serialized_message: *mut rcl_serialized_message_t,
    _taken: *mut bool,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_count_matched_publishers(
    _subscription: *const rmw_subscription_t,
    _publisher_count: *mut usize,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_actual_qos(
    subscription: *const rmw_subscription_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if subscription.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    unsafe {
        *qos = subscription_impl.qos;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_content_filter(
    _subscription: *const rmw_subscription_t,
    _allocator: *const rcl_allocator_t,
    _options: *mut rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_network_flow_endpoints(
    _subscription: *const rmw_subscription_t,
    _allocator: *const rcl_allocator_t,
    _network_flow_endpoints: *mut rmw_network_flow_endpoint_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_set_content_filter(
    _subscription: *mut rmw_subscription_t,
    _options: *const rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_str = unsafe { std::ffi::CStr::from_ptr(service_name) }
        .to_str()
        .unwrap_or("");

    // Create client using ros-z
    let _qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_policies });
    let zclient = match node_impl
        .inner
        .create_client::<rcl_z::msg::RosService>(service_str)
        .build()
    {
        Ok(client) => client,
        Err(e) => {
            tracing::error!("Failed to create client: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_type_support =
        match unsafe { rcl_z::type_support::ServiceTypeSupport::new(type_support) } {
            Ok(ts) => ts,
            Err(e) => {
                tracing::error!("Failed to create service type support: {}", e);
                return std::ptr::null_mut();
            }
        };

    let client_impl = crate::service::ClientImpl {
        inner: zclient,
        service_name: service_str.to_string(),
        options: rmw_client_options_t {
            qos: unsafe { *qos_policies },
        },
        request_ts: service_type_support,
        response_ts: service_type_support,
        callback: std::sync::Mutex::new(None),
        callback_user_data: std::sync::Mutex::new(std::ptr::null()),
    };

    let client = Box::new(rmw_client_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: service_name as *const _,
    });

    let client_ptr = Box::into_raw(client);
    client_ptr.assign_data(client_impl).unwrap_or(());

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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_str = unsafe { std::ffi::CStr::from_ptr(service_name) }
        .to_str()
        .unwrap_or("");

    // Create service using ros-z
    let _qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_profile });
    let zserver = match node_impl
        .inner
        .create_service::<rcl_z::msg::RosService>(service_str)
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

    let service_type_support =
        match unsafe { rcl_z::type_support::ServiceTypeSupport::new(type_support) } {
            Ok(ts) => ts,
            Err(e) => {
                tracing::error!("Failed to create service type support: {}", e);
                return std::ptr::null_mut();
            }
        };

    let service_impl = crate::service::ServiceImpl {
        inner: zserver,
        service_name: service_name_cstr,
        request_ts: service_type_support,
        response_ts: service_type_support,
        qos: unsafe { *qos_profile },
        callback: std::sync::Mutex::new(None),
        callback_user_data: std::sync::Mutex::new(std::ptr::null()),
    };

    let service = Box::new(rmw_service_t {
        implementation_identifier: RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: service_name as *const _,
    });

    let service_ptr = Box::into_raw(service);
    service_ptr.assign_data(service_impl).unwrap_or(());

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
    node_names: *mut rcl_z::ros::rcutils_string_array_s,
    node_namespaces: *mut rcl_z::ros::rcutils_string_array_s,
) -> rmw_ret_t {
    use std::ffi::CString;

    if node.is_null() || node_names.is_null() || node_namespaces.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for all nodes
    let nodes = node_impl.graph.get_node_names();

    // Convert to CString vectors
    let names_vec: Vec<CString> = nodes
        .iter()
        .map(|(name, _)| CString::new(name.as_str()).unwrap())
        .collect();

    let namespaces_vec: Vec<CString> = nodes
        .iter()
        .map(|(_, ns)| CString::new(ns.as_str()).unwrap())
        .collect();

    // Convert and assign to output arrays
    unsafe {
        *node_names = std::mem::transmute(rcl_z::ros::rcutils_string_array_t::from(names_vec));
        *node_namespaces = std::mem::transmute(rcl_z::ros::rcutils_string_array_t::from(namespaces_vec));
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_node_names_with_enclaves(
    node: *const rmw_node_t,
    node_names: *mut rcutils_string_array_t,
    node_namespaces: *mut rcutils_string_array_t,
    enclaves: *mut rcutils_string_array_t,
) -> rmw_ret_t {
    use std::ffi::CString;

    if node.is_null() || node_names.is_null() || node_namespaces.is_null() || enclaves.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for all nodes with enclaves
    let nodes = node_impl.graph.get_node_names_with_enclaves();

    // Convert to CString vectors
    let names_vec: Vec<CString> = nodes
        .iter()
        .map(|(name, _, _)| CString::new(name.as_str()).unwrap())
        .collect();

    let namespaces_vec: Vec<CString> = nodes
        .iter()
        .map(|(_, ns, _)| CString::new(ns.as_str()).unwrap())
        .collect();

    let _enclaves_vec: Vec<CString> = nodes
        .iter()
        .map(|(_, _, enc)| CString::new(enc.as_str()).unwrap())
        .collect();

    // Convert and assign to output arrays
    unsafe {
        *node_names = std::mem::transmute(rcl_z::ros::rcutils_string_array_t::from(names_vec));
        *node_namespaces = std::mem::transmute(rcl_z::ros::rcutils_string_array_t::from(namespaces_vec));
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_topic_names_and_types(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    _no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || topic_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Since graph cache is not implemented in rmw-z, return unsupported
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || service_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Since graph cache is not implemented in rmw-z, return unsupported
    RMW_RET_UNSUPPORTED as _
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let topic_str = match unsafe { std::ffi::CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for publisher count on this topic
    let publisher_count = node_impl
        .graph
        .count(ros_z::entity::EntityKind::Publisher, topic_str);

    unsafe {
        *count = publisher_count;
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let topic_str = match unsafe { std::ffi::CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for subscriber count on this topic
    let subscriber_count = node_impl
        .graph
        .count(ros_z::entity::EntityKind::Subscription, topic_str);

    unsafe {
        *count = subscriber_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_node_get_graph_guard_condition(
    node: *const rmw_node_t,
) -> *const rmw_guard_condition_t {
    if node.is_null() {
        return std::ptr::null();
    }
    // Since rmw-z does not implement graph guard condition, return null
    std::ptr::null()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_clients(
    node: *const rmw_node_t,
    service_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || service_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let service_str = match unsafe { std::ffi::CStr::from_ptr(service_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for client count on this service
    let client_count = node_impl
        .graph
        .count_by_service(ros_z::entity::EntityKind::Client, service_str);

    unsafe {
        *count = client_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_count_services(
    node: *const rmw_node_t,
    service_name: *const std::os::raw::c_char,
    count: *mut usize,
) -> rmw_ret_t {
    if node.is_null() || service_name.is_null() || count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let service_str = match unsafe { std::ffi::CStr::from_ptr(service_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for service count on this service
    let service_count = node_impl
        .graph
        .count_by_service(ros_z::entity::EntityKind::Service, service_str);

    unsafe {
        *count = service_count;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publishers_info_by_topic(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    topic_name: *const std::os::raw::c_char,
    _no_mangle: bool,
    publishers_info: *mut rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || topic_name.is_null() || publishers_info.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    // Since graph cache is not implemented in rmw-z, return unsupported
    RMW_RET_UNSUPPORTED as _
}



#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_subscriber_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    _no_demangle: bool,
    topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || node_name.is_null() || node_namespace.is_null() || topic_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    // Since graph cache is not implemented in rmw-z, return unsupported
    RMW_RET_UNSUPPORTED as _
}







#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialize(
    _ros_message: *const c_void,
    _type_support: *const rosidl_message_type_support_t,
    _serialized_message: *mut rcl_serialized_message_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_deserialize(
    serialized_message: *const rcl_serialized_message_t,
    type_support: *const rosidl_message_type_support_t,
    ros_message: *mut c_void,
) -> rmw_ret_t {
    if serialized_message.is_null() || type_support.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let ts = match unsafe { MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return RMW_RET_ERROR as _,
    };

    let data = unsafe {
        let msg = &*serialized_message;
        std::slice::from_raw_parts(msg.buffer, msg.buffer_length)
    };
    let data_vec = data.to_vec();

    let res = unsafe { ts.deserialize_message(&data_vec, ros_message as *mut rcl_z::c_void) };
    if res {
        RMW_RET_OK as _
    } else {
        RMW_RET_ERROR as _
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialization_support_init(
    _serialization_support: *mut rmw_serialization_support_t,
    _allocator: *const rcl_allocator_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_borrow_loaned_message(
    publisher: *const rmw_publisher_t,
    type_support: *const rosidl_message_type_support_t,
    ros_message: *mut *mut c_void,
) -> rmw_ret_t {
    // Validate input arguments
    if publisher.is_null() || type_support.is_null() || ros_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_return_loaned_message_from_publisher(
    publisher: *const rmw_publisher_t,
    loaned_message: *mut c_void,
) -> rmw_ret_t {
    // Validate input arguments
    if publisher.is_null() || loaned_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_return_loaned_message_from_subscription(
    _subscription: *const rmw_subscription_t,
    _loaned_message: *mut c_void,
) -> rmw_ret_t {
    // Loaned messages are not currently supported in this implementation
    // Return RMW_RET_UNSUPPORTED to match the behavior of rmw_zenoh_cpp
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialized_message_size(
    _type_support: *const rosidl_message_type_support_t,
    _message_bounds: *const rosidl_message_bounds_t,
    _size: *mut usize,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_event_init(
    _rmw_event: *mut rmw_event_t,
    _publisher: *const rmw_publisher_t,
    _event_type: rmw_event_type_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_event_init(
    _rmw_event: *mut rmw_event_t,
    _subscription: *const rmw_subscription_t,
    _event_type: rmw_event_type_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_set_callback(
    event: *mut rmw_event_t,
    callback: rmw_event_callback_t,
    user_data: *mut c_void,
    _allocator: *const rcl_allocator_t,
) -> rmw_ret_t {
    if event.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if unsafe { (*event).data }.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let rm_event_handle = unsafe { &mut *((*event).data as *mut RmEventHandle) };
    let user_data_ptr = user_data as usize;
    rm_event_handle.set_callback(move |change: i32| {
        if let Some(cb) = callback {
            let ud = user_data_ptr as *mut ::std::os::raw::c_void;
            unsafe { cb(ud as *const ::std::os::raw::c_void, change as usize) };
        }
    });

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_event_type_is_supported(event_type: rmw_event_type_t) -> bool {
    Option::<ZenohEventType>::from(event_type).is_some()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_send_request(
    client: *const rmw_client_t,
    ros_request: *const c_void,
    sequence_id: *mut i64,
) -> rmw_ret_t {
    if client.is_null() || ros_request.is_null() || sequence_id.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match client.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match client_impl.send_request(ros_request, sequence_id) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to send request: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_send_response(
    service: *const rmw_service_t,
    response_header: *mut rmw_request_id_t,
    ros_response: *mut c_void,
) -> rmw_ret_t {
    if service.is_null() || response_header.is_null() || ros_response.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match (service as *mut rmw_service_t).borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match service_impl.send_response(response_header, ros_response) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to send response: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_set_on_new_message_callback(
    subscription: *mut rmw_subscription_t,
    callback: rmw_subscription_new_message_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    if subscription.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match subscription.borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if let Ok(mut cb) = subscription_impl.callback.lock() {
        *cb = callback;
    }
    if let Ok(mut ud) = subscription_impl.callback_user_data.lock() {
        *ud = user_data as *const c_void;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_set_on_new_request_callback(
    service: *mut rmw_service_t,
    callback: rmw_service_new_request_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    if service.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match service.borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if let Ok(mut cb) = service_impl.callback.lock() {
        *cb = callback;
    }
    if let Ok(mut ud) = service_impl.callback_user_data.lock() {
        *ud = user_data as *const c_void;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_client_set_on_new_response_callback(
    client: *mut rmw_client_t,
    callback: rmw_client_new_response_callback_t,
    user_data: *mut c_void,
) -> rmw_ret_t {
    if client.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match client.borrow_mut_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if let Ok(mut cb) = client_impl.callback.lock() {
        *cb = callback;
    }
    if let Ok(mut ud) = client_impl.callback_user_data.lock() {
        *ud = user_data as *const c_void;
    }

    RMW_RET_OK as _
}



// NOTE: These functions are commented out to avoid linker conflicts with librmw
// They will be implemented when needed
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_publisher(
    publisher: *const rmw_publisher_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    if publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if gid.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if unsafe { (*publisher).implementation_identifier } != RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    // For now, return unsupported since gid is not implemented in rmw-z
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_gid_for_client(
    client: *const rmw_client_t,
    gid: *mut rmw_gid_t,
) -> rmw_ret_t {
    if client.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if gid.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if unsafe { (*client).implementation_identifier } != RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    // For now, return unsupported since gid is not implemented in rmw-z
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_compare_gids_equal(
    gid1: *const rmw_gid_t,
    gid2: *const rmw_gid_t,
    result: *mut bool,
) -> rmw_ret_t {
    if gid1.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let gid1_ref = unsafe { &*gid1 };
    if gid1_ref.implementation_identifier != RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    if gid2.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let gid2_ref = unsafe { &*gid2 };
    if gid2_ref.implementation_identifier != RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    if result.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        *result = gid1_ref.data == gid2_ref.data;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_request_subscription_get_actual_qos(
    service: *const rmw_service_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if service.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match service.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    unsafe {
        *qos = service_impl.qos;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_response_publisher_get_actual_qos(
    service: *const rmw_service_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    // The same QoS profile is used for receiving requests and sending responses.
    rmw_service_request_subscription_get_actual_qos(service, qos)
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_server_is_available(
    _node: *const rmw_node_t,
    _client: *const rmw_client_t,
    _is_available: *mut bool,
) -> rmw_ret_t {
    // Since graph cache is not implemented in rmw-z, return unsupported
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_publisher_allocation(
    _type_support: *const rosidl_message_type_support_t,
    _message_bounds: *const rosidl_message_bounds_t,
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_fini_publisher_allocation(
    _allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_subscription_allocation(
    _type_support: *const rosidl_message_type_support_t,
    _message_bounds: *const rosidl_message_bounds_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_fini_subscription_allocation(
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_dynamic_message(
    _subscription: *const rmw_subscription_t,
    _dynamic_message: *mut rcldynamic_message_t,
    _taken: *mut bool,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_dynamic_message_with_info(
    _subscription: *const rmw_subscription_t,
    _dynamic_message: *mut rcldynamic_message_t,
    _taken: *mut bool,
    _message_info: *mut rmw_message_info_t,
    _allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_feature_supported(feature: rmw_feature_t) -> bool {
    match feature {
        rmw_feature_t::RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER => false,
        rmw_feature_t::RMW_FEATURE_MESSAGE_INFO_RECEPTION_SEQUENCE_NUMBER => false,
        rmw_feature_t::RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY => true,
        rmw_feature_t::RMW_MIDDLEWARE_CAN_TAKE_DYNAMIC_MESSAGE => false,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_set_log_severity(_severity: rmw_log_severity_t) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_test_isolation_start() -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_test_isolation_stop() -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_clients_info_by_service(
    _node: *const rmw_node_t,
    _allocator: *mut rcl_z::ros::rcutils_allocator_t,
    _service_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    _clients_info: *mut rcl_z::ros::rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_client_names_and_types_by_node(
    _node: *const rmw_node_t,
    _allocator: *const rcl_allocator_t,
    _node_name: *const std::os::raw::c_char,
    _node_namespace: *const std::os::raw::c_char,
    _service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publisher_names_and_types_by_node(
    _node: *const rmw_node_t,
    _allocator: *const rcl_allocator_t,
    _node_name: *const std::os::raw::c_char,
    _node_namespace: *const std::os::raw::c_char,
    _no_demangle: bool,
    _topic_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_servers_info_by_service(
    _node: *const rmw_node_t,
    _allocator: *mut rcl_z::ros::rcutils_allocator_t,
    _service_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    _servers_info: *mut rcl_z::ros::rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types_by_node(
    _node: *const rmw_node_t,
    _allocator: *const rcl_allocator_t,
    _node_name: *const std::os::raw::c_char,
    _node_namespace: *const std::os::raw::c_char,
    _service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_subscriptions_info_by_topic(
    _node: *const rmw_node_t,
    _allocator: *const rcl_allocator_t,
    _topic_name: *const std::os::raw::c_char,
    _no_mangle: bool,
    _subscriptions_info: *mut rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_zenoh_get_session(_context: *const rmw_context_t) -> *const c_void {
    todo!()
}
