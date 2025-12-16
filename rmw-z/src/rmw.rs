#![allow(clippy::enum_variant_names)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::explicit_auto_deref)]
#![allow(clippy::collapsible_if)]
#![allow(clippy::cast_slice_from_raw_parts)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::useless_transmute)]
#![allow(clippy::missing_transmute_annotations)]
#![allow(unpredictable_function_pointer_comparisons)]

use crate::c_void;
use crate::ros::{
    rmw_event_callback_t, rmw_event_type_t, rmw_gid_t, rmw_topic_endpoint_info_array_t,
    rmw_feature_t,
};

use crate::type_support::MessageTypeSupport;
use ros_z::{event::{RmEventHandle, ZenohEventType}, Builder};

use crate::{pubsub::PublisherImpl, ros::*, traits::*};

// Implement the actual RMW functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_implementation_identifier() -> *const std::os::raw::c_char {
    eprintln!("[rmw_z] Successfully loaded! Identifier: {}",
              std::str::from_utf8(&crate::RMW_ZENOH_IDENTIFIER[..crate::RMW_ZENOH_IDENTIFIER.len()-1])
              .unwrap_or("rmw_z"));
    crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const std::os::raw::c_char
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_serialization_format() -> *const std::os::raw::c_char {
    crate::RMW_ZENOH_SERIALIZATION_FORMAT.as_ptr() as *const std::os::raw::c_char
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
        Err(e) => {
            eprintln!("[rmw_z] rmw_create_publisher: Failed to borrow node data: {:?}", e);
            let msg = std::ffi::CString::new("Failed to get node implementation").unwrap();
            unsafe { crate::ros::rcutils_set_error_state(msg.as_ptr(), file!().as_ptr() as *const _, line!() as usize) };
            return std::ptr::null_mut();
        }
    };

    let topic_str = unsafe { std::ffi::CStr::from_ptr(topic_name) }
        .to_str()
        .unwrap_or("");
    let ts = match unsafe { crate::type_support::MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(e) => {
            eprintln!("[rmw_z] rmw_create_publisher: Failed to create type support: {:?}", e);
            let msg = std::ffi::CString::new(format!("Failed to create type support: {}", e)).unwrap_or_else(|_| std::ffi::CString::new("Failed to create type support").unwrap());
            unsafe { crate::ros::rcutils_set_error_state(msg.as_ptr(), file!().as_ptr() as *const _, line!() as usize) };
            return std::ptr::null_mut();
        }
    };

    let zpub_builder = node_impl
        .inner
        .create_pub::<crate::msg::RosMessage>(topic_str)
        .with_serdes::<crate::msg::RosSerdes>();
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_profile });
    let zpub_builder = zpub_builder.with_qos(qos);
    let zpub = match zpub_builder.build() {
        Ok(zpub) => zpub,
        Err(e) => {
            eprintln!("[rmw_z] rmw_create_publisher: Failed to build ZPub: {:?}", e);
            let msg = std::ffi::CString::new(format!("Failed to build publisher: {}", e)).unwrap_or_else(|_| std::ffi::CString::new("Failed to build publisher").unwrap());
            unsafe { crate::ros::rcutils_set_error_state(msg.as_ptr(), file!().as_ptr() as *const _, line!() as usize) };
            return std::ptr::null_mut();
        }
    };

    let qualified_topic = zpub.entity.topic.clone();
    let entity = zpub.entity.clone();
    let topic_cstr = match std::ffi::CString::new(qualified_topic) {
        Ok(cstr) => cstr,
        Err(e) => {
            eprintln!("[rmw_z] rmw_create_publisher: Failed to create CString for topic: {:?}", e);
            let msg = std::ffi::CString::new("Failed to create topic string").unwrap();
            unsafe { crate::ros::rcutils_set_error_state(msg.as_ptr(), file!().as_ptr() as *const _, line!() as usize) };
            return std::ptr::null_mut();
        }
    };

    let publisher_impl = PublisherImpl {
        inner: zpub,
        ts,
        topic: topic_cstr,
        options: unsafe { *publisher_options },
        qos: unsafe { *qos_profile },
        graph: node_impl.graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = publisher_impl.graph.add_local_entity(ros_z::entity::Entity::Endpoint(entity)) {
        eprintln!("[rmw_z] rmw_create_publisher: Failed to add local entity to graph: {:?}", e);
    }

    // Box the publisher_impl first so the topic CString lives on the heap
    let publisher_impl_boxed = Box::new(publisher_impl);
    let topic_ptr = publisher_impl_boxed.topic.as_ptr();
    let publisher_impl_ptr = Box::into_raw(publisher_impl_boxed);

    let publisher = Box::new(rmw_publisher_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: publisher_impl_ptr as *mut _,
        topic_name: topic_ptr as *const _,
        options: unsafe { *publisher_options },
        can_loan_messages: false,
    });

    Box::into_raw(publisher)
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_publisher(
    node: *mut rmw_node_t,
    publisher: *mut rmw_publisher_t,
) -> rmw_ret_t {
    if node.is_null() || publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Remove local entity from graph
    if let Ok(publisher_impl) = publisher.borrow_data() {
        let entity = ros_z::entity::Entity::Endpoint(publisher_impl.entity.clone());
        if let Err(e) = publisher_impl.graph.remove_local_entity(&entity) {
            eprintln!("[rmw_z] rmw_destroy_publisher: Failed to remove local entity from graph: {:?}", e);
        }
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
    let ts = match unsafe { crate::type_support::MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return std::ptr::null_mut(),
    };

    let zsub_builder = node_impl
        .inner
        .create_sub::<crate::msg::RosMessage>(topic_str)
        .with_serdes::<crate::msg::RosSerdes>();
    let qos = crate::qos::rmw_qos_to_ros_z_qos(unsafe { &*qos_policies });
    let zsub_builder = zsub_builder.with_qos(qos);

    // Apply ignore_local_publications option if requested
    let ignore_local = unsafe { (*subscription_options).ignore_local_publications };
    let zsub_builder = zsub_builder.ignore_local_publications(ignore_local);

    let zsub = match zsub_builder.build() {
        Ok(zsub) => zsub,
        Err(_) => return std::ptr::null_mut(),
    };

    let entity = zsub.entity.clone();
    let topic_cstr = match std::ffi::CString::new(topic_str) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let subscription_impl = crate::pubsub::SubscriptionImpl {
        inner: zsub,
        ts,
        topic: topic_cstr.clone(),
        options: unsafe { *subscription_options },
        qos: unsafe { *qos_policies },
        callback: std::sync::Mutex::new(None),
        callback_user_data: std::sync::Mutex::new(std::ptr::null()),
        graph: node_impl.graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = subscription_impl.graph.add_local_entity(ros_z::entity::Entity::Endpoint(entity)) {
        eprintln!("[rmw_z] rmw_create_subscription: Failed to add local entity to graph: {:?}", e);
    }

    let subscription = Box::new(rmw_subscription_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        topic_name: subscription_impl.topic.as_ptr() as *const _,
        options: unsafe { *subscription_options },
        can_loan_messages: false,
        is_cft_enabled: false,
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

    // Remove local entity from graph
    if let Ok(subscription_impl) = subscription.borrow_data() {
        let entity = ros_z::entity::Entity::Endpoint(subscription_impl.entity.clone());
        if let Err(e) = subscription_impl.graph.remove_local_entity(&entity) {
            eprintln!("[rmw_z] rmw_destroy_subscription: Failed to remove local entity from graph: {:?}", e);
        }
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
pub extern "C" fn rmw_subscription_get_network_flow_endpoints(
    _subscription: *const rmw_subscription_t,
    _allocator: *const rcl_allocator_t,
    _network_flow_endpoints: *mut rmw_network_flow_endpoint_array_t,
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
    let qualified_service = match ros_z::topic_name::qualify_service_name(
        service_str,
        &node_impl.inner.entity.namespace,
        &node_impl.inner.entity.name,
    ) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!("Failed to qualify service name: {}", e);
            return std::ptr::null_mut();
        }
    };

    eprintln!("🔵 [CLIENT] Creating client for service: {}", qualified_service);

    let service_type_support =
        match unsafe { crate::type_support::ServiceTypeSupport::new(type_support) } {
            Ok(ts) => ts,
            Err(e) => {
                tracing::error!("Failed to create service type support: {}", e);
                return std::ptr::null_mut();
            }
        };

    let zclient_builder = node_impl
        .inner
        .create_client::<crate::msg::RosService>(&qualified_service)
        .with_type_info(service_type_support.get_type_info());
    let entity = zclient_builder.entity.clone();

    let zclient = match zclient_builder.build() {
        Ok(client) => {
            eprintln!("🔵 [CLIENT] Client created successfully");
            client
        },
        Err(e) => {
            tracing::error!("Failed to create client: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_cstr = match std::ffi::CString::new(qualified_service.clone()) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let client_impl = crate::service::ClientImpl {
        inner: zclient,
        service_name: qualified_service,
        options: rmw_client_options_t {
            qos: unsafe { *qos_policies },
        },
        request_ts: service_type_support,
        response_ts: service_type_support,
        callback: std::sync::Mutex::new(None),
        callback_user_data: std::sync::Mutex::new(std::ptr::null()),
        sequence_counter: std::sync::atomic::AtomicI64::new(1), // Start at 1 for ROS compatibility
        graph: node_impl.graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = client_impl.graph.add_local_entity(ros_z::entity::Entity::Endpoint(entity)) {
        eprintln!("[rmw_z] rmw_create_client: Failed to add local entity to graph: {:?}", e);
    }

    let client = Box::new(rmw_client_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: service_cstr.as_ptr() as *const _,
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

    // Remove local entity from graph
    if let Ok(client_impl) = client.borrow_data() {
        let entity = ros_z::entity::Entity::Endpoint(client_impl.entity.clone());
        if let Err(e) = client_impl.graph.remove_local_entity(&entity) {
            eprintln!("[rmw_z] rmw_destroy_client: Failed to remove local entity from graph: {:?}", e);
        }
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
    let qualified_service = match ros_z::topic_name::qualify_service_name(
        service_str,
        &node_impl.inner.entity.namespace,
        &node_impl.inner.entity.name,
    ) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!("Failed to qualify service name: {}", e);
            return std::ptr::null_mut();
        }
    };

    eprintln!("🟢 [SERVER] Creating server for service: {}", qualified_service);

    let service_type_support =
        match unsafe { crate::type_support::ServiceTypeSupport::new(type_support) } {
            Ok(ts) => ts,
            Err(e) => {
                tracing::error!("Failed to create service type support: {}", e);
                return std::ptr::null_mut();
            }
        };

    let zserver_builder = node_impl
        .inner
        .create_service::<crate::msg::RosService>(&qualified_service)
        .with_type_info(service_type_support.get_type_info());
    let entity = zserver_builder.entity.clone();

    let zserver = match zserver_builder.build() {
        Ok(server) => {
            eprintln!("🟢 [SERVER] Server created successfully");
            server
        },
        Err(e) => {
            tracing::error!("Failed to create service: {}", e);
            return std::ptr::null_mut();
        }
    };

    let service_name_cstr = match std::ffi::CString::new(qualified_service.clone()) {
        Ok(cstr) => cstr,
        Err(_) => return std::ptr::null_mut(),
    };

    let service_impl = crate::service::ServiceImpl {
        inner: zserver,
        service_name: service_name_cstr,
        request_ts: service_type_support,
        response_ts: service_type_support,
        qos: unsafe { *qos_profile },
        callback: std::sync::Mutex::new(None),
        callback_user_data: std::sync::Mutex::new(std::ptr::null()),
        graph: node_impl.graph.clone(),
        entity: entity.clone(),
    };

    // Add local entity to graph for immediate discovery
    if let Err(e) = service_impl.graph.add_local_entity(ros_z::entity::Entity::Endpoint(entity)) {
        eprintln!("[rmw_z] rmw_create_service: Failed to add local entity to graph: {:?}", e);
    }

    let service = Box::new(rmw_service_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        service_name: std::ptr::null(),
    });

    let service_ptr = Box::into_raw(service);
    service_ptr.assign_data(service_impl).unwrap_or(());

    // Update service_name pointer to point to the name stored in the impl
    unsafe {
        if let Ok(impl_ref) = service_ptr.borrow_data() {
            (*service_ptr).service_name = impl_ref.service_name.as_ptr();
        }
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

    // Remove local entity from graph
    if let Ok(service_impl) = service.borrow_data() {
        let entity = ros_z::entity::Entity::Endpoint(service_impl.entity.clone());
        if let Err(e) = service_impl.graph.remove_local_entity(&entity) {
            eprintln!("[rmw_z] rmw_destroy_service: Failed to remove local entity from graph: {:?}", e);
        }
    }

    drop(unsafe { Box::from_raw(service) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_service_get_service_name(
    service: *const rmw_service_t,
) -> *const std::os::raw::c_char {
    if service.is_null() {
        return std::ptr::null();
    }

    match service.borrow_data() {
        Ok(service_impl) => service_impl.service_name.as_ptr(),
        Err(_) => std::ptr::null(),
    }
}

// Graph queries
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get topic names and types from graph
    let topics_and_types = node_impl.graph.get_topic_names_and_types();

    // Group by topic name
    let mut topic_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (topic, type_name) in topics_and_types {
        topic_map.entry(topic).or_insert_with(Vec::new).push(type_name);
    }

    let topic_count = topic_map.len();

    // Initialize names_and_types
    unsafe {
        let ret = rmw_names_and_types_init(
            topic_names_and_types,
            topic_count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate the arrays
        let mut index = 0;
        for (topic_name, type_names) in topic_map.iter() {
            // Set topic name
            let topic_cstr = match std::ffi::CString::new(topic_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*topic_names_and_types).names.data.add(index).write(
                rcutils_strdup(topic_cstr.as_ptr(), *allocator),
            );
            if (*topic_names_and_types).names.data.add(index).read().is_null() {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Initialize types array for this topic
            let ret = rcutils_string_array_init(
                (*topic_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Populate type names
            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(topic_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }

            index += 1;
        }
    }

    RMW_RET_OK as _
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Get service names and types from graph
    let services_and_types = node_impl.graph.get_service_names_and_types();

    // Group by service name
    let mut service_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (service, type_name) in services_and_types {
        service_map.entry(service).or_insert_with(Vec::new).push(type_name);
    }

    let service_count = service_map.len();

    // Initialize names_and_types
    unsafe {
        let ret = rmw_names_and_types_init(
            service_names_and_types,
            service_count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate the arrays
        let mut index = 0;
        for (service_name, type_names) in service_map.iter() {
            // Set service name
            let service_cstr = match std::ffi::CString::new(service_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*service_names_and_types).names.data.add(index).write(
                rcutils_strdup(service_cstr.as_ptr(), *allocator),
            );
            if (*service_names_and_types).names.data.add(index).read().is_null() {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Initialize types array for this service
            let ret = rcutils_string_array_init(
                (*service_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Populate type names
            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(service_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }

            index += 1;
        }
    }

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

    // Get the graph guard condition from the node implementation
    match node.borrow_data() {
        Ok(node_impl) => node_impl.graph_guard_condition,
        Err(_) => std::ptr::null(),
    }
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Convert node name and namespace to strings
    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (target_node_ns.to_string(), target_node_name.to_string());

    // Get entities for this node
    let entities_and_types = node_impl.graph.get_names_and_types_by_node(
        node_key,
        ros_z::entity::EntityKind::Subscription,
    );

    // Group by entity name
    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_insert_with(Vec::new).push(type_name);
    }

    let entity_count = entity_map.len();

    // Initialize names_and_types
    unsafe {
        let ret = rmw_names_and_types_init(
            topic_names_and_types,
            entity_count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        // Populate the arrays
        let mut index = 0;
        for (entity_name, type_names) in entity_map.iter() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*topic_names_and_types).names.data.add(index).write(
                rcutils_strdup(entity_cstr.as_ptr(), *allocator),
            );
            if (*topic_names_and_types).names.data.add(index).read().is_null() {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Initialize types array
            let ret = rcutils_string_array_init(
                (*topic_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            // Populate type names
            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(topic_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }

            index += 1;
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_serialize(
    ros_message: *const c_void,
    type_support: *const rosidl_message_type_support_t,
    serialized_message: *mut rcl_serialized_message_t,
) -> rmw_ret_t {
    if ros_message.is_null() || type_support.is_null() || serialized_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let ts = match unsafe { MessageTypeSupport::new(type_support) } {
        Ok(ts) => ts,
        Err(_) => return RMW_RET_ERROR as _,
    };

    let serialized = unsafe { ts.serialize_message(ros_message) };

    // Allocate or resize the buffer in serialized_message
    unsafe {
        let msg = &mut *serialized_message;
        if msg.buffer_capacity < serialized.len() {
            // Need to reallocate
            if !msg.buffer.is_null() {
                // Drop the old buffer
                let _ = Vec::from_raw_parts(
                    msg.buffer,
                    msg.buffer_length,
                    msg.buffer_capacity,
                );
            }
            // Allocate new buffer
            let mut new_buffer = vec![0u8; serialized.len()];
            msg.buffer = new_buffer.as_mut_ptr();
            msg.buffer_capacity = new_buffer.len();
            std::mem::forget(new_buffer);
        }
        msg.buffer_length = serialized.len();
        std::ptr::copy_nonoverlapping(serialized.as_ptr(), msg.buffer, serialized.len());
    }

    RMW_RET_OK as _
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

    let res = unsafe { ts.deserialize_message(&data_vec, ros_message) };
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
    // Check if event_type is within valid range
    event_type <= ZenohEventType::LivelinessChanged as rmw_event_type_t
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
    if unsafe { (*publisher).implementation_identifier } != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
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
    if unsafe { (*client).implementation_identifier } != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
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
    if gid1_ref.implementation_identifier != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION as _;
    }

    if gid2.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let gid2_ref = unsafe { &*gid2 };
    if gid2_ref.implementation_identifier != crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _ {
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
    node: *const rmw_node_t,
    client: *const rmw_client_t,
    is_available: *mut bool,
) -> rmw_ret_t {
    if node.is_null() || client.is_null() || is_available.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Simple implementation: if the client exists, assume the server is available
    // A full implementation would use Zenoh's discovery mechanism
    unsafe {
        *is_available = true;
    }

    RMW_RET_OK as _
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
    subscription: *const rmw_subscription_t,
    dynamic_message: *mut rcldynamic_message_t,
    taken: *mut bool,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    // Call the _with_info version with a null message_info pointer
    rmw_take_dynamic_message_with_info(
        subscription,
        dynamic_message,
        taken,
        std::ptr::null_mut(),
        allocation,
    )
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
    #[allow(non_upper_case_globals)]
    match feature {
        rmw_feature_e_RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER => false,
        rmw_feature_e_RMW_FEATURE_MESSAGE_INFO_RECEPTION_SEQUENCE_NUMBER => false,
        rmw_feature_e_RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY => true,
        rmw_feature_e_RMW_MIDDLEWARE_CAN_TAKE_DYNAMIC_MESSAGE => false,
        _ => false,
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
    _allocator: *mut crate::ros::rcutils_allocator_t,
    _service_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    _clients_info: *mut crate::ros::rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_client_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || node_name.is_null() || node_namespace.is_null() || service_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (target_node_ns.to_string(), target_node_name.to_string());
    let entities_and_types = node_impl.graph.get_names_and_types_by_node(
        node_key,
        ros_z::entity::EntityKind::Client,
    );

    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_insert_with(Vec::new).push(type_name);
    }

    let entity_count = entity_map.len();

    unsafe {
        let ret = rmw_names_and_types_init(
            service_names_and_types,
            entity_count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        let mut index = 0;
        for (entity_name, type_names) in entity_map.iter() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*service_names_and_types).names.data.add(index).write(
                rcutils_strdup(entity_cstr.as_ptr(), *allocator),
            );
            if (*service_names_and_types).names.data.add(index).read().is_null() {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            let ret = rcutils_string_array_init(
                (*service_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(service_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }

            index += 1;
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_publisher_names_and_types_by_node(
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

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (target_node_ns.to_string(), target_node_name.to_string());
    let entities_and_types = node_impl.graph.get_names_and_types_by_node(
        node_key,
        ros_z::entity::EntityKind::Publisher,
    );

    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_insert_with(Vec::new).push(type_name);
    }

    let entity_count = entity_map.len();

    unsafe {
        let ret = rmw_names_and_types_init(
            topic_names_and_types,
            entity_count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        let mut index = 0;
        for (entity_name, type_names) in entity_map.iter() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*topic_names_and_types).names.data.add(index).write(
                rcutils_strdup(entity_cstr.as_ptr(), *allocator),
            );
            if (*topic_names_and_types).names.data.add(index).read().is_null() {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            let ret = rcutils_string_array_init(
                (*topic_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(topic_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(topic_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*topic_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(topic_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }

            index += 1;
        }
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_servers_info_by_service(
    _node: *const rmw_node_t,
    _allocator: *mut crate::ros::rcutils_allocator_t,
    _service_name: *const ::std::os::raw::c_char,
    _no_mangle: bool,
    _servers_info: *mut crate::ros::rmw_topic_endpoint_info_array_t,
) -> rmw_ret_t {
    RMW_RET_UNSUPPORTED as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_service_names_and_types_by_node(
    node: *const rmw_node_t,
    allocator: *const rcl_allocator_t,
    node_name: *const std::os::raw::c_char,
    node_namespace: *const std::os::raw::c_char,
    service_names_and_types: *mut rmw_names_and_types_t,
) -> rmw_ret_t {
    if node.is_null() || allocator.is_null() || node_name.is_null() || node_namespace.is_null() || service_names_and_types.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let target_node_name = match unsafe { std::ffi::CStr::from_ptr(node_name) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };
    let target_node_ns = match unsafe { std::ffi::CStr::from_ptr(node_namespace) }.to_str() {
        Ok(s) => s,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let node_key = (target_node_ns.to_string(), target_node_name.to_string());
    let entities_and_types = node_impl.graph.get_names_and_types_by_node(
        node_key,
        ros_z::entity::EntityKind::Service,
    );

    let mut entity_map: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for (name, type_name) in entities_and_types {
        entity_map.entry(name).or_insert_with(Vec::new).push(type_name);
    }

    let entity_count = entity_map.len();

    unsafe {
        let ret = rmw_names_and_types_init(
            service_names_and_types,
            entity_count,
            allocator as *mut _,
        );
        if ret != RMW_RET_OK as i32 {
            return RMW_RET_BAD_ALLOC as _;
        }

        let mut index = 0;
        for (entity_name, type_names) in entity_map.iter() {
            let entity_cstr = match std::ffi::CString::new(entity_name.as_str()) {
                Ok(s) => s,
                Err(_) => {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_ERROR as _;
                }
            };

            (*service_names_and_types).names.data.add(index).write(
                rcutils_strdup(entity_cstr.as_ptr(), *allocator),
            );
            if (*service_names_and_types).names.data.add(index).read().is_null() {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            let ret = rcutils_string_array_init(
                (*service_names_and_types).types.add(index),
                type_names.len(),
                allocator,
            );
            if ret != 0 {
                rmw_names_and_types_fini(service_names_and_types);
                return RMW_RET_BAD_ALLOC as _;
            }

            for (type_index, type_name) in type_names.iter().enumerate() {
                let type_cstr = match std::ffi::CString::new(type_name.as_str()) {
                    Ok(s) => s,
                    Err(_) => {
                        rmw_names_and_types_fini(service_names_and_types);
                        return RMW_RET_ERROR as _;
                    }
                };

                (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .write(rcutils_strdup(type_cstr.as_ptr(), *allocator));
                if (*(*service_names_and_types).types.add(index))
                    .data
                    .add(type_index)
                    .read()
                    .is_null()
                {
                    rmw_names_and_types_fini(service_names_and_types);
                    return RMW_RET_BAD_ALLOC as _;
                }
            }

            index += 1;
        }
    }

    RMW_RET_OK as _
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