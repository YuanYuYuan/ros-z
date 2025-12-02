use std::ffi::CString;

use crate::traits::{Waitable, BorrowData};
use crate::ros::*;
use zenoh::{Result, sample::Sample};
use crate::rmw_impl_has_data_ptr;
use rcl_z::ros::{rmw_message_sequence_t, rmw_message_info_sequence_t};

/// Publisher implementation for RMW
pub struct PublisherImpl {
    pub inner: ros_z::pubsub::ZPub<rcl_z::msg::RosMessage, rcl_z::msg::RosSerdes>,
    pub ts: rcl_z::type_support::MessageTypeSupport,
    pub topic: CString,
    pub options: rmw_publisher_options_t,
    pub qos: rmw_qos_profile_t,
}

impl PublisherImpl {
    pub fn publish(&self, msg: *const ::std::os::raw::c_void) -> Result<()> {
        let ros_msg = rcl_z::msg::RosMessage::new(msg as *const rcl_z::c_void, self.ts.clone());
        self.inner.publish(&ros_msg)
    }

    pub fn publish_serialized_message(&self, msg: &[u8]) -> Result<()> {
        self.inner.publish_serialized_message(msg)
    }
}

/// Subscription implementation for RMW
pub struct SubscriptionImpl {
    pub inner: ros_z::pubsub::ZSub<rcl_z::msg::RosMessage, Sample, rcl_z::msg::RosSerdes>,
    pub ts: rcl_z::type_support::MessageTypeSupport,
    pub topic: CString,
    pub options: rmw_subscription_options_t,
    pub qos: rmw_qos_profile_t,
}

impl SubscriptionImpl {
    pub fn take(&self, ros_message: *mut std::os::raw::c_void, taken: *mut bool) -> Result<()> {
        unsafe { *taken = false; }
        if let Ok(sample) = self.inner.queue.try_recv() {
            // Deserialize the sample payload into ros_message using ts
            // Assume the payload is CDR serialized
            let payload = sample.payload();
            let bytes = payload.to_bytes().to_vec();
            unsafe { self.ts.deserialize_message(&bytes, ros_message as *mut _) };
            unsafe { *taken = true; }
        }
        Ok(())
    }

    pub fn take_with_info(
        &self,
        ros_message: *mut std::os::raw::c_void,
        message_info: *mut rmw_message_info_t,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe { *taken = false; }
        if let Ok(sample) = self.inner.queue.try_recv() {
            // Deserialize the sample payload into ros_message using ts
            let payload = sample.payload();
            let bytes = payload.to_bytes().to_vec();
            unsafe { self.ts.deserialize_message(&bytes, ros_message as *mut _) };

            // Fill in message_info
            if !message_info.is_null() {
                unsafe {
                    // Initialize message_info with default values
                    (*message_info).source_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 }; // TODO: Extract from Zenoh sample timestamp
                    (*message_info).received_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 }; // TODO: Get current time
                    (*message_info).publication_sequence_number = 0;
                    (*message_info).reception_sequence_number = 0;

                    // Set publisher GID to zeros for now
                    // TODO: Extract proper GID from Zenoh sample
                    for i in 0..24 {
                        (*message_info).publisher_gid[i] = 0;
                    }
                    (*message_info).from_intra_process = false;
                }
            }

            unsafe { *taken = true; }
        }
        Ok(())
    }

    pub fn take_serialized(
        &self,
        serialized_message: *mut rcl_serialized_message_t,
        message_info: *mut rmw_message_info_t,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe { *taken = false; }
        if let Ok(sample) = self.inner.queue.try_recv() {
            let payload = sample.payload();
            let bytes = payload.to_bytes();

            unsafe {
                // Check if there's enough capacity
                if (*serialized_message).buffer_capacity < bytes.len() {
                    // Reallocate buffer if needed
                    if !(*serialized_message).buffer.is_null() {
                        // TODO: Use proper allocator from RMW context
                        let _ = Vec::from_raw_parts(
                            (*serialized_message).buffer,
                            (*serialized_message).buffer_length,
                            (*serialized_message).buffer_capacity,
                        );
                    }
                    let mut new_buffer = vec![0u8; bytes.len()];
                    (*serialized_message).buffer = new_buffer.as_mut_ptr();
                    (*serialized_message).buffer_capacity = new_buffer.len();
                    std::mem::forget(new_buffer);
                }

                // Copy bytes to buffer
                std::ptr::copy_nonoverlapping(
                    bytes.as_ptr(),
                    (*serialized_message).buffer,
                    bytes.len(),
                );
                (*serialized_message).buffer_length = bytes.len();
            }

            // Fill in message_info if provided
            if !message_info.is_null() {
                unsafe {
                    (*message_info).source_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 };
                    (*message_info).received_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 };
                    (*message_info).publication_sequence_number = 0;
                    (*message_info).reception_sequence_number = 0;
                    for i in 0..24 {
                        (*message_info).publisher_gid[i] = 0;
                    }
                    (*message_info).from_intra_process = false;
                }
            }

            unsafe { *taken = true; }
        }
        Ok(())
    }
}

impl Waitable for SubscriptionImpl {
    fn is_ready(&self) -> bool {
        !self.inner.queue.is_empty()
    }
}

rmw_impl_has_data_ptr!(rmw_publisher_t, rmw_publisher_impl_t, PublisherImpl);
rmw_impl_has_data_ptr!(rmw_subscription_t, rmw_subscription_impl_t, SubscriptionImpl);

// RMW Publisher Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_publish_serialized_message(
    publisher: *const rmw_publisher_t,
    serialized_message: *const rcl_serialized_message_t,
    allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    if publisher.is_null() || serialized_message.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match unsafe { publisher.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    let msg_slice = unsafe {
        std::slice::from_raw_parts((*serialized_message).buffer, (*serialized_message).buffer_length)
    };

    match publisher_impl.publish_serialized_message(msg_slice) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to publish serialized message: {}", e);
            RMW_RET_ERROR as _
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publish_loaned_message(
    publisher: *const rmw_publisher_t,
    ros_message: *mut ::std::os::raw::c_void,
    allocation: *mut rmw_publisher_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_count_matched_subscriptions(
    publisher: *const rmw_publisher_t,
    subscription_count: *mut usize,
) -> rmw_ret_t {
    if publisher.is_null() || subscription_count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // TODO: Implement actual matching logic with graph data
    // For now, return 0 as a placeholder
    unsafe {
        *subscription_count = 0;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_get_actual_qos(
    publisher: *const rmw_publisher_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if publisher.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let publisher_impl = match unsafe { publisher.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    unsafe {
        *qos = publisher_impl.qos;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_assert_liveliness(publisher: *const rmw_publisher_t) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_wait_for_all_acked(
    publisher: *const rmw_publisher_t,
    wait_timeout: rmw_time_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_publisher_get_network_flow_endpoints(
    publisher: *const rmw_publisher_t,
    allocator: *const rcl_allocator_t,
    endpoints: *mut rmw_network_flow_endpoint_array_t,
) -> rmw_ret_t {
    todo!()
}

// RMW Subscription Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_with_info(
    subscription: *const rmw_subscription_t,
    ros_message: *mut ::std::os::raw::c_void,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || ros_message.is_null() || taken.is_null() || message_info.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match unsafe { subscription.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take_with_info(ros_message, message_info, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_sequence(
    subscription: *const rmw_subscription_t,
    count: usize,
    message_sequence: *mut rmw_message_sequence_t,
    message_info_sequence: *mut rmw_message_info_sequence_t,
    taken: *mut usize,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || message_sequence.is_null() || message_info_sequence.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match unsafe { subscription.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    if count == 0 {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        if count > (*message_sequence).capacity || count > (*message_info_sequence).capacity {
            return RMW_RET_INVALID_ARGUMENT as _;
        }

        *taken = 0;
        while *taken < count {
            let mut one_taken = false;
            let msg_ptr = *(*message_sequence).data.add(*taken);
            let info_ptr = (*message_info_sequence).data.add(*taken) as *mut crate::ros::rmw_message_info_t;

            match subscription_impl.take_with_info(msg_ptr, info_ptr, &mut one_taken) {
                Ok(_) => {
                    if !one_taken {
                        break;
                    }
                    *taken += 1;
                }
                Err(_) => break,
            }
        }

        (*message_sequence).size = *taken;
        (*message_info_sequence).size = *taken;
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_serialized_message(
    subscription: *const rmw_subscription_t,
    serialized_message: *mut rcl_serialized_message_t,
    taken: *mut bool,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || serialized_message.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match unsafe { subscription.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take_serialized(serialized_message, std::ptr::null_mut(), taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_serialized_message_with_info(
    subscription: *const rmw_subscription_t,
    serialized_message: *mut rcl_serialized_message_t,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    if subscription.is_null() || serialized_message.is_null() || taken.is_null() || message_info.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match unsafe { subscription.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match subscription_impl.take_serialized(serialized_message, message_info, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_loaned_message(
    subscription: *const rmw_subscription_t,
    loaned_message: *mut *mut ::std::os::raw::c_void,
    taken: *mut bool,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_loaned_message_with_info(
    subscription: *const rmw_subscription_t,
    loaned_message: *mut *mut ::std::os::raw::c_void,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_count_matched_publishers(
    subscription: *const rmw_subscription_t,
    publisher_count: *mut usize,
) -> rmw_ret_t {
    if subscription.is_null() || publisher_count.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // TODO: Implement actual matching logic with graph data
    // For now, return 0 as a placeholder
    unsafe {
        *publisher_count = 0;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_actual_qos(
    subscription: *const rmw_subscription_t,
    qos: *mut rmw_qos_profile_t,
) -> rmw_ret_t {
    if subscription.is_null() || qos.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let subscription_impl = match unsafe { subscription.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    unsafe {
        *qos = subscription_impl.qos;
    }
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_set_content_filter(
    subscription: *const rmw_subscription_t,
    content_filter: *const rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    todo!()
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_subscription_get_content_filter(
    subscription: *const rmw_subscription_t,
    allocator: *const rcl_allocator_t,
    content_filter: *mut rmw_subscription_content_filter_options_t,
) -> rmw_ret_t {
    todo!()
}