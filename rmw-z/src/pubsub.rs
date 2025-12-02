use std::ffi::CString;

use crate::traits::Waitable;
use crate::ros::*;
use zenoh::{Result, sample::Sample};
use crate::rmw_impl_has_data_ptr;

/// Publisher implementation for RMW
pub struct PublisherImpl {
    pub inner: ros_z::pubsub::ZPub<rcl_z::msg::RosMessage, rcl_z::msg::RosSerdes>,
    pub ts: rcl_z::type_support::MessageTypeSupport,
    pub topic: CString,
    pub options: rmw_publisher_options_t,
}

impl PublisherImpl {
    pub fn publish(&self, msg: *const crate::c_void) -> Result<()> {
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
}

impl SubscriptionImpl {
    pub fn take(&self, ros_message: *mut crate::c_void, taken: *mut bool) -> Result<()> {
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
}

impl Waitable for SubscriptionImpl {
    fn is_ready(&self) -> bool {
        !self.inner.queue.is_empty()
    }
}

rmw_impl_has_data_ptr!(rmw_publisher_t, rmw_publisher_impl_t, PublisherImpl);
rmw_impl_has_data_ptr!(rmw_subscription_t, rmw_subscription_impl_t, SubscriptionImpl);