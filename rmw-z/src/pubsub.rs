use std::ffi::CString;
use std::sync::Arc;

use crate::node::NodeImpl;
use crate::qos::rmw_qos_to_ros_z_qos;
use crate::traits::{BorrowImpl, OwnImpl, Waitable};
use crate::ros::*;
use zenoh::{Result, Session, sample::Sample};
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
        let ros_msg = rcl_z::msg::RosMessage::new(msg, self.inner.entity.ts.clone());
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
    pub fn take(&self, ros_message: *mut crate::c_void, taken: *mut bool) -> Result<(), zenoh::Error> {
        unsafe { *taken = false; }
        if let Ok(sample) = self.inner.queue.try_recv() {
            // Deserialize the sample payload into ros_message using ts
            // Assume the payload is CDR serialized
            let payload = sample.payload();
            let bytes = payload.to_bytes();
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