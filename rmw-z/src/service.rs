use std::ffi::CString;
use std::sync::Arc;

use crate::traits::{BorrowImpl, OwnImpl, Waitable};
use crate::ros::*;
use crate::rmw_impl_has_data_ptr;

/// Client implementation for RMW
pub struct ClientImpl {
    pub inner: ros_z::service::ZClient<rcl_z::msg::RosService>,
    pub service_name: String,
    pub options: rmw_client_options_t,
}

/// Service implementation for RMW
pub struct ServiceImpl {
    pub inner: ros_z::service::ZServer<rcl_z::msg::RosService>,
    pub service_name: CString,
}

impl Waitable for ClientImpl {
    fn is_ready(&self) -> bool {
        !self.inner.rx.is_empty()
    }
}

impl Waitable for ServiceImpl {
    fn is_ready(&self) -> bool {
        !self.inner.rx.is_empty()
    }
}

rmw_impl_has_data_ptr!(rmw_client_t, rmw_client_impl_t, ClientImpl);
rmw_impl_has_data_ptr!(rmw_service_t, rmw_service_impl_t, ServiceImpl);