use std::ffi::CString;

use crate::traits::{Waitable, BorrowData, OwnData};
use crate::ros::*;
use crate::c_void;
use crate::rmw_impl_has_data_ptr;
use zenoh::Result;

/// Client implementation for RMW
pub struct ClientImpl {
    pub inner: ros_z::service::ZClient<rcl_z::msg::RosService>,
    pub service_name: String,
    pub options: rmw_client_options_t,
    pub request_ts: rcl_z::type_support::ServiceTypeSupport,
    pub response_ts: rcl_z::type_support::ServiceTypeSupport,
}

impl ClientImpl {
    pub fn send_request(&self, request: *const c_void, sequence_id: *mut i64) -> Result<()> {
        // Create RosMessage from the raw pointer using request MessageTypeSupport
        let req = rcl_z::msg::RosMessage::new(request as *const rcl_z::c_void, self.request_ts.request);

        // Use rcl_send_request to send and get sequence number
        let sn = self.inner.rcl_send_request(&req, || {})?;
        unsafe { *sequence_id = sn; }
        Ok(())
    }

    pub fn take_response(
        &self,
        request_header: *mut rmw_service_info_t,
        response: *mut c_void,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe { *taken = false; }

        // Try to receive a response
        if let Ok(sample) = self.inner.rx.try_recv() {
            let payload = sample.payload();
            let bytes = payload.to_bytes().to_vec();

            // Deserialize response using response MessageTypeSupport
            unsafe { self.response_ts.response.deserialize_message(&bytes, response as *mut _); }

            // Fill request_header
            if !request_header.is_null() {
                unsafe {
                    // Extract sequence number from attachment if available
                    (*request_header).request_id.sequence_number = 0; // TODO: extract from sample
                    for i in 0..16 {
                        (*request_header).request_id.writer_guid[i] = 0; // TODO: extract GID
                    }
                    (*request_header).source_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 };
                    (*request_header).received_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 };
                }
            }

            unsafe { *taken = true; }
        }
        Ok(())
    }
}

/// Service implementation for RMW
pub struct ServiceImpl {
    pub inner: ros_z::service::ZServer<rcl_z::msg::RosService>,
    pub service_name: CString,
    pub request_ts: rcl_z::type_support::ServiceTypeSupport,
    pub response_ts: rcl_z::type_support::ServiceTypeSupport,
}

impl ServiceImpl {
    pub fn take_request(
        &mut self,
        request_header: *mut rmw_service_info_t,
        request: *mut c_void,
        taken: *mut bool,
    ) -> Result<()> {
        unsafe { *taken = false; }

        // Try to receive a request from the raw receiver
        if let Ok(query) = self.inner.rx.try_recv() {
            // Get the payload bytes
            let bytes = if let Some(payload) = query.payload() {
                payload.to_bytes().to_vec()
            } else {
                return Ok(());
            };

            // TODO: Extract proper QueryKey from query attachments
            // For now, use a placeholder key
            let key = ros_z::service::QueryKey {
                gid: [0u8; 16], // Placeholder GID
                sn: 0i64, // Placeholder sequence number
            };

            // Store the query for later response
            self.inner.map.insert(key.clone(), query);

            // Deserialize into the provided request buffer using request MessageTypeSupport
            unsafe { self.request_ts.request.deserialize_message(&bytes, request as *mut _); }

            // Fill request_header with sequence info
            if !request_header.is_null() {
                unsafe {
                    (*request_header).request_id.sequence_number = key.sn as i64;
                    // Copy GID from key
                    for (i, &byte) in key.gid.iter().enumerate() {
                        if i < 16 {
                            (*request_header).request_id.writer_guid[i] = byte;
                        }
                    }
                    (*request_header).source_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 };
                    (*request_header).received_timestamp = crate::ros::rmw_time_t { sec: 0, nsec: 0 };
                }
            }

            unsafe { *taken = true; }
        }
        Ok(())
    }

    pub fn send_response(
        &mut self,
        request_header: *const rmw_request_id_t,
        response: *const c_void,
    ) -> Result<()> {
        // Extract QueryKey from request_header
        let key = unsafe {
            let mut gid = [0u8; 16];
            for i in 0..16 {
                gid[i] = (*request_header).writer_guid[i];
            }
            ros_z::service::QueryKey {
                gid,
                sn: (*request_header).sequence_number,
            }
        };

        // Create RosMessage Response from the raw pointer using response MessageTypeSupport
        let resp = rcl_z::msg::RosMessage::new(response as *const rcl_z::c_void, self.response_ts.response);

        // Send response
        self.inner.send_response(&resp, &key)?;
        Ok(())
    }
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

// RMW Service Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_take_request(
    service: *const rmw_service_t,
    request_header: *mut rmw_service_info_t,
    ros_request: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    if service.is_null() || request_header.is_null() || ros_request.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let service_impl = match unsafe { (service as *mut rmw_service_t).borrow_mut_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match service_impl.take_request(request_header, ros_request, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to take request: {}", e);
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

    let service_impl = match unsafe { (service as *mut rmw_service_t).borrow_mut_data() } {
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

// RMW Client Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_send_request(
    client: *const rmw_client_t,
    ros_request: *const c_void,
    sequence_id: *mut i64,
) -> rmw_ret_t {
    if client.is_null() || ros_request.is_null() || sequence_id.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match unsafe { client.borrow_data() } {
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
pub extern "C" fn rmw_take_response(
    client: *const rmw_client_t,
    request_header: *mut rmw_service_info_t,
    ros_response: *mut c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    if client.is_null() || request_header.is_null() || ros_response.is_null() || taken.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let client_impl = match unsafe { client.borrow_data() } {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    match client_impl.take_response(request_header, ros_response, taken) {
        Ok(_) => RMW_RET_OK as _,
        Err(e) => {
            tracing::error!("Failed to take response: {}", e);
            RMW_RET_ERROR as _
        }
    }
}