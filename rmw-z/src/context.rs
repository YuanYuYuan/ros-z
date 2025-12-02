use std::ffi::CString;
use std::sync::Arc;

use crate::ros::*;
use zenoh::{Session, Wait};
use crate::rmw_impl_has_impl_ptr;
use crate::node::NodeImpl;

/// Context implementation for RMW
pub struct ContextImpl {
    pub session: Arc<Session>,
    pub domain_id: usize,
    pub counter: Arc<ros_z::context::GlobalCounter>,
    pub graph: Arc<ros_z::graph::Graph>,
}

impl ContextImpl {
    pub fn new(domain_id: usize) -> Result<Self, zenoh::Error> {
        // For now, create a default Zenoh session
        // In a real implementation, this would be configured based on domain_id
        let session = Arc::new(zenoh::open(zenoh::Config::default()).wait()?);
        let counter = Arc::new(ros_z::context::GlobalCounter::default());
        let graph = Arc::new(ros_z::graph::Graph::new(session.clone()));
        Ok(Self { session, domain_id, counter, graph })
    }

    pub fn new_node(
        &self,
        name: *const ::std::os::raw::c_char,
        namespace_: *const ::std::os::raw::c_char,
        _context: *mut rmw_context_t,
        _options: *const rcl_node_options_t,
    ) -> Result<NodeImpl, Box<dyn std::error::Error>> {
        let name_str = unsafe { std::ffi::CStr::from_ptr(name) }.to_str()?;
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(namespace_) }.to_str()?;

        let node_impl = NodeImpl::new(
            self.session.clone(),
            self.counter.clone(),
            self.graph.clone(),
            name_str,
            namespace_str,
        )?;

        Ok(node_impl)
    }
}

rmw_impl_has_impl_ptr!(rmw_context_t, rmw_context_impl_t, ContextImpl);