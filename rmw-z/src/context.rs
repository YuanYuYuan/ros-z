use std::sync::{Arc, Mutex};
use std::collections::HashMap;


use crate::ros::*;
use zenoh::{Session, Wait};
use crate::rmw_impl_has_impl_ptr;
use crate::node::NodeImpl;
use crate::traits::*;
use rcl_z::utils::Notifier;

/// Context implementation for RMW
pub struct ContextImpl {
    pub session: Arc<Session>,
    pub domain_id: usize,
    pub enclave: String,
    pub counter: Arc<ros_z::context::GlobalCounter>,
    pub graph: Arc<ros_z::graph::Graph>,
    pub next_entity_id: Arc<Mutex<usize>>,
    pub is_shutdown: Arc<Mutex<bool>>,
    pub nodes: Arc<Mutex<HashMap<*const rmw_node_t, Arc<NodeImpl>>>>,
    pub notifier: Arc<Notifier>,
}

impl ContextImpl {
    pub fn new(domain_id: usize, enclave: String) -> Result<Self, String> {
        // For now, create a default Zenoh session
        // In a real implementation, this would be configured based on domain_id
        let session = zenoh::open(zenoh::Config::default()).wait()
            .map_err(|e| format!("Failed to create Zenoh session: {}", e))?;
        let counter = Arc::new(ros_z::context::GlobalCounter::default());
        let graph = ros_z::graph::Graph::new(&session, domain_id)
            .map_err(|e| format!("Failed to create graph: {}", e))?;
        Ok(Self {
            session: Arc::new(session),
            domain_id,
            enclave,
            counter,
            graph: Arc::new(graph),
            next_entity_id: Arc::new(Mutex::new(1)),
            is_shutdown: Arc::new(Mutex::new(false)),
            #[allow(clippy::arc_with_non_send_sync)]
            nodes: Arc::new(Mutex::new(HashMap::new())),
            notifier: Arc::new(Notifier::default()),
        })
    }

    pub fn new_node(
        &self,
        name: *const ::std::os::raw::c_char,
        namespace_: *const ::std::os::raw::c_char,
        _context: *mut rmw_context_t,
        _options: *const rcl_node_options_t,
    ) -> Result<NodeImpl, String> {
        let name_str = unsafe { std::ffi::CStr::from_ptr(name) }.to_str()
            .map_err(|e| format!("Invalid name string: {}", e))?;
        let namespace_str = unsafe { std::ffi::CStr::from_ptr(namespace_) }.to_str()
            .map_err(|e| format!("Invalid namespace string: {}", e))?;

        let node_impl = NodeImpl::new(
            self.session.clone(),
            self.counter.clone(),
            self.graph.clone(),
            name_str,
            namespace_str,
        ).map_err(|e| format!("Failed to create node: {}", e))?;

        Ok(node_impl)
    }

    pub fn share_notifier(&self) -> Arc<Notifier> {
        self.notifier.clone()
    }
}

rmw_impl_has_impl_ptr!(rmw_context_t, rmw_context_impl_t, ContextImpl);

// RMW Context Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_options_init(
    init_options: *mut rmw_init_options_t,
    _domain_id: usize,
    _allocator: rcl_allocator_t,
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
    let domain_id = unsafe { (*options).domain_id };
    let enclave = "/".to_string(); // Default enclave
    let context_impl = match ContextImpl::new(domain_id, enclave) {
        Ok(impl_) => impl_,
        Err(e) => {
            tracing::error!("Failed to create context: {}", e);
            return RMW_RET_ERROR as _;
        }
    };

    // Assign implementation
    match context.assign_impl(context_impl) {
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
    match context.own_impl() {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

