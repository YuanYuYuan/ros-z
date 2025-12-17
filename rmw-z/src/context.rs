use std::sync::{Arc, Mutex};
use std::collections::HashMap;


use crate::ros::*;
use zenoh::{Session, Wait};
use crate::rmw_impl_has_impl_ptr;
use crate::node::NodeImpl;
use crate::traits::*;
use crate::utils::Notifier;

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
        // Create Zenoh config with timestamping enabled
        let mut config = zenoh::Config::default();

        // Enable timestamping for AdvancedPublisher support
        config.insert_json5("timestamping/enabled", "true")
            .map_err(|e| format!("Failed to enable timestamping in config: {}", e))?;

        // Disable multicast scouting to avoid conflicts
        config.insert_json5("scouting/multicast/enabled", "false")
            .map_err(|e| format!("Failed to configure scouting: {}", e))?;

        let session = zenoh::open(config).wait()
            .map_err(|e| format!("Failed to create Zenoh session: {}", e))?;
        let counter = Arc::new(ros_z::context::GlobalCounter::default());
        let graph = ros_z::graph::Graph::new(&session, domain_id)
            .map_err(|e| format!("Failed to create graph: {}", e))?;

        // Set up graph guard condition trigger callback
        {
            let event_manager = graph.event_manager.clone();
            let trigger_callback = Box::new(|gc: *mut std::ffi::c_void| {
                let gc_ptr = gc as *mut rmw_guard_condition_t;
                if !gc_ptr.is_null() {
                    let _ = crate::guard_condition::rmw_trigger_guard_condition(gc_ptr);
                }
            });
            event_manager.set_guard_condition_trigger(trigger_callback);
        }

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

    unsafe {
        // Check if dst is already initialized
        if !(*dst).impl_.is_null() {
            return RMW_RET_INVALID_ARGUMENT as _;
        }

        // Copy all fields from src to dst
        (*dst).instance_id = (*src).instance_id;
        (*dst).implementation_identifier = (*src).implementation_identifier;
        (*dst).domain_id = (*src).domain_id;
        (*dst).security_options = (*src).security_options;
        (*dst).localhost_only = (*src).localhost_only;
        (*dst).discovery_options = (*src).discovery_options;
        (*dst).allocator = (*src).allocator;

        // Copy enclave string if it exists
        if !(*src).enclave.is_null() {
            let enclave_cstr = std::ffi::CStr::from_ptr((*src).enclave);
            let enclave_bytes = enclave_cstr.to_bytes_with_nul();
            let allocator = &(*src).allocator;

            if let Some(allocate) = allocator.allocate {
                let new_enclave = allocate(
                    enclave_bytes.len(),
                    allocator.state,
                ) as *mut std::os::raw::c_char;

                if new_enclave.is_null() {
                    return RMW_RET_BAD_ALLOC as _;
                }

                std::ptr::copy_nonoverlapping(
                    enclave_bytes.as_ptr() as *const std::os::raw::c_char,
                    new_enclave,
                    enclave_bytes.len(),
                );
                (*dst).enclave = new_enclave;
            } else {
                // If no allocator, we can't copy the enclave string
                return RMW_RET_INVALID_ARGUMENT as _;
            }
        } else {
            (*dst).enclave = std::ptr::null_mut();
        }

        // For rmw_z, we don't have implementation-specific data, so impl_ is null
        (*dst).impl_ = std::ptr::null_mut();
    }

    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_init_options_fini(init_options: *mut rmw_init_options_t) -> rmw_ret_t {
    if init_options.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        // Free enclave string if it was allocated
        if !(*init_options).enclave.is_null() {
            let allocator = &(*init_options).allocator;
            if let Some(deallocate) = allocator.deallocate {
                deallocate(
                    (*init_options).enclave as *mut _,
                    allocator.state,
                );
                (*init_options).enclave = std::ptr::null_mut();
            }
        }

        // Free impl if it exists (rmw_z doesn't use it, but be safe)
        if !(*init_options).impl_.is_null() {
            // For now, rmw_z doesn't allocate impl_, so nothing to do
            (*init_options).impl_ = std::ptr::null_mut();
        }
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
            unsafe {
                (*context).instance_id = 1; // TODO: proper instance ID
                (*context).actual_domain_id = domain_id; // Set the actual domain ID
            }
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

    if unsafe { (*context).impl_.is_null() } {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Mark context as shutdown
    match context.borrow_impl() {
        Ok(context_impl) => {
            *context_impl.is_shutdown.lock().unwrap() = true;
            RMW_RET_OK as _
        }
        Err(_) => RMW_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_context_fini(context: *mut rmw_context_t) -> rmw_ret_t {
    if context.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // First try to borrow the implementation to validate it
    let context_impl = match context.borrow_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Check if context has been shut down
    if !*context_impl.is_shutdown.lock().unwrap() {
        return RMW_RET_INVALID_ARGUMENT as _; // Must call rmw_shutdown before rmw_context_fini
    }

    // Check if there are still nodes attached to this context
    if !context_impl.nodes.lock().unwrap().is_empty() {
        return RMW_RET_INVALID_ARGUMENT as _; // Cannot finalize context with active nodes
    }

    // Additional validation: check if context has been properly initialized
    // by verifying the implementation_identifier
    if unsafe { (*context).implementation_identifier }.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Own and drop the implementation
    match context.own_impl() {
        Ok(_) => RMW_RET_OK as _,
        Err(_) => RMW_RET_ERROR as _,
    }
}

