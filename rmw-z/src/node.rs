use std::ffi::CString;
use std::sync::Arc;

use crate::ros::*;
use zenoh::Session;
use crate::rmw_impl_has_data_ptr;
use ros_z::Builder;
use crate::traits::*;

/// Node implementation for RMW
pub struct NodeImpl {
    pub session: Arc<Session>,
    pub counter: Arc<ros_z::context::GlobalCounter>,
    pub graph: Arc<ros_z::graph::Graph>,
    pub inner: ros_z::node::ZNode,
    pub name: CString,
    pub namespace: CString,
    pub fq_name: CString,
    pub graph_guard_condition: *mut rmw_guard_condition_t,
}

impl NodeImpl {
    pub fn new(session: Arc<Session>, counter: Arc<ros_z::context::GlobalCounter>, graph: Arc<ros_z::graph::Graph>, name: &str, namespace: &str) -> Result<Self, String> {
        let name_cstr = CString::new(name)
            .map_err(|e| format!("Invalid name string: {}", e))?;
        let namespace_cstr = CString::new(namespace)
            .map_err(|e| format!("Invalid namespace string: {}", e))?;
        let fq_name = if namespace.is_empty() || namespace == "/" {
            format!("/{}", name)
        } else {
            format!("{}/{}", namespace, name)
        };
        let fq_name_cstr = CString::new(fq_name)
            .map_err(|e| format!("Invalid fully qualified name: {}", e))?;

        let inner = ros_z::node::ZNodeBuilder {
            domain_id: 0, // TODO: use actual domain_id
            name: name.to_string(),
            namespace: namespace.to_string(),
            session: session.clone(),
            counter: counter.clone(),
            graph: graph.clone(),
        }.build().map_err(|e| format!("Failed to build node: {}", e))?;

        Ok(Self {
            session,
            counter,
            graph,
            inner,
            name: name_cstr,
            namespace: namespace_cstr,
            fq_name: fq_name_cstr,
            graph_guard_condition: std::ptr::null_mut(),
        })
    }
}

rmw_impl_has_data_ptr!(rmw_node_t, rmw_node_impl_t, NodeImpl);

// RMW Node Functions
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

    let _name_str = unsafe { std::ffi::CStr::from_ptr(name) }
        .to_str()
        .unwrap_or("");
    let _namespace_str = unsafe { std::ffi::CStr::from_ptr(namespace_) }
        .to_str()
        .unwrap_or("");

    let mut node_impl = match context_impl.new_node(name, namespace_, context, std::ptr::null()) {
        Ok(impl_) => impl_,
        Err(_) => return std::ptr::null_mut(),
    };

    // Create a graph guard condition for this node
    let graph_guard_condition = crate::guard_condition::rmw_create_guard_condition(context);
    if graph_guard_condition.is_null() {
        return std::ptr::null_mut();
    }
    node_impl.graph_guard_condition = graph_guard_condition;

    // Get pointers to the owned CStrings in node_impl
    let name_ptr = node_impl.name.as_ptr();
    let namespace_ptr = node_impl.namespace.as_ptr();

    let node = Box::new(rmw_node_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        name: name_ptr as *const _,
        namespace_: namespace_ptr as *const _,
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

    // Destroy the graph guard condition first
    if let Ok(node_impl) = node.borrow_data() {
        if !node_impl.graph_guard_condition.is_null() {
            crate::guard_condition::rmw_destroy_guard_condition(node_impl.graph_guard_condition);
        }
    }

    // Drop the implementation data
    let _ = node.own_data();

    drop(unsafe { Box::from_raw(node) });
    RMW_RET_OK as _
}



#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_node_names(
    node: *const rmw_node_t,
    node_names: *mut rcutils_string_array_t,
    node_namespaces: *mut rcutils_string_array_t,
) -> rmw_ret_t {
    if node.is_null() || node_names.is_null() || node_namespaces.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let node_impl = match node.borrow_data() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Query graph for all nodes
    let _nodes = node_impl.graph.get_node_names();
    // For now, just return OK with empty lists
    // Full implementation would populate the string arrays
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_get_node_names_with_enclaves(
    node: *const rmw_node_t,
    node_names: *mut rcutils_string_array_t,
    node_namespaces: *mut rcutils_string_array_t,
    enclaves: *mut rcutils_string_array_t,
) -> rmw_ret_t {
    if node.is_null() || node_names.is_null() || node_namespaces.is_null() || enclaves.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    // Delegate to rmw_get_node_names for now
    rmw_get_node_names(node, node_names, node_namespaces)
}