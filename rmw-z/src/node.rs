use std::ffi::CString;
use std::sync::Arc;

use crate::ros::*;
use zenoh::Session;
use crate::rmw_impl_has_data_ptr;
use ros_z::Builder;

/// Node implementation for RMW
pub struct NodeImpl {
    pub session: Arc<Session>,
    pub counter: Arc<ros_z::context::GlobalCounter>,
    pub graph: Arc<ros_z::graph::Graph>,
    pub inner: ros_z::node::ZNode,
    pub name: CString,
    pub namespace: CString,
    pub fq_name: CString,
}

impl NodeImpl {
    pub fn new(session: Arc<Session>, counter: Arc<ros_z::context::GlobalCounter>, graph: Arc<ros_z::graph::Graph>, name: &str, namespace: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let name_cstr = CString::new(name)?;
        let namespace_cstr = CString::new(namespace)?;
        let fq_name = if namespace.is_empty() || namespace == "/" {
            format!("/{}", name)
        } else {
            format!("{}/{}", namespace, name)
        };
        let fq_name_cstr = CString::new(fq_name)?;

        let inner = ros_z::node::ZNodeBuilder {
            domain_id: 0, // TODO: use actual domain_id
            name: name.to_string(),
            namespace: namespace.to_string(),
            session: session.clone(),
            counter: counter.clone(),
            graph: graph.clone(),
        }.build().map_err(|e| Box::new(e.to_string()) as Box<dyn std::error::Error>)?;

        Ok(Self {
            session,
            counter,
            graph,
            inner,
            name: name_cstr,
            namespace: namespace_cstr,
            fq_name: fq_name_cstr,
        })
    }
}

rmw_impl_has_data_ptr!(rmw_node_t, rmw_node_impl_t, NodeImpl);