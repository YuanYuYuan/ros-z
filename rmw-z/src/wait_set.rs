use crate::traits::{Waitable, BorrowData, OwnData, BorrowImpl, OwnImpl};
use crate::ros::*;
use crate::rmw_impl_has_impl_ptr;

/// Wait set implementation for RMW
pub struct WaitSetImpl {
    pub subscriptions: Vec<*mut rmw_subscription_t>,
    pub guard_conditions: Vec<*mut rmw_guard_condition_t>,
    pub services: Vec<*mut rmw_service_t>,
    pub clients: Vec<*mut rmw_client_t>,
    pub events: Vec<*mut rmw_event_t>,
}

impl WaitSetImpl {
    pub fn new(max_conditions: usize) -> Self {
        Self {
            subscriptions: Vec::with_capacity(max_conditions),
            guard_conditions: Vec::with_capacity(max_conditions),
            services: Vec::with_capacity(max_conditions),
            clients: Vec::with_capacity(max_conditions),
            events: Vec::with_capacity(max_conditions),
        }
    }

    pub fn wait(&self, timeout: &rmw_time_t) -> bool {
        // Simple implementation - check if any waitable is ready
        for sub in &self.subscriptions {
            if let Ok(sub_impl) = (*sub).borrow_data() {
                if sub_impl.is_ready() {
                    return true;
                }
            }
        }
        for gc in &self.guard_conditions {
            // Guard conditions are always ready if triggered
            // For simplicity, assume not ready
        }
        for srv in &self.services {
            if let Ok(srv_impl) = (*srv).borrow_data() {
                if srv_impl.is_ready() {
                    return true;
                }
            }
        }
        for cli in &self.clients {
            if let Ok(cli_impl) = (*cli).borrow_data() {
                if cli_impl.is_ready() {
                    return true;
                }
            }
        }
        false
    }
}

rmw_impl_has_impl_ptr!(rmw_wait_set_t, rmw_wait_set_impl_t, WaitSetImpl);

// RMW Wait Set Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_wait_set(
    context: *mut rmw_context_t,
    max_conditions: usize,
) -> *mut rmw_wait_set_t {
    if context.is_null() {
        return std::ptr::null_mut();
    }

    let wait_set_impl = WaitSetImpl::new(max_conditions);
    let wait_set = Box::new(rmw_wait_set_t {
        impl_: std::ptr::null_mut(),
    });

    let wait_set_ptr = Box::into_raw(wait_set);
    unsafe {
        wait_set_ptr.assign_impl(wait_set_impl).unwrap_or(());
    }

    wait_set_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_wait_set(wait_set: *mut rmw_wait_set_t) -> rmw_ret_t {
    if wait_set.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(wait_set) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_wait(
    subscriptions: *mut rmw_subscriptions_t,
    guard_conditions: *mut rmw_guard_conditions_t,
    services: *mut rmw_services_t,
    clients: *mut rmw_clients_t,
    events: *mut rmw_events_t,
    wait_set: *mut rmw_wait_set_t,
    wait_timeout: *const rmw_time_t,
) -> rmw_ret_t {
    if wait_set.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let wait_set_impl = match wait_set.borrow_mut_impl() {
        Ok(impl_) => impl_,
        Err(_) => return RMW_RET_INVALID_ARGUMENT as _,
    };

    // Clear the wait set
    wait_set_impl.subscriptions.clear();
    wait_set_impl.guard_conditions.clear();
    wait_set_impl.services.clear();
    wait_set_impl.clients.clear();
    wait_set_impl.events.clear();

    // Add subscriptions to wait set
    if !subscriptions.is_null() {
        let sub_array = unsafe { &*subscriptions };
        for i in 0..sub_array.subscriber_count {
            let sub = unsafe { *sub_array.subscribers.add(i) };
            if !sub.is_null() {
                wait_set_impl.subscriptions.push(sub);
            }
        }
    }

    // Add guard conditions
    if !guard_conditions.is_null() {
        let gc_array = unsafe { &*guard_conditions };
        for i in 0..gc_array.guard_condition_count {
            let gc = unsafe { *gc_array.guard_conditions.add(i) };
            if !gc.is_null() {
                wait_set_impl.guard_conditions.push(gc);
            }
        }
    }

    // Add services
    if !services.is_null() {
        let srv_array = unsafe { &*services };
        for i in 0..srv_array.service_count {
            let srv = unsafe { *srv_array.services.add(i) };
            if !srv.is_null() {
                wait_set_impl.services.push(srv);
            }
        }
    }

    // Add clients
    if !clients.is_null() {
        let cli_array = unsafe { &*clients };
        for i in 0..cli_array.client_count {
            let cli = unsafe { *cli_array.clients.add(i) };
            if !cli.is_null() {
                wait_set_impl.clients.push(cli);
            }
        }
    }

    // Wait for ready entities
    let timeout = if wait_timeout.is_null() {
        rmw_time_t { sec: -1, nsec: 0 }
    } else {
        unsafe { *wait_timeout }
    };

    let ready = wait_set_impl.wait(&timeout);

    if ready {
        // Update arrays to only contain ready entities
        if !subscriptions.is_null() {
            let sub_array = unsafe { &mut *subscriptions };
            let mut ready_count = 0;
            for i in 0..sub_array.subscriber_count {
                let sub = unsafe { *sub_array.subscribers.add(i) };
                if !sub.is_null() {
                    if let Ok(sub_impl) = sub.borrow_data() {
                        if sub_impl.is_ready() {
                            unsafe {
                                *sub_array.subscribers.add(ready_count) = sub;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            // Null out the rest
            for i in ready_count..sub_array.subscriber_count {
                unsafe {
                    *sub_array.subscribers.add(i) = std::ptr::null_mut();
                }
            }
        }

        // Similar for services
        if !services.is_null() {
            let srv_array = unsafe { &mut *services };
            let mut ready_count = 0;
            for i in 0..srv_array.service_count {
                let srv = unsafe { *srv_array.services.add(i) };
                if !srv.is_null() {
                    if let Ok(srv_impl) = srv.borrow_data() {
                        if srv_impl.is_ready() {
                            unsafe {
                                *srv_array.services.add(ready_count) = srv;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            for i in ready_count..srv_array.service_count {
                unsafe {
                    *srv_array.services.add(i) = std::ptr::null_mut();
                }
            }
        }

        // Similar for clients
        if !clients.is_null() {
            let cli_array = unsafe { &mut *clients };
            let mut ready_count = 0;
            for i in 0..cli_array.client_count {
                let cli = unsafe { *cli_array.clients.add(i) };
                if !cli.is_null() {
                    if let Ok(cli_impl) = cli.borrow_data() {
                        if cli_impl.is_ready() {
                            unsafe {
                                *cli_array.clients.add(ready_count) = cli;
                            }
                            ready_count += 1;
                        }
                    }
                }
            }
            for i in ready_count..cli_array.client_count {
                unsafe {
                    *cli_array.clients.add(i) = std::ptr::null_mut();
                }
            }
        }

        RMW_RET_OK as _
    } else {
        RMW_RET_TIMEOUT as _
    }
}