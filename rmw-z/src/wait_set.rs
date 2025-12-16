use crate::traits::{Waitable, BorrowData, OwnData};
use crate::ros::*;

/// Wait set implementation for RMW
pub struct WaitSetImpl {
    pub subscriptions: Vec<*mut rmw_subscription_impl_t>,
    pub guard_conditions: Vec<*mut rmw_guard_condition_impl_t>,
    pub services: Vec<*mut rmw_service_impl_t>,
    pub clients: Vec<*mut rmw_client_impl_t>,
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
        use std::time::{Duration, Instant};

        // Calculate timeout duration
        let timeout_duration = if timeout.sec == u64::MAX {
            Duration::from_secs(365 * 24 * 3600) // Essentially infinite
        } else {
            Duration::from_secs(timeout.sec) + Duration::from_nanos(timeout.nsec)
        };

        let start = Instant::now();
        let poll_interval = Duration::from_millis(10);

        // Poll until something is ready or timeout expires
        loop {
            // Check if any waitable is ready
            for (idx, sub_impl_ptr) in self.subscriptions.iter().enumerate() {
                if sub_impl_ptr.is_null() {
                    continue;
                }
                unsafe {
                    // rmw_subscription_impl_t is an opaque type, but we know it's actually SubscriptionImpl
                    let sub_impl = &*((*sub_impl_ptr) as *const _ as *const crate::pubsub::SubscriptionImpl);
                    if sub_impl.is_ready() {
                        return true;
                    }
                }
            }
            for gc_impl_ptr in &self.guard_conditions {
                if gc_impl_ptr.is_null() {
                    continue;
                }
                unsafe {
                    let gc_impl = &*(*gc_impl_ptr as *const _ as *const crate::guard_condition::GuardConditionImpl);
                    if gc_impl.is_ready() {
                        return true;
                    }
                }
            }
            for srv_impl_ptr in &self.services {
                if srv_impl_ptr.is_null() {
                    continue;
                }
                unsafe {
                    let srv_impl = &*(*srv_impl_ptr as *const _ as *const crate::service::ServiceImpl);
                    if srv_impl.is_ready() {
                        return true;
                    }
                }
            }
            for cli_impl_ptr in &self.clients {
                if cli_impl_ptr.is_null() {
                    continue;
                }
                unsafe {
                    let cli_impl = &*(*cli_impl_ptr as *const _ as *const crate::service::ClientImpl);
                    if cli_impl.is_ready() {
                        return true;
                    }
                }
            }

            // Check if timeout has expired
            if start.elapsed() >= timeout_duration {
                return false;
            }

            // Sleep briefly before polling again
            std::thread::sleep(poll_interval);
        }
    }
}

rmw_impl_has_data_ptr!(rmw_wait_set_t, rmw_wait_set_impl_t, WaitSetImpl);

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
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        guard_conditions: std::ptr::null_mut(),
        data: std::ptr::null_mut(),
    });

    let wait_set_ptr = Box::into_raw(wait_set);
    unsafe {
        (*wait_set_ptr).data = Box::into_raw(Box::new(wait_set_impl)) as *mut _;
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
    _events: *mut rmw_events_t,
    wait_set: *mut rmw_wait_set_t,
    wait_timeout: *const rmw_time_t,
) -> rmw_ret_t {
    if wait_set.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    let wait_set_impl = match wait_set.borrow_mut_data() {
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
            // subscribers is *mut *mut c_void, so subscribers.add(i) gives *mut *mut c_void
            // We dereference once to get *mut c_void, which points to rmw_subscription_impl_t
            let sub_impl = unsafe { *sub_array.subscribers.add(i) as *mut rmw_subscription_impl_t };
            if !sub_impl.is_null() {
                wait_set_impl.subscriptions.push(sub_impl);
            }
        }
    }

    // Add guard conditions
    if !guard_conditions.is_null() {
        let gc_array = unsafe { &*guard_conditions };
        for i in 0..gc_array.guard_condition_count {
            let gc_impl = unsafe { *gc_array.guard_conditions.add(i) as *mut rmw_guard_condition_impl_t };
            if !gc_impl.is_null() {
                wait_set_impl.guard_conditions.push(gc_impl);
            }
        }
    }

    // Add services
    if !services.is_null() {
        let srv_array = unsafe { &*services };
        for i in 0..srv_array.service_count {
            let srv_impl = unsafe { *srv_array.services.add(i) as *mut rmw_service_impl_t };
            if !srv_impl.is_null() {
                wait_set_impl.services.push(srv_impl);
            }
        }
    }

    // Add clients
    if !clients.is_null() {
        let cli_array = unsafe { &*clients };
        for i in 0..cli_array.client_count {
            let cli_impl = unsafe { *cli_array.clients.add(i) as *mut rmw_client_impl_t };
            if !cli_impl.is_null() {
                wait_set_impl.clients.push(cli_impl);
            }
        }
    }

    // Wait for ready entities
    let timeout = if wait_timeout.is_null() {
        rmw_time_t { sec: u64::MAX, nsec: 0 }
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
                let sub_impl_ptr = unsafe { *sub_array.subscribers.add(i) as *mut rmw_subscription_impl_t };
                if !sub_impl_ptr.is_null() {
                    unsafe {
                        let sub_impl = &*(sub_impl_ptr as *const _ as *const crate::pubsub::SubscriptionImpl);
                        if sub_impl.is_ready() {
                            *sub_array.subscribers.add(ready_count) = sub_impl_ptr as *mut _;
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
                let srv_impl_ptr = unsafe { *srv_array.services.add(i) as *mut rmw_service_impl_t };
                if !srv_impl_ptr.is_null() {
                    unsafe {
                        let srv_impl = &*(srv_impl_ptr as *const _ as *const crate::service::ServiceImpl);
                        if srv_impl.is_ready() {
                            *srv_array.services.add(ready_count) = srv_impl_ptr as *mut _;
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
                let cli_impl_ptr = unsafe { *cli_array.clients.add(i) as *mut rmw_client_impl_t };
                if !cli_impl_ptr.is_null() {
                    unsafe {
                        let cli_impl = &*(cli_impl_ptr as *const _ as *const crate::service::ClientImpl);
                        if cli_impl.is_ready() {
                            *cli_array.clients.add(ready_count) = cli_impl_ptr as *mut _;
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

        // Similar for guard conditions
        if !guard_conditions.is_null() {
            let gc_array = unsafe { &mut *guard_conditions };
            let mut ready_count = 0;
            for i in 0..gc_array.guard_condition_count {
                let gc_impl_ptr = unsafe { *gc_array.guard_conditions.add(i) as *mut rmw_guard_condition_impl_t };
                if !gc_impl_ptr.is_null() {
                    unsafe {
                        let gc_impl = &*(gc_impl_ptr as *const _ as *const crate::guard_condition::GuardConditionImpl);
                        if gc_impl.is_ready() {
                            *gc_array.guard_conditions.add(ready_count) = gc_impl_ptr as *mut _;
                            ready_count += 1;
                        }
                    }
                }
            }
            for i in ready_count..gc_array.guard_condition_count {
                unsafe {
                    *gc_array.guard_conditions.add(i) = std::ptr::null_mut();
                }
            }
        }

        RMW_RET_OK as _
    } else {
        RMW_RET_TIMEOUT as _
    }
}