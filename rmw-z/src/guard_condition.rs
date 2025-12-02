use std::sync::atomic::{AtomicBool, Ordering};
use crate::ros::*;
use crate::rmw_impl_has_data_ptr;
use crate::traits::*;

/// Guard condition implementation for RMW
pub struct GuardConditionImpl {
    pub triggered: AtomicBool,
}

impl GuardConditionImpl {
    pub fn new() -> Self {
        Self {
            triggered: AtomicBool::new(false),
        }
    }

    pub fn trigger(&mut self) {
        self.triggered.store(true, Ordering::SeqCst);
    }

    pub fn is_triggered(&self) -> bool {
        self.triggered.load(Ordering::SeqCst)
    }
}

rmw_impl_has_data_ptr!(rmw_guard_condition_t, rmw_guard_condition_impl_t, GuardConditionImpl);

// RMW Guard Condition Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_create_guard_condition(
    context: *mut rmw_context_t,
) -> *mut rmw_guard_condition_t {
    if context.is_null() {
        return std::ptr::null_mut();
    }

    let gc_impl = GuardConditionImpl::new();
    let gc = Box::new(rmw_guard_condition_t {
        implementation_identifier: crate::RMW_ZENOH_IDENTIFIER.as_ptr() as *const _,
        data: std::ptr::null_mut(),
        context,
    });

    let gc_ptr = Box::into_raw(gc);
    unsafe {
        gc_ptr.assign_data(gc_impl).unwrap_or(());
    }

    gc_ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_destroy_guard_condition(
    guard_condition: *mut rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    drop(unsafe { Box::from_raw(guard_condition) });
    RMW_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rmw_trigger_guard_condition(
    guard_condition: *const rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }

    if let Ok(mut gc_impl) = (guard_condition as *mut rmw_guard_condition_t).borrow_mut_data() {
        gc_impl.trigger();
    }

    RMW_RET_OK as _
}