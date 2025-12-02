use std::sync::atomic::{AtomicBool, Ordering};
use crate::ros::*;
use crate::rmw_impl_has_data_ptr;

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