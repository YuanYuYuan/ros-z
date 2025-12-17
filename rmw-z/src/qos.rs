use crate::ros::*;

/// Check if publisher and subscriber QoS profiles are compatible for matching
/// Returns true if they can be matched, false otherwise
pub fn qos_profiles_are_compatible(
    pub_qos: &rmw_qos_profile_t,
    sub_qos: &rmw_qos_profile_t,
) -> bool {
    let mut compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_OK;
    let ret = rmw_qos_profile_check_compatible(
        *pub_qos,
        *sub_qos,
        &mut compatibility as *mut _,
        std::ptr::null_mut(),
        0,
    );

    eprintln!("[QoS Debug] pub_reliability={}, sub_reliability={}, compat={:?}",
        pub_qos.reliability, sub_qos.reliability,
        if matches!(compatibility, rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_OK) { "OK" }
        else if matches!(compatibility, rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_WARNING) { "WARN" }
        else { "ERROR" });

    if ret != (RMW_RET_OK as rmw_ret_t) {
        return false;
    }

    // Only OK compatibility means they can be matched
    // WARNING or ERROR means they should not be counted as matched
    matches!(compatibility, rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_OK)
}

/// Convert ros-z QoS profile to RMW QoS profile
pub fn ros_z_qos_to_rmw_qos(qos: &ros_z::qos::QosProfile) -> rmw_qos_profile_t {
    use ros_z::qos::*;

    #[allow(non_upper_case_globals)]
    let (history, depth) = match &qos.history {
        QosHistory::KeepLast(n) => (rmw_qos_history_policy_e_RMW_QOS_POLICY_HISTORY_KEEP_LAST, *n),
        QosHistory::KeepAll => (rmw_qos_history_policy_e_RMW_QOS_POLICY_HISTORY_KEEP_ALL, 0),
    };

    #[allow(non_upper_case_globals)]
    let reliability = match qos.reliability {
        QosReliability::Reliable => rmw_qos_reliability_policy_e_RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        QosReliability::BestEffort => rmw_qos_reliability_policy_e_RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    };

    #[allow(non_upper_case_globals)]
    let durability = match qos.durability {
        QosDurability::TransientLocal => rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        QosDurability::Volatile => rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_VOLATILE,
    };

    rmw_qos_profile_t {
        history,
        depth,
        reliability,
        durability,
        deadline: rmw_time_t { sec: 0, nsec: 0 },
        lifespan: rmw_time_t { sec: 0, nsec: 0 },
        liveliness: rmw_qos_liveliness_policy_e_RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

/// Convert RMW QoS profile to ros-z QoS profile
pub fn rmw_qos_to_ros_z_qos(qos: &rmw_qos_profile_t) -> ros_z::qos::QosProfile {
    use ros_z::qos::*;

    #[allow(non_upper_case_globals)]
    let history = match qos.history {
        rmw_qos_history_policy_e_RMW_QOS_POLICY_HISTORY_KEEP_LAST => {
            QosHistory::KeepLast(qos.depth)
        }
        rmw_qos_history_policy_e_RMW_QOS_POLICY_HISTORY_KEEP_ALL => QosHistory::KeepAll,
        _ => QosHistory::KeepLast(10), // Default
    };

    #[allow(non_upper_case_globals)]
    let reliability = match qos.reliability {
        rmw_qos_reliability_policy_e_RMW_QOS_POLICY_RELIABILITY_RELIABLE => QosReliability::Reliable,
        rmw_qos_reliability_policy_e_RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT => QosReliability::BestEffort,
        _ => QosReliability::Reliable, // Default
    };

    #[allow(non_upper_case_globals)]
    let durability = match qos.durability {
        rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL => QosDurability::TransientLocal,
        rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_VOLATILE => QosDurability::Volatile,
        _ => QosDurability::Volatile, // Default
    };

    QosProfile {
        history,
        reliability,
        durability,
        deadline: ros_z::qos::Duration::default(),
        lifespan: ros_z::qos::Duration::default(),
        liveliness: ros_z::qos::QosLiveliness::default(),
        liveliness_lease_duration: ros_z::qos::Duration::default(),
    }
}

// RMW QoS Functions
#[unsafe(no_mangle)]
pub extern "C" fn rmw_qos_profile_check_compatible(
    publisher_profile: rmw_qos_profile_t,
    subscription_profile: rmw_qos_profile_t,
    compatibility: *mut rmw_qos_compatibility_type_t,
    reason: *mut ::std::os::raw::c_char,
    reason_size: usize,
) -> rmw_ret_t {
    if compatibility.is_null() {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    if !reason.is_null() && reason_size == 0 {
        return RMW_RET_INVALID_ARGUMENT as _;
    }
    // Initialize reason buffer
    if !reason.is_null() && reason_size > 0 {
        unsafe { *reason = 0; }
    }
    // Check reliability compatibility
    // A RELIABLE subscriber cannot be matched with a BEST_EFFORT publisher
    if publisher_profile.reliability == rmw_qos_reliability_policy_e_RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        && subscription_profile.reliability == rmw_qos_reliability_policy_e_RMW_QOS_POLICY_RELIABILITY_RELIABLE {
        unsafe { *compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_ERROR; }
        return RMW_RET_OK as _;
    }

    // Check durability compatibility
    // A TRANSIENT_LOCAL subscriber cannot be matched with a VOLATILE publisher
    if publisher_profile.durability == rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_VOLATILE
        && subscription_profile.durability == rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL {
        unsafe { *compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_ERROR; }
        return RMW_RET_OK as _;
    }

    // Check for warning conditions (opposite direction)
    if publisher_profile.durability == rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        && subscription_profile.durability == rmw_qos_durability_policy_e_RMW_QOS_POLICY_DURABILITY_VOLATILE {
        unsafe { *compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_WARNING; }
        return RMW_RET_OK as _;
    }

    unsafe { *compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_OK; }
    RMW_RET_OK as _
}