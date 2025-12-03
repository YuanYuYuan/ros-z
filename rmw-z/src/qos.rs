use crate::ros::*;

/// Convert RMW QoS profile to ros-z QoS profile
pub fn rmw_qos_to_ros_z_qos(qos: &rmw_qos_profile_t) -> ros_z::qos::QosProfile {
    use ros_z::qos::*;

    let history = match qos.history {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST => {
            QosHistory::KeepLast(qos.depth)
        }
        RMW_QOS_POLICY_HISTORY_KEEP_ALL => QosHistory::KeepAll,
        _ => QosHistory::KeepLast(10), // Default
    };

    let reliability = match qos.reliability {
        RMW_QOS_POLICY_RELIABILITY_RELIABLE => QosReliability::Reliable,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT => QosReliability::BestEffort,
        _ => QosReliability::Reliable, // Default
    };

    let durability = match qos.durability {
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL => QosDurability::TransientLocal,
        RMW_QOS_POLICY_DURABILITY_VOLATILE => QosDurability::Volatile,
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
    // Check for specific incompatibility
    if publisher_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        && subscription_profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE {
        unsafe { *compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_WARNING; }
        // For simplicity, don't append to reason
        RMW_RET_OK as _
    } else {
        unsafe { *compatibility = rmw_qos_compatibility_type_t::RMW_QOS_COMPATIBILITY_OK; }
        RMW_RET_OK as _
    }
}