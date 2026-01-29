# Export rmw_zenoh_rs library
# This file is included when find_package(rmw_zenoh_rs) is called

if(NOT TARGET rmw_zenoh_rs::rmw_zenoh_rs)
  add_library(rmw_zenoh_rs::rmw_zenoh_rs SHARED IMPORTED)
  set_target_properties(rmw_zenoh_rs::rmw_zenoh_rs PROPERTIES
    IMPORTED_LOCATION "${rmw_zenoh_rs_DIR}/../../../lib/librmw_zenoh_rs.so"
  )
endif()

# Also provide the library without namespace for ament_target_dependencies
set(rmw_zenoh_rs_LIBRARIES "${rmw_zenoh_rs_DIR}/../../../lib/librmw_zenoh_rs.so")
