list(REMOVE_DUPLICATES _AGNOCAST_COMPONENTS_PACKAGE_RESOURCE_INDICES)
foreach(resource_index ${_AGNOCAST_COMPONENTS_PACKAGE_RESOURCE_INDICES})
  ament_index_register_resource(
    ${resource_index} CONTENT "${_AGNOCAST_COMPONENTS_${resource_index}__NODES}")
endforeach()
