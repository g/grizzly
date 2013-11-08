
add_executable(${PROJECT_NAME}_motion_generator motion_generator)
target_link_libraries(${PROJECT_NAME}_motion_generator ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_generator
                      PROPERTIES OUTPUT_NAME motion_generator PREFIX "")

add_executable(${PROJECT_NAME}_motion_safety motion_safety encoders_monitor)
target_link_libraries(${PROJECT_NAME}_motion_safety ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_safety
                      PROPERTIES OUTPUT_NAME motion_safety PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_motion_generator
  ${PROJECT_NAME}_motion_safety
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

