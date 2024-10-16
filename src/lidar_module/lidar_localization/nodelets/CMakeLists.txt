add_library(${PROJECT_NAME}_nodelets preprocess.cpp global_map.cpp localization.cpp)

add_dependencies(${PROJECT_NAME}_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelets ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_nodelets
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})