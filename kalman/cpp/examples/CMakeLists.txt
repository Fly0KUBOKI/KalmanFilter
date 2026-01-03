# examples/CMakeLists.txt
add_executable(main_eskf main_eskf.cpp)

# Include both Lib and Inc for compatibility
target_include_directories(main_eskf PRIVATE
    ${CMAKE_SOURCE_DIR}/Lib
    ${CMAKE_SOURCE_DIR}/../Inc
)

target_link_libraries(main_eskf PRIVATE kalman_lib)

install(TARGETS main_eskf RUNTIME DESTINATION bin)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_interface.cpp)
    add_executable(test_interface test_interface.cpp)
    target_include_directories(test_interface PRIVATE
        ${CMAKE_SOURCE_DIR}/Lib
        ${CMAKE_SOURCE_DIR}/../Inc
    )
    target_link_libraries(test_interface PRIVATE kalman_lib)
    install(TARGETS test_interface RUNTIME DESTINATION bin)
endif()
