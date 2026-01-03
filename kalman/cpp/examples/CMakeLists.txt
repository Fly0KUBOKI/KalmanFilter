# examples/CMakeLists.txt
add_executable(main_eskf main_eskf.cpp)

# Include both Lib and Inc for compatibility
target_include_directories(main_eskf PRIVATE
    ${CMAKE_SOURCE_DIR}/Lib
    ${CMAKE_SOURCE_DIR}/../Inc
)

target_link_libraries(main_eskf PRIVATE kalman_lib)

install(TARGETS main_eskf RUNTIME DESTINATION bin)
