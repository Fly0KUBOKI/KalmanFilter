# Lib/CMakeLists.txt
# Collect all module sources under Lib/*/src and build a static library

file(GLOB_RECURSE KALMAN_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*/src/*.cxx
)

add_library(kalman_lib STATIC ${KALMAN_SOURCES})

# Add include dirs: Lib root, each module's inc, and top-level Inc for backward compatibility
target_include_directories(kalman_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/../Inc
)

# add each module's inc directory if present
file(GLOB LIB_MODULE_INC LIST_DIRECTORIES ON ${CMAKE_CURRENT_SOURCE_DIR}/*/inc)
foreach(dir ${LIB_MODULE_INC})
    target_include_directories(kalman_lib PUBLIC ${dir})
endforeach()

# compile options (allow override)
target_compile_features(kalman_lib PUBLIC cxx_std_11)

# expose library for downstream
set_target_properties(kalman_lib PROPERTIES EXPORT_NAME kalman)

install(TARGETS kalman_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/kalman/Lib
    FILES_MATCHING PATTERN "*.hpp"
)
