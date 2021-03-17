include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test_pnp test_pnp.cc)
target_link_libraries(test_pnp pnpsolver)