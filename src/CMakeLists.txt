# Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
# its contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

if(IS_MSVC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
elseif(IS_GNU OR IS_CLANG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON) # default use dynamic

set(LIBRARY_NAME pnpsolver)

include_directories(${PROJECT_SOURCE_DIR}/src)

aux_source_directory(base base_src)

aux_source_directory(interface interface_src)

aux_source_directory(estimators estimators_src)

aux_source_directory(optim optim_src)

aux_source_directory(util util_src)

add_library(${LIBRARY_NAME}
    ${base_src}
    ${interface_src}
    ${estimators_src}
    ${optim_src}
    ${util_src})
set_target_properties(${LIBRARY_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(PnpSolver::pnpsolver ALIAS ${LIBRARY_NAME})

if(TARGET ceres)
    target_link_libraries(${LIBRARY_NAME} PUBLIC ceres)
else()
    target_link_libraries(${LIBRARY_NAME} PUBLIC Ceres::ceres)
endif()

if(TARGET Eigen3::Eigen)
    target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen)
else()
    target_include_directories(${LIBRARY_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
endif()

target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interface>
    $<INSTALL_INTERFACE:include/>
)

install(TARGETS ${LIBRARY_NAME} DESTINATION lib EXPORT ${LIBRARY_NAME}-targets)
install(FILES ${PROJECT_SOURCE_DIR}/src/interface/pnp_solver.h DESTINATION include)

install(EXPORT ${LIBRARY_NAME}-targets DESTINATION lib/cmake NAMESPACE PnpSolver::)

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/PnpSolverConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/PnpSolverConfig.cmake"
    INSTALL_DESTINATION lib/cmake/
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/PnpSolverConfig.cmake"
    DESTINATION lib/cmake
)
