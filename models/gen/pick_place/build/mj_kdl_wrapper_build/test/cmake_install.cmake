# Install script for directory: /home/batsy/work/ms/src/mj_kdl_wrapper/test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/_deps/googletest-build/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_init")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_init")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_dual_arm")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_dual_arm")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_table_scene")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_table_scene")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_mjcf_load")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_load")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_mjcf_pos_ctrl")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pos_ctrl")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_mjcf_vel_ctrl")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_vel_ctrl")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_mjcf_trq_ctrl")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_trq_ctrl")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick"
         RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/test_mjcf_pick")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick"
         OLD_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build:/home/batsy/work/ms/install/orocos_kdl/lib:"
         NEW_RPATH "/opt/mujoco-3.8.0/lib:/home/batsy/work/ms/install/orocos_kdl/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_mjcf_pick")
    endif()
  endif()
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/batsy/work/ms/src/bdd_collab_bhv_cpp/models/gen/pick_place/build/mj_kdl_wrapper_build/test/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
