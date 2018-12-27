execute_process(COMMAND "/home/yus/Documents/_tmp/_ros_tmp/build/example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yus/Documents/_tmp/_ros_tmp/build/example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
