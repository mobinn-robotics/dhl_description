cmake_minimum_required(VERSION 3.0.2)
project(dhl_description)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()

# NOT catkin_install_python(): in this workspace CMake resolved
# PYTHON_EXECUTABLE to anaconda's python 3.12, and the devel-space wrapper it
# generates hard-codes that interpreter in its shebang. A Noetic node started
# under 3.12 hangs before it registers with the master. install(PROGRAMS)
# leaves the script's own "#!/usr/bin/env python3" in charge instead.
install(PROGRAMS scripts/fake_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

find_package(roslaunch)

foreach(dir launch meshes rviz urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()
