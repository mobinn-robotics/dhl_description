cmake_minimum_required(VERSION 3.8)
project(dhl_description)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/fake_driver.py
  DESTINATION lib/${PROJECT_NAME})

foreach(dir launch meshes rviz urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()
