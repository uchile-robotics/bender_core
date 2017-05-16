cmake_minimum_required(VERSION 2.8.3)
project(uchile_rqt_batteries)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# TODO: que instalar?
install(PROGRAMS
  scripts/uchile_rqt_batteries
  #scripts/test_plugin.py
  #scripts/batt_arms_pub.py
  #scripts/Motor_state_test.py
  scripts/notebook_battery_pub.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config images)
 install(DIRECTORY ${dir}/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
