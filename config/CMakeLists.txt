cmake_minimum_required(VERSION 2.8.3)
project(devine_config)

find_package(catkin REQUIRED)

catkin_python_setup()

# workaround for devel space
configure_file(src/devine_config/static/Parameters.ini ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/static/Parameters.ini COPYONLY)
