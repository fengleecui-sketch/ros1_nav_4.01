find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)
include_directories( include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS} )
link_directories(${csm_LIBRARY_DIRS})