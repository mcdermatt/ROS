# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(loam_ouster_CONFIG_INCLUDED)
  return()
endif()
set(loam_ouster_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(loam_ouster_SOURCE_PREFIX /home/derm/ROS/src/LOAM_3D_LIDARs)
  set(loam_ouster_DEVEL_PREFIX /home/derm/ROS/devel)
  set(loam_ouster_INSTALL_PREFIX "")
  set(loam_ouster_PREFIX ${loam_ouster_DEVEL_PREFIX})
else()
  set(loam_ouster_SOURCE_PREFIX "")
  set(loam_ouster_DEVEL_PREFIX "")
  set(loam_ouster_INSTALL_PREFIX /home/derm/ROS/install)
  set(loam_ouster_PREFIX ${loam_ouster_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'loam_ouster' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(loam_ouster_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include;/usr/include/eigen3;/pc-pcl-1.13.1/build/include/ " STREQUAL " ")
  set(loam_ouster_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/include/eigen3;/pc-pcl-1.13.1/build/include/")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Ed Venator <evenator@swri.org>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${loam_ouster_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'loam_ouster' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'loam_ouster' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
    endif()
    _list_append_unique(loam_ouster_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "loam;/usr/local/lib/libpcl_common.so;/usr/local/lib/libpcl_kdtree.so;/usr/local/lib/libpcl_octree.so;/usr/local/lib/libpcl_search.so;/usr/local/lib/libpcl_sample_consensus.so;/usr/local/lib/libpcl_filters.so;/usr/local/lib/libpcl_io.so;/usr/local/lib/libpcl_features.so;/usr/local/lib/libpcl_ml.so;/usr/local/lib/libpcl_segmentation.so;/usr/local/lib/libpcl_visualization.so;/usr/local/lib/libpcl_surface.so;/usr/local/lib/libpcl_registration.so;/usr/local/lib/libpcl_keypoints.so;/usr/local/lib/libpcl_tracking.so;/usr/local/lib/libpcl_recognition.so;/usr/local/lib/libpcl_stereo.so;/usr/local/lib/libpcl_outofcore.so;/usr/local/lib/libpcl_people.so;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;/usr/lib/x86_64-linux-gnu/libusb-1.0.so;/usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libflann_cpp.so;/usr/lib/x86_64-linux-gnu/libqhull_r.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND loam_ouster_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND loam_ouster_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT loam_ouster_NUM_DUMMY_TARGETS)
      set(loam_ouster_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::loam_ouster::wrapped-linker-option${loam_ouster_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR loam_ouster_NUM_DUMMY_TARGETS "${loam_ouster_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::loam_ouster::wrapped-linker-option${loam_ouster_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND loam_ouster_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND loam_ouster_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND loam_ouster_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/derm/ROS/install/lib;/home/derm/ROS/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(loam_ouster_LIBRARY_DIRS ${lib_path})
      list(APPEND loam_ouster_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'loam_ouster'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND loam_ouster_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(loam_ouster_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${loam_ouster_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "geometry_msgs;nav_msgs;roscpp;rospy;std_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 loam_ouster_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${loam_ouster_dep}_FOUND)
      find_package(${loam_ouster_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${loam_ouster_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(loam_ouster_INCLUDE_DIRS ${${loam_ouster_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(loam_ouster_LIBRARIES ${loam_ouster_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${loam_ouster_dep}_LIBRARIES})
  _list_append_deduplicate(loam_ouster_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(loam_ouster_LIBRARIES ${loam_ouster_LIBRARIES})

  _list_append_unique(loam_ouster_LIBRARY_DIRS ${${loam_ouster_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(loam_ouster_EXPORTED_TARGETS ${${loam_ouster_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${loam_ouster_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
