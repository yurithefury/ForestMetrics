include(ExternalProject)

ExternalProject_Add(tviewer
  GIT_REPOSITORY https://github.com/taketwo/tviewer.git
  GIT_TAG 019c53e
  TIMEOUT 10
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DWITH_QT=ON
)

ExternalProject_Get_Property(tviewer INSTALL_DIR)

set(LIBRARIES ${INSTALL_DIR}/lib/libtviewer.a)
if(${VTK_MAJOR_VERSION} VERSION_LESS "6.0")
  list(APPEND LIBRARIES QVTK)
endif()

set(TVIEWER_LIBRARIES ${LIBRARIES} CACHE INTERNAL "TViewer libraries")
set(TVIEWER_INCLUDE_DIR ${INSTALL_DIR}/include CACHE INTERNAL "TViewer include directory")
set(TVIEWER_DEFINITIONS "-std=c++11 -DBOOST_PP_VARIADICS=1" CACHE STRING "TViewer compiler definitions")
