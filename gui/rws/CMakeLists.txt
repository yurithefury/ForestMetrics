set(GUI_RWS_SOURCES main.cpp main_window.cpp seed_selection.cpp)
set(GUI_RWS_HEADERS main_window.h seed_selection.h)
set(GUI_RWS_FORMS   main_window.ui)

qt_wrap_ui(GUI_RWS_FORMS_HEADERS ${GUI_RWS_FORMS})

add_executable(gui_random_walker_segmentation
  ${GUI_RWS_SOURCES}
  ${GUI_RWS_FORMS_HEADERS}
)
target_link_libraries(gui_random_walker_segmentation
  io
  random_walker_segmentation
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${TVIEWER_LIBRARIES}
)
add_dependencies(gui_random_walker_segmentation
  tviewer
)
