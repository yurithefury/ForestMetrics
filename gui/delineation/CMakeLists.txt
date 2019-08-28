ADD_QT_FORM(GUI_DELINEATION main_window)
ADD_QT_FORM(GUI_DELINEATION preprocessing_form)
ADD_QT_FORM(GUI_DELINEATION trunk_detection_form)
ADD_QT_FORM(GUI_DELINEATION graph_building_form)
ADD_QT_FORM(GUI_DELINEATION seed_selection_form)
ADD_QT_FORM(GUI_DELINEATION segmentation_form)
ADD_QT_FORM(GUI_DELINEATION min_max_widget)

list(APPEND GUI_DELINEATION_SOURCES main.cpp
                                    seed_selection.cpp
                                    cluster_list_model.cpp)
list(APPEND GUI_DELINEATION_HEADERS seed_selection.h
                                    cluster_list_model.h)

qt_wrap_ui(GUI_DELINEATION_FORMS_HEADERS ${GUI_DELINEATION_FORMS})

add_executable(gui_delineation
  config.cpp
  ${PROJECT_SOURCE_DIR}/src/mesh_grid.cpp
  ${PROJECT_SOURCE_DIR}/src/tree_top_detector.cpp
  ${GUI_DELINEATION_SOURCES}
  ${GUI_DELINEATION_FORMS_HEADERS}
)
target_link_libraries(gui_delineation
  io
  kde
  random_walker_segmentation
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${TVIEWER_LIBRARIES}
)
add_dependencies(gui_delineation
  tviewer
)
