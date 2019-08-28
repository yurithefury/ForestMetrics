#-------------------------------------------------
#
# Project created by QtCreator 2014-08-21T10:59:23
#
#-------------------------------------------------

QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gui
TEMPLATE = app

SOURCES += main.cpp\
           mainwindow.cpp \
           seed_selection.cpp \
           main_window.cpp \
    preprocessing_form.cpp \
    trunk_detection_form.cpp \
    graph_building_form.cpp \
    seed_selection_form.cpp \
    segmentation_form.cpp \
    min_max_widget.cpp

HEADERS  += mainwindow.h \
            seed_selection.h \
            main_window.h \
    preprocessing_form.h \
    trunk_detection_form.h \
    graph_building_form.h \
    seed_selection_form.h \
    segmentation_form.h \
    min_max_widget.h

FORMS += main_window.ui \
    preprocessing_form.ui \
    trunk_detection_form.ui \
    graph_building_form.ui \
    seed_selection_form.ui \
    segmentation_form.ui \
    min_max_widget.ui
