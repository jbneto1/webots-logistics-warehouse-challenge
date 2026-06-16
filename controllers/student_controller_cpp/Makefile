# Webots R2025a C++ student controller.

CXX_SOURCES = student_controller.cpp robot_navigation.cpp
LIBRARIES = -lm

ifeq ($(strip $(WEBOTS_HOME)),)
WEBOTS_HOME = C:/PROGRA~1/Webots
endif
ifneq (,$(findstring Program Files,$(WEBOTS_HOME)))
WEBOTS_HOME = C:/PROGRA~1/Webots
endif
WEBOTS_HOME_PATH = $(subst \,/,$(WEBOTS_HOME))

include $(WEBOTS_HOME_PATH)/resources/Makefile.include