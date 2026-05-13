# Webots R2025a C student controller Makefile.

C_SOURCES = student_controller.c robot_navigation.c
LIBRARIES = -lm

ifeq ($(strip $(WEBOTS_HOME)),)
WEBOTS_HOME = C:/PROGRA~1/Webots
endif
ifneq (,$(findstring Program Files,$(WEBOTS_HOME)))
WEBOTS_HOME = C:/PROGRA~1/Webots
endif
WEBOTS_HOME_PATH = $(subst \,/,$(WEBOTS_HOME))

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
