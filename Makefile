ccflags-y += ${PROJECTINCLUDE}
ccflags-y += -I$(srctree)/drivers

#
# This will pull in all source files present in the src directory
#
arduino_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/platform/*/*.c))
arduino_OBJECTS = $(patsubst %.c, %.o, $(arduino_SOURCES))

obj-y += $(arduino_OBJECTS)