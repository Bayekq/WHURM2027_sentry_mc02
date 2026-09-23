#######################################
# LuojiaFox RM2027 LFCore makefile
#######################################

CPP_SOURCES += \
$(wildcard $(CORE_DIR)/Application/SysMng/*.cpp) \
$(wildcard $(CORE_DIR)/Application/UserTasks/*.cpp) \
$(wildcard $(CORE_DIR)/BSP/*/*.cpp) \
$(wildcard $(CORE_DIR)/Components/Algorithm/*/*.cpp) \
$(wildcard $(CORE_DIR)/Components/Device/*/*.cpp) \
$(wildcard $(CORE_DIR)/Components/Device/*/*/*.cpp)


# 获取所有子目录
APP_SUBDIRS := $(wildcard $(CORE_DIR)/Application/*)
BSP_SUBDIRS := $(wildcard $(CORE_DIR)/BSP/*)
ALGORITHM_SUBDIRS := $(wildcard $(CORE_DIR)/Components/Algorithm/*)
DEVICE_SUBDIRS := $(wildcard $(CORE_DIR)/Components/Device/*)
INSTANCE_SUBDIRS += \
$(wildcard $(CORE_DIR)/Components/Device/RC/DT7) \
$(wildcard $(CORE_DIR)/Components/Device/Motor/DJI) \
$(wildcard $(CORE_DIR)/Components/Device/Motor/DM)
LF_SUBDIRS := $(wildcard $(CORE_DIR)/LuojiaFox/)

C_INCLUDES += \
$(addprefix -I,$(APP_SUBDIRS)) \
$(addprefix -I,$(BSP_SUBDIRS)) \
$(addprefix -I,$(ALGORITHM_SUBDIRS)) \
$(addprefix -I,$(DEVICE_SUBDIRS)) \
$(addprefix -I,$(INSTANCE_SUBDIRS)) \
$(addprefix -I,$(LF_SUBDIRS))
