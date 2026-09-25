#######################################
# LuojiaFox RM2027 LFCore makefile
#######################################

CPP_SOURCES += $(wildcard $(CORE_DIR)/1_Middleware/*/*.cpp) \
$(wildcard $(CORE_DIR)/2_Device/*/*.cpp) \
$(wildcard $(CORE_DIR)/2_Device/*/*/*.cpp) \
$(wildcard $(CORE_DIR)/3_LuojiaFox/*/*.cpp) \
$(wildcard $(CORE_DIR)/4_Application/*/*.cpp) 

# 获取所有子目录
MIDDLEWARE_SUBDIRS += \
$(wildcard $(CORE_DIR)/1_Middleware/*)

DVC_SUBDIRS += \
$(wildcard $(CORE_DIR)/2_Device/*)

LUOJIAFOX_SUBDIRS += \
$(wildcard $(CORE_DIR)/3_LuojiaFox/*)

APP_SUBDIRS += \
$(wildcard $(CORE_DIR)/4_Application/*)

C_INCLUDES += \
$(addprefix -I,$(MIDDLEWARE_SUBDIRS)) \
$(addprefix -I,$(DVC_SUBDIRS)) \
$(addprefix -I,$(LUOJIAFOX_SUBDIRS)) \
$(addprefix -I,$(APP_SUBDIRS))
