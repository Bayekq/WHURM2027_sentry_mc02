#######################################
# LuojiaFox RM2027 LFCore makefile
#######################################

CPP_SOURCES += \
$(wildcard $(CORE_DIR)/1_Middleware/*/*.cpp) \
$(wildcard $(CORE_DIR)/1_Middleware/*/*/*.cpp) \
$(wildcard $(CORE_DIR)/4_Application/*/*.cpp) \
$(wildcard $(CORE_DIR)/3_LuojiaFox/*.cpp) \
$(wildcard $(CORE_DIR)/template.cpp)

# 获取所有子目录（用于头文件包含路径）
MIDDLEWARE_SUBDIRS := \
$(wildcard $(CORE_DIR)/1_Middleware/*) \
$(wildcard $(CORE_DIR)/1_Middleware/*/*)

APP_SUBDIRS := \
$(wildcard $(CORE_DIR)/4_Application/*)

LUOJIAFOX_SUBDIRS := \
$(wildcard $(CORE_DIR)/3_LuojiaFox)

C_INCLUDES += \
-I$(CORE_DIR) \
$(addprefix -I,$(MIDDLEWARE_SUBDIRS)) \
$(addprefix -I,$(APP_SUBDIRS)) \
$(addprefix -I,$(LUOJIAFOX_SUBDIRS))