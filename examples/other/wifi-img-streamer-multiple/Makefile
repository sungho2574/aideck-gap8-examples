io=uart
PMSIS_OS = freertos

# Build with: make HOTSPOT_SSID=MyWifi HOTSPOT_PASSWORD=mypassword
ifdef HOTSPOT_SSID
APP_CFLAGS += -DHOTSPOT_SSID=\"$(HOTSPOT_SSID)\"
else
$(error HOTSPOT_SSID is required. Usage: make HOTSPOT_SSID=MyWifi HOTSPOT_PASSWORD=mypassword)
endif

ifdef HOTSPOT_PASSWORD
APP_CFLAGS += -DHOTSPOT_PASSWORD=\"$(HOTSPOT_PASSWORD)\"
else
$(error HOTSPOT_PASSWORD is required. Usage: make HOTSPOT_SSID=MyWifi HOTSPOT_PASSWORD=mypassword)
endif

APP = wifi-img-streamer
APP_SRCS += wifi-img-streamer.c ../../../lib/cpx/src/com.c ../../../lib/cpx/src/cpx.c
APP_CFLAGS += -O3 -g
APP_INC=../../../lib/cpx/inc
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

CONFIG_GAP_LIB_JPEG = 1

RUNNER_CONFIG = $(CURDIR)/config.ini


include $(RULES_DIR)/pmsis_rules.mk
