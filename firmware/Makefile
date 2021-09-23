#
# Copyright 2018 Curt Brune <curt@brune.net>
# All rights reserved.
#

TARGET = argb-fx
C_SRCS += argb-fx.c init.c opensimplex2d.c
C_SRCS += $(BSP_BASE)/drivers/plic/plic_driver.c

C_OBJS = $(C_SRCS:.c=.o)
CFLAGS += -O2 -fno-builtin-printf -DNO_INIT -DUSE_PLIC -DUSE_M_TIME $(WARN_ERROR_FLAGS)

$(C_OBJS) : WARN_ERROR_FLAGS = -Wall -Werror
$(C_OBJS) : CFLAGS += -std=c99

include $(BSP_BASE)/env/common.mk
