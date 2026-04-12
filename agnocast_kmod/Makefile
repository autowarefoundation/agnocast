VERSION := $(shell cat $(src)/version.txt 2>/dev/null || echo "unknown")
ccflags-y += -Wall -Werror -DVERSION='"$(VERSION)"'

ifeq ($(KUNIT_BUILD),y)
  ifeq ($(CONFIG_KUNIT)$(CONFIG_GCOV_KERNEL),yy)
    obj-m := agnocast_kunit.o
    agnocast_kunit-objs := agnocast_kunit_main.o agnocast_main.o agnocast_memory_allocator.o\
	  agnocast_init.o \
	  agnocast_exit.o \
	  agnocast_ioctl.o \
	  agnocast_internal.o
    include $(src)/agnocast_kunit/Makefile.inc
    ccflags-y += -DKUNIT_BUILD -fprofile-arcs -ftest-coverage
  else
    $(warning "CONFIG_KUNIT or CONFIG_GCOV_KERNEL is not set")
  endif
else
  obj-m := agnocast.o
  agnocast-objs := agnocast_main.o agnocast_memory_allocator.o\
	agnocast_init.o \
	agnocast_exit.o \
	agnocast_ioctl.o \
	agnocast_internal.o
endif

KERNEL_VERSION ?= $(shell uname -r)
KERNEL_MAJOR := $(shell echo $(KERNEL_VERSION) | cut -d '.' -f 1)
KERNEL_MINOR := $(shell echo $(KERNEL_VERSION) | cut -d '.' -f 2)

# In Linux kernel v5.17 and earlier, the top-level Linux Makefile specifies -std=gnu89,
# so it is necessary to declare the variable used as the iterator in the for loop beforehand.
# For example:
#   int i;
#   for (i = 0; i < n; i++) {
#       ...
#   }
# Therefore, add -std=gnu11 to suppress this error.
ifeq ($(shell expr $(KERNEL_MAJOR) \< 5 \| $(KERNEL_MAJOR) = 5 \& $(KERNEL_MINOR) \<= 17), 1)
  ccflags-y += -std=gnu11
endif

# In Linux kernel versions v6.4 and earlier, the top-level Makefile includes
# -Wdeclaration-after-statement, which requires all variables used within
# a function to be declared at the beginning of the function. To suppress
# this, we should add -Wno-declaration-after-statement.
ifeq ($(shell expr $(KERNEL_MAJOR) \< 6 \| $(KERNEL_MAJOR) = 6 \& $(KERNEL_MINOR) \<= 4), 1)
  ccflags-y += -Wno-declaration-after-statement
endif

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

test:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) KUNIT_BUILD=y modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

.PHONY: all test clean
