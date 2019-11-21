
# OUTPUT type
# 1 - .out
# 2 - .a
# 3 - .so
OUTPUT_TYPE = 2
OUTPUT_NAME = wq_vtb

SUB_DIRS    = src

# .h files dir
ADD_INCLUDE += inc ext_inc

# predefined macro
PRE_MARCO   +=

# lib dir
ADD_LIBDIR  =

# lib need to ld together
ADD_LIB     =

#####################################################

ifdef TOPDIR
include $(TOPDIR)/build/makefile.cfg
else
include $(CURDIR)/build/makefile.cfg
TOPDIR = $(CURDIR)
export TOPDIR
endif

# display the obj files and output name
debug:
	@echo TOPDIR=$(TOPDIR)
	@echo OUTPUT_DIR=$(OUTPUT_DIR)
	@echo OUTPUT_MAP=$(OUTPUT_MAP)
	@echo OUTPUT_LIB=$(OUTPUT_FULL_NAME)
	@echo DEPS=$(DEPS)
	@echo OBJECTS=$(OBJECTS)
	@echo SRCS=$(SRCS)
	@echo OBJECTS folder=$(foreach dirname, $(SUB_DIRS), $(addprefix $(BIN_DIR)/, $(dirname)))
	@echo output_name=$(OUTPUT_FULL_NAME)
