HOME      = /RAID2/COURSE/2026_Spring/es26mlchip/es26mlchipTA10
LIB_DIR = $(HOME)/systemc-2.3.3/lib-linux64
INC_DIR = $(HOME)/systemc-2.3.3/include
LIB = -lsystemc-2.3.3
RPATH = -Wl,-rpath,$(HOME)/systemc-2.3.3/lib-linux64

O = run
C = *.cpp
CXX ?= g++
TRACE ?= 0
TRACE_FLAGS =

ifeq ($(TRACE),1)
TRACE_FLAGS += -DFP_TRACE
endif

CXXFLAGS += -I . -I $(INC_DIR) -g $(TRACE_FLAGS)
LDFLAGS += -L . -L $(LIB_DIR) $(RPATH)

# Default target
all: $(O)

# Compile and link
$(O): $(C)
	$(CXX) $(CXXFLAGS) -o $(O) $(C) $(LDFLAGS) $(LIB)

# Run with cat.txt
cat: $(O)
	IMAGE_FILE_NAME=cat.txt ./run

# Run with dog.txt
dog: $(O)
	IMAGE_FILE_NAME=dog.txt ./run

# Run only the final DRAM output write/read/print path.
test-writeback: $(O)
	OUTPUT_WRITEBACK_TEST=1 IMAGE_FILE_NAME=cat.txt ./run

# Rebuild with trace logs enabled, then run with cat.txt
trace-cat:
	$(MAKE) clean
	$(MAKE) TRACE=1 cat

# Rebuild with trace logs enabled, then run with dog.txt
trace-dog:
	$(MAKE) clean
	$(MAKE) TRACE=1 dog

# Rebuild with trace logs enabled, then run only the final write-back path.
trace-test-writeback:
	$(MAKE) clean
	$(MAKE) TRACE=1 test-writeback

# Clean up compiled files
clean:
	rm -rf $(O)

.PHONY: all cat dog test-writeback trace-cat trace-dog trace-test-writeback clean
