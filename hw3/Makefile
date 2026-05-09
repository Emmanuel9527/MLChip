HOME = /RAID2/COURSE/2026_Spring/es26mlchip/es26mlchipTA10/
LIB_DIR=$(HOME)/systemc-2.3.3/lib-linux64
INC_DIR=$(HOME)/systemc-2.3.3/include
LIB=-lsystemc-2.3.3
RPATH=-Wl,-rpath,$(HOME)/systemc-2.3.3/lib-linux64

O = run
C = *.cpp

all:
	g++ -I . -I $(INC_DIR) -L . -L $(LIB_DIR) -o $(O) $(C) $(LIB) $(RPATH)
	./$(O) 

clean:
	rm -rf $(O)