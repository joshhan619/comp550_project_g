CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=`pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++
all: ProjectG

clean:
	rm -f *.o
	rm -f ProjectG

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

ProjectGNeedle: ProjectGNeedle.o SMR.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

ProjectG: ProjectG.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)