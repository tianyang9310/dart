CXX    = g++
CFLAGS = -O2
INCLUDES = -I ./

TARGET = InvPendulum.out

SOURCES=$(shell find . -name '*.cpp')
HEADERS=$(shell find . -name '*.h')
OBJS =$(SOURCES:%.cpp=%.o)

all: $(TARGET)
					
$(TARGET): $(OBJS) $(HEADERS)
	$(CXX)  -o $@ $(OBJS) $(LIBS_GL)

clean:
	-rm -f $(OBJS)
.cpp.o:
	$(CXX) $(INCLUDES) -c $<
