ZMODDAC_APP = zmoddac
ZMODADC_APP = zmodadc
ZMODSTART_APP = zmodstart

LIB_C_SOURCES   = $(shell find zmodlib -name '*.c')
LIB_CPP_SOURCES = $(shell find zmodlib -name '*.cpp') 


LIB_C_OBJS   = $(LIB_C_SOURCES:.c=.o)
LIB_CPP_OBJS = $(LIB_CPP_SOURCES:.cpp=.o)
LIB_OBJS     = $(LIB_C_OBJS) $(LIB_CPP_OBJS)

LDLIBS += -pthread

ZMODDAC_OBJS = zmoddac.o $(LIB_OBJS)
ZMODADC_OBJS = zmodadc.o $(LIB_OBJS)
ZMODSTART_OBJS = zmodstart.o $(LIB_OBJS)

CPPFLAGS += -Izmodlib \
            -Izmodlib/Zmod \
            -Izmodlib/Zmod/linux \
            -Izmodlib/ZmodDAC1411 \
            -Izmodlib/ZmodADC1410


all: $(ZMODDAC_APP) $(ZMODADC_APP) $(ZMODSTART_APP)


$(ZMODDAC_APP): $(ZMODDAC_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(ZMODADC_APP): $(ZMODADC_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(ZMODSTART_APP): $(ZMODSTART_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(ZMODDAC_APP) $(ZMODADC_APP) $(ZMODSTART_APP) \
	      $(LIB_OBJS) zmoddac.o zmodadc.o zmodstart.o
