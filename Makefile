# Makefile

TOP_SRCDIR = /home/demulab/src/ode-0.16.1

CXX = g++
CFLAGS = -g
# CFLAGS = -O3 -march=native -DNDEBUG

LIB = -lode -ldrawstuff -lGL -lGLU -lX11 -lrt -lm -lpthread -lstdc++
LIBDIR = -L/usr/lib/x86_64-linux-gnu -L$(TOP_SRCDIR)/drawstuff/src/.libs  -L$(TOP_SRCDIR)/ode/src/.libs -L/usr/lib
INCDIR = -I$(TOP_SRCDIR)/include -I$(TOP_SRCDIR)/ode/src -I/usr/include


OBJS = 72action_6p_learning_crane_with_draw simple_action_v2_learning_with_draw  DQN_6p_learning_crane_with_draw 6p_learning_crane_with_draw xythetaLearning xythetaMove action_read stickness2 xythetanov learning legged boxcollide hopper2 bounce sensor4 stickness2_copy cranex7 monoBot monoBot2 omni arm1 learning_crane v2_learning_crane cranex7_nolearning cranex7_nolearning_ver_relative cranex7_act arm6 6p_learning_crane
# action_read_6p_replay
all: $(OBJS)

$(OBJS): %: %.cpp texturepath.h
	$(CXX) $(CFLAGS) -o $@ $< $(INCDIR) $(LIBDIR) $(LIB)

clean:
	rm  $(OBJS)  *~  *.*~


.PHONY: all clean
