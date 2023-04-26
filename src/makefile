# Makefile for Radiation Detector Linux

CC := gcc
LDLIBS := -lusb

TARGETS := RadiationDetectorLinux

SRCS := main.c c12137_comm.c
OBJS := $(SRCS:.c=.o) 

.PHONY: all
all: $(TARGETS)
$(TARGETS): $(OBJS)
	$(CC) $(OBJS) $(LDLIBS) -o $(TARGETS)

.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGETS)