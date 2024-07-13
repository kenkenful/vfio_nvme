TESTFILE = test
CFLAGS = -DMSIX
#CFLAGS = -DINTx


#---------------------------------------------------------------
# Don't change the following
#---------------------------------------------------------------

INCLUDES = -I./include
LDFLAGS  = -lpthread
OBJDIR = ./objs
TARGETDIR = ./bin
TARGET = $(TARGETDIR)/$(TESTFILE)

GCC = g++
CFLAGS += -g

CPPS = $(shell find * -name *.cc)
SRCS = $(filter-out $(NOMAKEDIR), $(CPPS))
DIRS = $(dir $(SRCS))
BINDIRS = $(addprefix $(OBJDIR)/, $(DIRS))

OBJS = $(addprefix $(OBJDIR)/, $(patsubst %.cc, %.o, $(SRCS))) 
DEPS = $(OBJS:.o=.d)
TILS = $(patsubst %.cc, %.cc~, $(SRCS))

ifeq "$(strip $(OBJDIR))" ""
  OBJDIR = .
endif

ifeq "$(strip $(DIRS))" ""
  OBJDIR = .
endif

default:
	@[ -d  $(OBJDIR)   ] || mkdir -p $(OBJDIR)
	@[ -d "$(TARGETDIR)" ] || mkdir -p $(TARGETDIR)
	@[ -d "$(BINDIRS)" ] || mkdir -p $(BINDIRS)
	@make all --no-print-directory

all : $(OBJS) $(TARGET)

$(TARGET): $(OBJS) $(LIBS)
	$(GCC) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: %.cc
	$(GCC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

clean:
	@rm -rf $(TARGET) $(TILS) $(OBJDIR) $(TARGETDIR)

-include $(DEPS)