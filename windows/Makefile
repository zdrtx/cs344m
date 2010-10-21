# Makefile to compile Robocup Soccer Simulation Team UvA_Trilearn 
# Date: Nov 28, 2002
# Author: Alexey Vasilyev
# Usage: make will create trilearn_player_player and trilearn_player_coach
# Comments: This makefile compiles all related sources
# This file designed for Borland C++ Compiler and Borland C++ Builder

CC                      = bcc32 ${FLAGS} $(INCLUDES) ${DEFINES}
COMPILE                 = $(CC) 

SRC_DIR                 = .
DST_DIR                 = .
TARGET_PLAYER           = trilearn_player
TARGET_COACH            = trilearn_coach
TARGET_TEST             = trilearn_test

# ==== INCLUDES =================================================

SOCCER_INCLUDES         = 
INCLUDES	        = $(SOCCER_INCLUDES)

# ==== FLAGS ====================================================

OSFLAGS                 = -DWIN32

CFLAGS_DEBUG            =
CFLAGS_OPT              = -O2
CFLAGS_WARN             =

FLAGS              = ${OSFLAGS} ${CFLAGS_OPT} ${CFLAGS_DEBUG} ${CFLAGS_WARN}

# ==== LIBS ====================================================

CLIBS_SunOS_5           = -lsocket -lnsl -lpthread
CLIBS_linux             = -lpthread

# ==== DEFINES =================================================

#DEFINES	                = -DEC_LOG_ENABLED=1

# ==== SOURCES =================================================

SRCS        =    \
        Connection.cpp \
        SenseHandler.cpp \
        ActHandler.cpp \
        SoccerTypes.cpp \
        Objects.cpp \
        WorldModel.cpp \
        WorldModelHighLevel.cpp \
        WorldModelPredict.cpp \
        WorldModelUpdate.cpp \
        ServerSettings.cpp \
        PlayerSettings.cpp \
        GenericValues.cpp \
        Formations.cpp \
        Geometry.cpp \
        Parse.cpp \
        Logger.cpp \
	tiles2.cpp 

SRCS_PLAYER = ${SRCS}    \
        BasicPlayer.cpp \
        Player.cpp \
        PlayerTest.cpp \
        PlayerTeams.cpp \
        PlayerTrilearn2003.cpp \
	CoordinationGraph.cpp \
	Agent.cpp \
	ValueRule.cpp \
	Action.cpp \
        Context.cpp \
        main.cpp 

SRCS_TEST = ${SRCS}    \
        mainTest.cpp

SRCS_COACH = ${SRCS} \
        BasicCoach.cpp \
        BasicCoachTest.cpp \
        mainCoach.cpp \
	CoordinationGraph.cpp \
	Agent.cpp \
	ValueRule.cpp \
	Action.cpp \
        Context.cpp 


OBJS_PLAYER  = $(SRCS_PLAYER:.cpp=.obj)
OBJS_COACH   = $(SRCS_COACH:.cpp=.obj)
OBJS_TEST    = $(SRCS_TEST:.cpp=.obj)

# ==== RULES ==================================================

all::    $(TARGET_PLAYER) $(TARGET_COACH) $(TARGET_TEST)

#.cpp.obj:
#	$(COMPILE) -c -o $@ $(@F:.obj=.cpp)

trilearn_player: $(OBJS_PLAYER)
	$(COMPILE) -e$(TARGET_PLAYER) $(OBJS_PLAYER) ${CFLAGS_LIBS}

trilearn_coach: ${OBJS_COACH}
	$(COMPILE) -e$(TARGET_COACH) $(OBJS_COACH) ${CFLAGS_LIBS}

trilearn_test: ${OBJS_TEST}
	$(COMPILE) -e$(TARGET_TEST) $(OBJS_TEST) ${CFLAGS_LIBS}

clean:
	del -f $(TARGET_COACH).exe $(TARGET_PLAYER).exe \
           $(TARGET_COACH).tds $(TARGET_PLAYER).tds \
           $(OBJS_PLAYER) $(OBJS_COACH)

depend:
	makedepend $(DEPENDFLAGS) -s

# DO NOT DELETE THIS LINE
