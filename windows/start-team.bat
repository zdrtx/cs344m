@echo off
rem This script starts the UvA_Trilearn_2002 team. When player numbers are 
rem supplied before the (optional) host-name and team-name arguments only
rem these players are started, otherwise all players are started.

set host=localhost
set team=UvA_Trilearn
set prog=trilearn_player
set coach=trilearn_coach

echo "*****************************************************************"
echo "* UvA_Trilearn 2003 - University of Amsterdam, The Netherlands  *"
echo "* Created by:           Jelle Kok                               *"
echo "* Research Coordinator: Nikos Vlassis                           *" 
echo "* Team Coordinator:     Frans Groen                             *"
echo "* Copyright 2000-2001.  Jelle Kok and Remco de Boer             *"
echo "* Copyright 2001-2002.  Jelle Kok                               *"
echo "* Copyright 2002-2003.  Jelle Kok                               *"
echo "* All rights reserved.                                          *"
echo "*                                                               *"
echo "* Windows port:         Alexey Vasilyev                         *"
echo "*                       Riga Technical University, Latvia       *"
echo "*****************************************************************"

start %PROG% -n 1 -t %TEAM% -h %HOST%
start %PROG% -n 2 -t %TEAM% -h %HOST%
start %PROG% -n 3 -t %TEAM% -h %HOST%
start %PROG% -n 4 -t %TEAM% -h %HOST%
start %PROG% -n 5 -t %TEAM% -h %HOST%
start %PROG% -n 6 -t %TEAM% -h %HOST%
start %PROG% -n 7 -t %TEAM% -h %HOST%
start %PROG% -n 8 -t %TEAM% -h %HOST%
start %PROG% -n 9 -t %TEAM% -h %HOST%
start %PROG% -n 10 -t %TEAM% -h %HOST%
start %PROG% -n 11 -t %TEAM% -h %HOST%
start %COACH% -t %TEAM% -h %HOST%
