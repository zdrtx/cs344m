/*
Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*! \file PlayerTeams.cpp
<pre>
<b>File:</b>          PlayerTest.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       10/12/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class definitions for the
                      Player that are used to test the teams' high level
                      strategy.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
10/12/2000        Jelle Kok       Initial version created
</pre>
*/

#include "Player.h"

/*!This method is the first complete simple team and defines the actions taken
   by all the players on the field (excluding the goalie). It is based on the
   high-level actions taken by the simple team FC Portugal that it released in
   2000. The players do the following:
   - if ball is kickable
       kick ball to goal (random corner of goal)
   - else if i am fastest player to ball 
       intercept the ball
   - else
       move to strategic position based on your home position and pos ball */
SoccerCommand Player::deMeer5(  )
{

	SoccerCommand soc(CMD_ILLEGAL);
	VecPosition   posAgent = WM->getAgentGlobalPosition();
	VecPosition   posBall  = WM->getBallPos();
	int           iTmp;

	if( WM->isBeforeKickOff( ) )
	{
		if( WM->isKickOffUs( ) && WM->getPlayerNumber() == 9 ) // 9 takes kick
		{
			if( WM->isBallKickable() )
			{
				VecPosition posGoal( PITCH_LENGTH/2.0, (-1 + 2*(WM->getCurrentCycle()%2)) * 0.4 * SS->getGoalWidth() );
				soc = kickTo( posGoal, SS->getBallSpeedMax() ); // kick maximal
				Log.log( 100, "take kick off" );        
			}
			else
			{
				soc = intercept( false );  
				Log.log( 100, "move to ball to take kick-off" );
			}  
			ACT->putCommandInQueue( soc );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			return soc;
		}  
	
		if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
				posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )  
		{
			formations->setFormation( FT_INITIAL );       // go to kick_off formation
			ACT->putCommandInQueue( soc=teleportToPos( WM->getStrategicPosition() ));
		}
		else                                            // else turn to center
		{
			ACT->putCommandInQueue( soc=turnBodyToPoint( VecPosition( 0, 0 ), 0 ) );
			ACT->putCommandInQueue( alignNeckWithBody( ) );
		}
	}
	else
	{
		formations->setFormation( FT_433_OFFENSIVE );
		soc.commandType = CMD_ILLEGAL;

		/*
		 * Begin modified assignment 4 code
		 */
	
		if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
		{
			//If ball pos unknown, search for it
			ACT->putCommandInQueue( soc = searchBall() );
			ACT->putCommandInQueue( alignNeckWithBody( ) );
		}
		else if( WM->isBallKickable())
		{
			//Check deadball situations
			if( WM->isDeadBallUs())
			{
				ObjectT target = WM->getClosestInSetTo(OBJECT_SET_TEAMMATES_NO_GOALIE, OBJECT_BALL);
				VecPosition posPass = WM->getGlobalPosition(target);
				if(posAgent.getDistanceTo(posPass) < 10)
				{
					soc = directPass(posPass, PASS_NORMAL);
				}
			
				else
				{
					soc = directPass(posPass, PASS_FAST);
				}
			
				ACT->putCommandInQueue( soc );
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
				Log.log( 100, "kick ball" );
			}
			//Otherwise live ball
			else if( WM->getRelDistanceOpponentGoal() < 25)
			{
				//Are we within shooting distance of the opponent's goal?
				//If so, shoot towards a random corner of the goal
				VecPosition posGoal( PITCH_LENGTH/2.0,
					  (-1 + 2*(WM->getCurrentCycle()%2)) * 0.4 * SS->getGoalWidth() );
				soc = kickTo( posGoal, SS->getBallSpeedMax() ); // kick maximal

				ACT->putCommandInQueue( soc );
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
				Log.log( 100, "kick ball" );
			}
			//Is there a teammate we can pass to that's ahead of us?
			else
			{
				ObjectT teammate = WM->iterateObjectStart(iTmp, OBJECT_SET_TEAMMATES_NO_GOALIE);
				list<VecPosition> targets;
				while(teammate != OBJECT_ILLEGAL)
				{
					VecPosition posTeammate = WM->getGlobalPosition(teammate);
					if( posTeammate.getDistanceTo(WM->getPosOpponentGoal()) < WM->getRelDistanceOpponentGoal() - 5 &&
							posAgent.getDistanceTo(posTeammate) > 5 &&
							posAgent.getDistanceTo(posTeammate) < 35 &&
							WM->isOnside(teammate) &&
							WM->getListCloseOpponents(posTeammate, 10).size() == 0)
					{
						targets.push_back(posTeammate);
					}
					teammate = WM->iterateObjectNext(iTmp, OBJECT_SET_TEAMMATES_NO_GOALIE);
				}
				list<VecPosition>::iterator itr;
				if(targets.size() > 0)
				{
					//Select target to pass to randomly
					int pos = WM->getCurrentCycle() % targets.size();
					itr = targets.begin();
					for(int i = 0; i < pos; i++)
					{
						itr++;
					}
					VecPosition posPass = *itr;
					if(posAgent.getDistanceTo(posPass) < 10)
					{
						soc = directPass(posPass, PASS_NORMAL);
					}
					else
					{
						soc = directPass(posPass, PASS_FAST);
					}
				}
				else
				{
					//There are no suitable targets to pass to, so dribble
					soc = dribble(WM->getRelAngleOpponentGoal(), DRIBBLE_FAST);
				}	
			}
			ACT->putCommandInQueue( soc );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			Log.log( 100, "kick ball" );
		}
		else if( WM->getFastestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp ) == WM->getAgentObjectType()  && !WM->isDeadBallThem() )
		{           
		    //I'm fastest to ball, intercept
			Log.log( 100, "I am fastest to ball; can get there in %d cycles", iTmp );
			soc = intercept( false );

			//If stamina is low
			if( soc.commandType == CMD_DASH && WM->getAgentStamina().getStamina() < SS->getRecoverDecThr()*SS->getStaminaMax()+200 )
			{
				// Dash slow
				soc.dPower = 30.0 * WM->getAgentStamina().getRecovery(); 
				ACT->putCommandInQueue( soc );
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			}
			else
			{
				//Stamina is high, dash at full speed
				ACT->putCommandInQueue( soc );              
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			}
		}
		else
		{
			if( WM->getAgentStamina().getStamina() > SS->getRecoverDecThr()*SS->getStaminaMax()+800 )
			{
				// Stamina is high, move according to forces
				soc = moveToPos(posAgent + getForces(posAgent), PS->getPlayerWhenToTurnAngle());
				ACT->putCommandInQueue( soc );            
				if(WM->getCurrentCycle()%2 == 3)
				{
					ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
				}
				else
				{
					ACT->putCommandInQueue( turnNeckToObject( getLeastConfidentPlayer(), soc ) );
				}
			}
			else                                        
			{
				//Stamina is low, update agent's modeling of players/ball
				ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
				if(WM->getCurrentCycle()%2 == 3)
				{
					ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
				}
				else
				{
					ACT->putCommandInQueue( turnNeckToObject( getLeastConfidentPlayer(), soc ) );
				}
			}
		}
	
		/*
		 * End modified assignment 4 code
		 */
	}
	
	return soc;
}

/*
 * Begin research code
 */
 
#define DEBUG 0

#define sigmoid(x) (1/(1+exp(-x)))

#define sigmoid_deriv(x) (1/((1+exp(-x))*(1+exp(x))))

#define soft_not(x) (1-(x))

#define soft_and(x, y) ((x)*(y))

#define soft_or(x,y) (1-(1-(x))*(1-(y)))

#define soft_greater(x,y,delta) (sigmoid(((x)-(y))/(delta)))

#define soft_less(x,y,delta) (sigmoid(((y)-(x))/(delta)))

float soft_equal(float x, float y, float delta){
  float z = (x-y)/delta;
  return exp(-z*z);
}

float soft_if(float p, float x, float y){
  return p*x + (1-p)*y;
}

VecPosition soft_if_vector(float p, VecPosition x, VecPosition y){
  VecPosition v = x * p + y *(1-p);
  return v;
}

ObjectT Player::getLeastConfidentPlayer()
{
	int iIndex;
	ObjectT leastConfident;
	double lowestConfidence = 1;
	for ( ObjectT obj = WM->iterateObjectStart(iIndex, OBJECT_SET_PLAYERS);
		obj != OBJECT_ILLEGAL;
		obj = WM->iterateObjectNext(iIndex, OBJECT_SET_PLAYERS) )
	{
		double confidence = WM->getConfidence(obj);
		if(confidence < lowestConfidence)
		{
			leastConfident = obj;
			lowestConfidence = confidence;
		}
	}
	WM->iterateObjectDone(iIndex);
	
	if(WM->getConfidence(OBJECT_BALL) < lowestConfidence)
	{
		return OBJECT_BALL;
	}
	
	return leastConfident;
}

bool Player::isOffensive(VecPosition myPosition)
{
	int iIndex;
	int numTeammatesAhead = 0;
	for ( ObjectT obj = WM->iterateObjectStart(iIndex, OBJECT_SET_TEAMMATES);
		obj != OBJECT_ILLEGAL;
		obj = WM->iterateObjectNext(iIndex, OBJECT_SET_TEAMMATES) )
	{
		VecPosition teammatePosition = WM->getGlobalPosition(obj);
		if(teammatePosition.getX() > myPosition.getX())
		{
			numTeammatesAhead++;
		}
	}
	WM->iterateObjectDone(iIndex);
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 1)
	{
		cout << "Number of teammates in front: " << numTeammatesAhead << endl;
	}
	
	return numTeammatesAhead <= 2;
}

bool Player::isDefensive(VecPosition myPosition)
{
	int iIndex;
	int numTeammatesAhead = 0;
	for ( ObjectT obj = WM->iterateObjectStart(iIndex, OBJECT_SET_TEAMMATES);
		obj != OBJECT_ILLEGAL;
		obj = WM->iterateObjectNext(iIndex, OBJECT_SET_TEAMMATES) )
	{
		VecPosition teammatePosition = WM->getGlobalPosition(obj);
		if(teammatePosition.getX() > myPosition.getX())
		{
			numTeammatesAhead++;
		}
	}
	WM->iterateObjectDone(iIndex);
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 2)
	{
		cout << "Number of teammates in front: " << numTeammatesAhead << endl;
	}
	
	return numTeammatesAhead >= 7;
}

VecPosition Player::getForces(VecPosition myPosition)
{
	float ballDist = WM->getRelativeDistance(OBJECT_BALL);
	return getBoundaryForce(myPosition) +
			getOffsidesForce(myPosition) +
			soft_if_vector(soft_less(ballDist, 20, 10),
					getTacticalForce(myPosition) + getOpposingForce(myPosition),
					getStrategicForce(myPosition)) + 
			getBallFollowForce(myPosition) +
			getTransitionForce(myPosition) + 
      		getHotSpotForce(myPosition);
}

VecPosition Player::getBoundaryForce(VecPosition myPosition)
{
	float xMin = -52.5;
	float xMax = 52.5;
	float yMin = -34;
	float yMax = 34;

	float x = myPosition.getX();
	float y = myPosition.getY();
	
	float forceX = ((x > xMax) ? -5 : ((x < xMin) ? 5 : (5 / (x - xMin) - 5 / (xMax - x))));
	float forceY = ((y > yMax) ? -5 : ((y < yMin) ? 5 : (5 / (y - yMin) - 5 / (yMax - y))));
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 3)
	{
		cout << "Boundary forces: " << forceX << ", " << forceY << endl;
	}
	
	return VecPosition(forceX, forceY);
}

VecPosition Player::getOffsidesForce(VecPosition myPosition)
{
	float xOff = WM->getOffsideX();
	float forceX = soft_if(soft_less(xOff - myPosition.getX(), 5, 1), -5, 0);
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 4)
	{
		cout << "Offsides force: " << forceX << endl;
	}
	
	return VecPosition(forceX, 0);
}

VecPosition Player::getStrategicForce(VecPosition myPosition)
{
	int iIndex;
	VecPosition aggregateForce = VecPosition(0, 0);
	for ( ObjectT obj = WM->iterateObjectStart(iIndex, OBJECT_SET_TEAMMATES);
		obj != OBJECT_ILLEGAL;
		obj = WM->iterateObjectNext(iIndex, OBJECT_SET_TEAMMATES) )
	{
		VecPosition teammatePosition = WM->getGlobalPosition (obj);
		VecPosition relativeTeammatePosition = teammatePosition - myPosition;
		relativeTeammatePosition.normalize();

		double dist = WM->getRelativeDistance(obj);
		float force = soft_equal(dist, 20, 10) - 2 * soft_less(dist, 20, 10);
		aggregateForce += relativeTeammatePosition * force;
	}
	WM->iterateObjectDone(iIndex);
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 5)
	{
		cout << "Stategic forces: " << aggregateForce.getX() << ", " << aggregateForce.getY() << endl;
	}
	
	return aggregateForce;
}

VecPosition Player::getTacticalForce(VecPosition myPosition)
{
	int iIndex;
	VecPosition aggregateForce = VecPosition(0, 0);
	for ( ObjectT obj = WM->iterateObjectStart(iIndex, OBJECT_SET_TEAMMATES);
		obj != OBJECT_ILLEGAL;
		obj = WM->iterateObjectNext(iIndex, OBJECT_SET_TEAMMATES) )
	{
		VecPosition relativeVector = WM->getRelativePosition(obj).normalize();
		double dist = WM->getRelativeDistance(obj);
		float force = soft_if(soft_less(dist, 8, 3), -5, 0);
		aggregateForce += relativeVector * force;
	}
	WM->iterateObjectDone(iIndex);
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 6)
	{
		cout << "Tactical forces: " << aggregateForce.getX() << ", " << aggregateForce.getY() << endl;
	}
	
	return aggregateForce;
}

VecPosition Player::getOpposingForce(VecPosition myPosition)
{
	if(isDefensive(myPosition))
	{
		return getCoverForce(myPosition);
	}
	
	return getClearForce(myPosition);
}

VecPosition Player::getCoverForce(VecPosition myPosition)
{
	return getClearForce(myPosition) * -1;
}

VecPosition Player::getClearForce(VecPosition myPosition)
{
	int iIndex;
	VecPosition ballPosition = WM->getBallPos();
	Line passLane = Line::makeLineFromTwoPoints(myPosition, ballPosition);

	double closestDistToPassLane = 1000;
	double closestDistToMe = 1000;
	VecPosition keyDPosition;

	for ( ObjectT obj = WM->iterateObjectStart(iIndex, OBJECT_SET_OPPONENTS);
		obj != OBJECT_ILLEGAL;
		obj = WM->iterateObjectNext(iIndex, OBJECT_SET_OPPONENTS) )
	{
		VecPosition pos = WM->getGlobalPosition(obj);
		double dist = passLane.getDistanceWithPoint(pos);
		if(dist < closestDistToPassLane)
		{
			closestDistToPassLane = dist;
			closestDistToMe = WM->getRelativeDistance(obj);
			keyDPosition = pos;
		}
	}
	WM->iterateObjectDone(iIndex);

	float force;
	VecPosition orthVec = VecPosition(0, 0);
	if(closestDistToPassLane != 1000)
	{
	  VecPosition interceptPosition = passLane.getPointOnLineClosestTo(keyDPosition);
	  orthVec = keyDPosition - interceptPosition;
	  orthVec.normalize();
	  
	  force = soft_if(soft_less(closestDistToMe, 15, 2), -5, 0);
	  orthVec = orthVec * force;
	}

	if(WM->getPlayerNumber() == 6 && DEBUG == 7)
	{
		cout << "Ball Pos: " << ballPosition << endl;
		cout << "My Pos: " << myPosition << endl;
		cout << "Key D Pos: " << keyDPosition << endl;
		cout << "Get Clear Force: " << force << endl;
		cout << "OrthVec: " << orthVec << endl << endl;
	}
	
	return orthVec;
} 

VecPosition Player::getBallFollowForce(VecPosition myPosition)
{
	VecPosition unitVector = (WM->getBallPos() - myPosition).normalize();
	float dist = WM->getRelativeDistance(OBJECT_BALL);
	float force = soft_greater(dist, 30, 10);
	
	if(WM->getPlayerNumber() == 6 && DEBUG == 8)
	{
		cout << "Follow Ball forces: " << force << endl;
	}
	
	return unitVector * force;
}

VecPosition Player::getTransitionForce(VecPosition myPosition)
{
	if(isOffensive(myPosition))
	{
		float force = soft_if(soft_greater(WM->getBallPos().getX(), 10, 10),
								5 * soft_less(myPosition.getX(), 30, 10),
								-.25);
						
		VecPosition unitVec = (WM->getPosOpponentGoal() - myPosition).normalize();
		unitVec.setY(unitVec.getY() * .5);
	
		if(WM->getPlayerNumber() == 9 && DEBUG == 9)
		{
			cout << "Offensive force: " << force << endl;
		}
	
		return unitVec * force;
	}
	else if(isDefensive(myPosition))
	{
		float xGoalie = WM->getGlobalPosition(WM->getOwnGoalieType()).getX();
		float goalieX = soft_if(soft_less(myPosition.getX() - xGoalie, 5, 1), 5, 0);
		
		float force = soft_if(soft_less(WM->getBallPos().getX(), -10, 10),
								5 * soft_greater(myPosition.getX(), -30, 10),
								-.25);
	
		VecPosition unitVec = (WM->getPosOwnGoal() - myPosition).normalize();
		unitVec.setY(unitVec.getY() * .5);

		if(WM->getPlayerNumber() == 2 && DEBUG == 10)
		{
			cout << "Goalie force: " << goalieX << endl;
			cout << "Goal force: " << force << endl;
			cout << "Unit Vec: " << unitVec << endl;
		}

		return VecPosition(goalieX, 0) + unitVec * force;
	}
	
	return VecPosition(0, 0);
}

VecPosition Player::getHotSpotForce(VecPosition myPosition)
{
  VecPosition accumulator = VecPosition(0,0);
  int index;
  for (index = 0; index < NUM_HOTSPOTS; index++) 
  {
    if(isDefensive(myPosition))
    {
      VecPosition hotBallPos = WM->getHotBallPositions()[index];
      VecPosition unitVector = (hotBallPos - myPosition).normalize();
      float dist = (hotBallPos - myPosition).getMagnitude();
      double force = soft_if(soft_less(dist, 20, 0.1),
                5*soft_greater(dist, 10, 5),
                0);

      if(WM->getPlayerNumber() == 2 && DEBUG == 11)
      {
        cout << "hotBallPos: " << index << " : " << hotBallPos << endl;
        cout << "Unit Vec: " << unitVector << endl;
        cout << "Force : " << force << endl;
      }

      accumulator += unitVector * force;
    } 
    else if (isOffensive(myPosition))
    {
      VecPosition hotOppPos = WM->getHotOpponentPositions()[index];
      VecPosition unitVector = (myPosition - hotOppPos).normalize();
      float dist = (myPosition - hotOppPos).getMagnitude();
      float force = soft_if(soft_less(dist, 20, 0.1),
          5*soft_less(dist, 10, 5),
          0);

      if(WM->getPlayerNumber() == 9 && DEBUG == 12)
      {
        cout << "hotOppPos: " << index << " : " << hotOppPos << endl;
        cout << "Unit Vec: " << unitVector << endl;
        cout << "Force : " << force << endl;
      }

      accumulator += unitVector * force;
    }
  }

  return accumulator;
}

/*
 * End research code
 */





/*!This method is a simple goalie based on the goalie of the simple Team of
   FC Portugal. It defines a rectangle in its penalty area and moves to the
   position on this rectangle where the ball intersects if you make a line
   between the ball position and the center of the goal. If the ball can
   be intercepted in the own penalty area the ball is intercepted and catched.
*/
SoccerCommand Player::deMeer5_goalie(  )
{
  int i;
  SoccerCommand soc;
  VecPosition   posAgent = WM->getAgentGlobalPosition();
  AngDeg        angBody  = WM->getAgentGlobalBodyAngle();

  // define the top and bottom position of a rectangle in which keeper moves
  static const VecPosition posLeftTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH/4.0 );
  static const VecPosition posRightTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, +PENALTY_AREA_WIDTH/4.0 );

  // define the borders of this rectangle using the two points.
  static Line  lineFront = Line::makeLineFromTwoPoints(posLeftTop,posRightTop);
  static Line  lineLeft  = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posLeftTop.getY()), posLeftTop );
  static Line  lineRight = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posRightTop.getY()),posRightTop );


  if( WM->isBeforeKickOff( ) )
  {
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )  
    {
      formations->setFormation( FT_INITIAL );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos(WM->getStrategicPosition()) );
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc = turnBodyToPoint( VecPosition( 0, 0 ), 0 ));
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
    return soc;
  }

  if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
  {                                                // confidence ball too  low
    ACT->putCommandInQueue( searchBall() );        // search ball
    ACT->putCommandInQueue( alignNeckWithBody( ) );
  }
  else if( WM->getPlayMode() == PM_PLAY_ON || WM->isFreeKickThem() ||
           WM->isCornerKickThem() )               
  {
    if( WM->isBallCatchable() )
    {
      ACT->putCommandInQueue( soc = catchBall() );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
     else if( WM->isBallKickable() )
    {
       soc = kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );    
       ACT->putCommandInQueue( soc );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else if( WM->isInOwnPenaltyArea( getInterceptionPointBall( &i, true ) ) &&
             WM->getFastestInSetTo( OBJECT_SET_PLAYERS, OBJECT_BALL, &i ) == 
                                               WM->getAgentObjectType() )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
    {
      // make line between own goal and the ball
      VecPosition posMyGoal = ( WM->getSide() == SIDE_LEFT )
             ? SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_L, SIDE_LEFT )
             : SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_R, SIDE_RIGHT);
      Line lineBall = Line::makeLineFromTwoPoints( WM->getBallPos(),posMyGoal);

      // determine where your front line intersects with the line from ball
      VecPosition posIntersect = lineFront.getIntersection( lineBall );

      // outside rectangle, use line at side to get intersection
      if (posIntersect.isRightOf( posRightTop ) )
        posIntersect = lineRight.getIntersection( lineBall );
      else if (posIntersect.isLeftOf( posLeftTop )  )
        posIntersect = lineLeft.getIntersection( lineBall );

      if( posIntersect.getX() < -49.0 )
        posIntersect.setX( -49.0 );
        
      // and move to this position
      if( posIntersect.getDistanceTo( WM->getAgentGlobalPosition() ) > 0.5 )
      {
        soc = moveToPos( posIntersect, PS->getPlayerWhenToTurnAngle() );
        ACT->putCommandInQueue( soc );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
      else
      {
        ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
  }
  else if( WM->isFreeKickUs() == true || WM->isGoalKickUs() == true )
  {
    if( WM->isBallKickable() )
    {
      if( WM->getTimeSinceLastCatch() == 25 && WM->isFreeKickUs() )
      {
        // move to position with lesser opponents.
        if( WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS, 
                                          Circle(posRightTop, 15.0 )) <
            WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS, 
                                           Circle(posLeftTop,  15.0 )) )
          soc.makeCommand( CMD_MOVE,posRightTop.getX(),posRightTop.getY(),0.0);
        else
          soc.makeCommand( CMD_MOVE,posLeftTop.getX(), posLeftTop.getY(), 0.0);
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() > 28 )
      {
        soc = kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );    
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() < 25 )
      {
        VecPosition posSide( 0.0, posAgent.getY() ); 
        if( fabs( (posSide - posAgent).getDirection() - angBody) > 10 )
        {
          soc = turnBodyToPoint( posSide );
          ACT->putCommandInQueue( soc );
        }
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
    else if( WM->isGoalKickUs()  )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }
  else
  {
     ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
     ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }
  return soc;
}


