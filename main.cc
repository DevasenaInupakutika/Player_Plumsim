/*

compiler: g++ -o main `pkg-config --cflags playerc++` main.cc `pkg-config --libs playerc++`

to run more than 1 robot (ex: robot id=2): ./main -p 6666 -r 2

*/ 

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "args.h"

#define MaximumRobots 12
#define RobotStep 3.0 //Maximum amount of distance a robot walks each time
#define RobotMinimumWalk 0.01 //Minimum amount of distance a robot walks 
#define goal_distance_cutoff 0.5  //Cut-off distance from goal above which the robots start to scatter
#define SQUARE(A) (((A) * (A)))         //Returns the square of a number

using namespace PlayerCc;

PlayerClient    *robot;
LaserProxy      *lp;
Position2dProxy *pp;
Position2dProxy *pp1;

int ProgramON = 1;
float GoalDest[2] = { 200 , 120 };


//Stucture values of the robots
struct Struct_RobotValues{

    int ID;
    int state;
    float position_x;
    float position_y;
    float goalreach_value;
};

struct Struct_RobotValues RobotValues[MaximumRobots];
struct Struct_RobotValues ThisRobot;
char* itoa(int val, int base){
	
	static char buf[32] = {0};
	static char zero[5];
	strcpy(zero, "0\0");
	int i = 30;
	if (val ==0) return zero;
	for(; val && i ; --i, val /= base)
	
		buf[i] = "0123456789abcdef"[val % base];
	
	return &buf[i+1];
	
}

void UpdateRobotValues(){

    RobotValues[ThisRobot.ID-1].ID = ThisRobot.ID;
    RobotValues[ThisRobot.ID-1].state = ThisRobot.state;
    RobotValues[ThisRobot.ID-1].position_x = ThisRobot.position_x;
    RobotValues[ThisRobot.ID-1].position_y = ThisRobot.position_y;
    RobotValues[ThisRobot.ID-1].goalreach_value = ThisRobot.goalreach_value;

    printf("Updating robot values to file in line %i \n", ThisRobot.ID-1);

    printf("RobotValues: %i, %i, %f %f %f \n", RobotValues[ThisRobot.ID-1].ID, RobotValues[ThisRobot.ID-1].state, RobotValues[ThisRobot.ID-1].position_x, RobotValues[ThisRobot.ID-1].position_y, RobotValues[ThisRobot.ID-1].goalreach_value);

}

void SaveValues(void)
{
    FILE *stream;            /* need a pointer to FILE for the stream */
    int buffer_character;    /* need an int to hold a single character */
    int i;
    char filename[30];
	strcpy(filename,"robotvalues");
    strcpy(filename,"robotvalues\0");
    strcat(filename,itoa(ThisRobot.ID-1,10));
  	strcat(filename,".txt\0");
		
	stream = fopen(filename, "wt");
	printf("updating file %s\n\n",filename);
	
	
  
    if (stream == (FILE *)0) {
        fprintf(stderr, "Error opening file (printed to standard error)\n");
        return ;
        exit (1);
        }  /* end if */
        
    fprintf(stream,"+\n");
	i = ThisRobot.ID-1;
        fprintf(stream,"%i %.2f %.2f %i %f \n",RobotValues[i].ID, RobotValues[i].position_x, RobotValues[i].position_y, RobotValues[i].state, RobotValues[i].goalreach_value);
        printf("%i %f %f %i %f \n",RobotValues[i].ID, RobotValues[i].position_x, RobotValues[i].position_y, RobotValues[i].state, RobotValues[i].goalreach_value);

    if ((fclose(stream)) == EOF) {
        fprintf(stderr,"Error closing stream.  (printed to standard error)\n");
        }  /* end if */
}

void ReadValues(void){

    FILE *stream;            /* need a pointer to FILE for the stream */
    int buffer_character;    /* need an int to hold a single character */
    int i;
    char cc;
    char filename[30];

    strcpy(filename,"robotvalues");
  printf(" reading files ... \n");
    for (i=0;i<MaximumRobots; i++){
    	    strcpy(filename,"robotvalues\0");
    	    strcat(filename,itoa(i,10));
    	    strcat(filename,".txt\0");
		
			stream = fopen(filename, "rt");
			if (stream == (FILE *)0) {
				fprintf(stderr, "Error opening file (printed to standard error)\n");
				continue;
				exit (1);
				}  /* end if */

			fscanf(stream,"%c",&cc);
			while (cc != '+')
				fscanf(stream,"%c",&cc);
			{
				fscanf(stream,"%i %f %f %i %f \n",&RobotValues[i].ID, &RobotValues[i].position_x, &RobotValues[i].position_y, &RobotValues[i].state, &RobotValues[i].goalreach_value);
				printf("%i %f %f %i %f \n",RobotValues[i].ID, RobotValues[i].position_x, RobotValues[i].position_y, RobotValues[i].state, RobotValues[i].goalreach_value);
			}

			if ((fclose(stream)) == EOF) {
				fprintf(stderr,"Error closing stream.  (printed to standard error)\n");
			}  /* end if */
			}
}

//Return 1 if it has the greatest distance from goal and 0 if not
int HighGoalValue(void){

    int i, temp_ID;

    for(i=0 ; i < MaximumRobots ; i++){
        if(RobotValues[i].ID == ThisRobot.ID){
            temp_ID = i;
            break;
        }
    }

    for(i=0 ; i < MaximumRobots ; i++){

        if(( RobotValues[i].ID == ThisRobot.ID) || (RobotValues[i].ID <= 0) || (RobotValues[i].goalreach_value > 0) || (RobotValues[i].state != 0) )
            continue;
            else         
            if( RobotValues[i].goalreach_value > RobotValues[temp_ID].goalreach_value )
            {
				if( RobotValues[i].goalreach_value < RobotValues[temp_ID].goalreach_value-0.01 && i< temp_ID) 
				continue;
				else
				{
					fprintf(stderr,"\n ################# is not the greatest stopped !  ######## \n");
					fprintf(stderr,"\n robot %d is greater than me \n", i);
				
					return 0;
				}
			}
    }

    return 1;
}

int GetID_RobotLowGoal(void){

    int i, temp_ID ;
    float temp_goal=0.5;

    for(i=0 ; i < MaximumRobots ; i++){

        if(RobotValues[i].goalreach_value < temp_goal && RobotValues[i].state == 0){
            temp_ID = RobotValues[i].ID;
            temp_goal = RobotValues[i].goalreach_value;
        }
    }

    return temp_ID;

}


//Get position, goal reach values and state(stoped(for localization) or exploring) from other robots and if this robot as lower goal distance value it is the one to move, have to see if there are enough robots stopped to make the localization so it can walk.

int ExchangeValues(){

    int i, temp_numberRobots = 0;

    //Get position, state and goal distance values from other robots
    ReadValues();
        
    UpdateRobotValues();

    SaveValues();

    //if it has the higher goal distance value
    if( HighGoalValue() == 1){

        printf("With Highest goal distance\n");
        //See how many robots stopped there are
        for(i=0 ; i < MaximumRobots ; i++){
            if(RobotValues[i].ID != ThisRobot.ID && RobotValues[i].state == 0 && RobotValues[i].ID > 0) //last condition to prevent reading from robots who haven't been initialized
                temp_numberRobots++;
        }
        printf("Number of stopped robots: %d \n", temp_numberRobots);
        //If there are enough robots stopped to explore
        if( temp_numberRobots >= 3){
            return 1;
        }
    }
    printf("--> Robot stopped, waiting for turn to move \n");
    return 0;
}

//! Stops the robot
void StopRobot() {

    pp->SetSpeed(0,0,0);
    pp1->SetSpeed(0,0,0);
  return;
}

//if it is the time for this robot to move,go in the direction of the objective until the maximum distance allowed by the other robots
int explore(float objective_position[2], float *angle){

    int id;
    float temp_x, temp_y, objective[2], temp_goal, temp_x1, temp_y1;

    player_pose2d goto_pose;

    printf("------  Starting searching ------\n");

    objective_position[0] = ThisRobot.position_x + RobotStep*cos(*angle)-2;  
    objective_position[1] = ThisRobot.position_y + RobotStep*sin(*angle);

    goto_pose.px = objective_position[0];
    goto_pose.py = objective_position[1];
    goto_pose.pa = 0.0;

    pp1->GoTo(goto_pose);

    robot->Read();
    temp_x = pp1->GetXPos();
    temp_y = pp1->GetYPos();

    temp_goal = ThisRobot.goalreach_value;

    printf("x « %.2f y « %.2f , Objective = ( %.2f, %.2f)  angle = %f \n",temp_x,temp_y, objective_position[0], objective_position[1], *angle);

    //if it reach the objective or the maximum allowed distance
    while(sqrt(SQUARE( pp1->GetYPos() - temp_y ) + SQUARE(pp1->GetXPos() - temp_x))  < RobotStep  ){


        id = GetID_RobotLowGoal(); //ID of the robot with lowest goal distance value
        
//         if(sqrt(SQUARE( pp1->GetYPos() - temp_y ) + SQUARE(pp1->GetXPos() - temp_x))  > RobotMinimumWalk){

//             printf("Passed minimum walk \n");
            if( ThisRobot.goalreach_value < RobotValues[id].goalreach_value  || ThisRobot.goalreach_value/temp_goal > 0.8 ){
                printf("Got Lower goal distance value \n");
                
                temp_x1 = pp1->GetXPos();
                temp_y1 = pp1->GetYPos();
                while(sqrt(SQUARE( pp1->GetYPos() - temp_y ) + SQUARE(pp1->GetXPos() - temp_x))  < RobotMinimumWalk){
                    robot->Read(); //Read Robot odometry
                    sleep(1);
                }                
                break;
            }
//         }

        robot->Read(); //Read Robot odometry
        ThisRobot.position_x = pp1->GetXPos();
        ThisRobot.position_y = pp1->GetYPos();
//         pp1->GoTo(goto_pose);

        printf("odometry = ( %f, %f )\n", ThisRobot.position_x, ThisRobot.position_y);

        objective_position[0] = ThisRobot.position_x + RobotStep*cos(*angle)-2 ; // wind effect ali 2010 
        objective_position[1] = ThisRobot.position_y + RobotStep*sin(*angle);

        goto_pose.px = objective_position[0];
        goto_pose.py = objective_position[1];
        goto_pose.pa = 0.0;
        pp1->GoTo(goto_pose);
        printf("Position(%.2f, %.2f) -->> GoTo( %.2f, %.2f) :: angle:%f -> RobotLowGoal:%d \n", ThisRobot.position_x, ThisRobot.position_y, objective_position[0], objective_position[1], *angle, id);

    
        
        sleep(1);

    }

    //stop and change the state to stop. Exit the if
    StopRobot();
    printf("Robot stoping, finish exploring \n");

    return 1;
}

//Get the orientation to follow according to the values and position of other robots
int GetObjectiveOrientation( float objective_position[2] , float *angle){

    int i, j, temp_i=-1, temp_Gas=0, RobotToFollow, pp, p;
    int ordertable[MaximumRobots], lowest_1=0, lowest_2=0, lowest_3=0;

    /* initialize random seed: */
    srand ( time(NULL) );

    for(i=0 ; i < MaximumRobots; i++){
        ordertable[i] = RobotValues[i].ID;
    }

    //Buble sort algorithm, sorts robots by goal distance value
    for(i=0 ; i < MaximumRobots-1; i++){
        for(j=0 ; j < MaximumRobots-i-1 ; j++){
            p = ordertable[j]-1;
            pp = ordertable[j+1]-1;
            if(RobotValues[p].goalreach_value < RobotValues[pp].goalreach_value){
                ordertable[j] = RobotValues[pp].ID;
                ordertable[j+1] = RobotValues[p].ID;
            }
            else{
                ordertable[j] = RobotValues[p].ID;
                ordertable[j+1] = RobotValues[pp].ID;
            }
        }

    }

    printf("Order table \n");
    for(i=0 ; i < MaximumRobots; i++){
        printf("%d --> %d-(%.3f) \n ",i ,ordertable[i], RobotValues[ordertable[i]-1].goalreach_value);
    }
    printf("\n");

    
    for(i=MaximumRobots-1; i>=0 ; i--){
        p = ordertable[i]-1;
        if(RobotValues[p].state == 0){
            if(lowest_1 == 0.5){
                lowest_1 = ordertable[i];
                continue;
            }
            if(lowest_2 == 0.5){
                lowest_2 = ordertable[i];
                continue;
            }
            if(lowest_3 == 0.5){
                lowest_3 = ordertable[i];
                break;
            }
        }
    }

    /* generate number from 1 to 3 */
    RobotToFollow = rand() % 3 + 1;

    temp_i = 0;
    //See which robot to follow from the lowest 3

        if(RobotToFollow == 1)
            temp_i = lowest_1;

        if(RobotToFollow == 2)
            temp_i = lowest_2;

        if(RobotToFollow == 3)
            temp_i = lowest_3;


   *angle = atan2(RobotValues[temp_i-1].position_y - ThisRobot.position_y , RobotValues[temp_i-1].position_x - ThisRobot.position_x);



    objective_position[0] = RobotValues[temp_i-1].position_x + RobotStep*cos(*angle)- 5; 
    objective_position[1] = RobotValues[temp_i-1].position_y + RobotStep*sin(*angle);

    printf("Objective = (%.2f , %.2f) , angle=%f  robot to follow:%d \n", objective_position[0], objective_position[1], *angle, temp_i);
    printf("angle:%.2f This robot(%.2f; %.2f) -> objective robot %d = (%.2f; %.2f) : rand=%d \n", *angle, ThisRobot.position_x, ThisRobot.position_y, temp_i, RobotValues[temp_i-1].position_x, RobotValues[temp_i-1].position_y, RobotToFollow);
 
    return 1;

}

/*void *SensorMeasure(void *a){


    while( ProgramON ){
		robot->Read();
		nose->GetData(reinterpret_cast<uint8_t*>(&noseData));
		//Read angles from laser, representing the 5 sonars
        printf("w(%.3lf %.3lf) c(%.3lf)\n",noseData.windSpeed, noseData.windDirection, noseData.chemical);
		odor = noseData.chemical;
		ThisRobot.gas_value = odor;
		
        //Read gas sensor value
///        ThisRobot.gas_value = (500.0 - sqrt(SQUARE(GasSource[0] - ThisRobot.position_x ) + SQUARE(GasSource[1] - ThisRobot.position_y)) )/500.0 *4;

        usleep(300000);
    }

    return NULL;
}*/

int HighGoalDistance(void){
    
    int i;

    for(i=0 ; i<MaximumRobots ; i++ ){
        
        if(RobotValues[i].goalreach_value < goal_distance_cutoff)
            return 0;
    }
    return 1;
    
}

int Scatter(void){

    int i, temp_n;
    float centre_x, centre_y, temp_x, temp_y, angle, temp_goal;
    player_pose2d goto_pose;

    printf("\n******  starting to scatter *****\n\n");

    temp_x=0;
    temp_y=0;
    temp_n=0;   //number of robots working

    //Get centre of mass
    for(i=0 ; i<MaximumRobots ; i++) {
        if(RobotValues[i].ID > 0 && RobotValues[i].state != 0){ //Don't include stopped robots
            temp_x += RobotValues[i].position_x;
            temp_y += RobotValues[i].position_y;
            temp_n++;
        }
    }
    centre_x = float(temp_x/temp_n);
    centre_y = float(temp_y/temp_n);
    printf("centre=(%.2f ; %.2f) temp_x=%.2f  temp_y=%.2f temp_n=%d \n", centre_x, centre_y, temp_x, temp_y, temp_n);

    //Calcule oposite direction
    angle = atan2(ThisRobot.position_y - centre_y , ThisRobot.position_x - centre_x);

    //Go in that direction
    goto_pose.px = ThisRobot.position_x + RobotStep*cos(angle);  
    goto_pose.py = ThisRobot.position_y + RobotStep*sin(angle);
    goto_pose.pa = 0.0;

    pp1->GoTo(goto_pose);
    
    printf("Centre(%.2f ; %.2f) angle=%.2f Robot position=(%.2f ; %.2f) goto=(%.2f ; %.2f) \n", centre_x, centre_y, angle, ThisRobot.position_x, ThisRobot.position_y, goto_pose.px, goto_pose.py);

    robot->Read();
    temp_x = pp1->GetXPos();
    temp_y = pp1->GetYPos();

    temp_goal = ThisRobot.goalreach_value;

    //if it reaches the objective or the maximum allowed distance
    while(sqrt(SQUARE( pp1->GetYPos() - temp_y ) + SQUARE(pp1->GetXPos() - temp_x))  < RobotStep  ){

            //Exit "Scatter" state if found a goal distance less than the goal cut-off distance.
            if( ThisRobot.goalreach_value < goal_distance_cutoff){
                printf("Got less goal distance than the required goal cut-off /n");
         
                break;
            }


        robot->Read(); //Read odometry
        ThisRobot.position_x = pp1->GetXPos();
        ThisRobot.position_y = pp1->GetYPos();

        goto_pose.px = ThisRobot.position_x + RobotStep*cos(angle); 
        goto_pose.py = ThisRobot.position_y + RobotStep*sin(angle);
        goto_pose.pa = 0.0;
        pp1->GoTo(goto_pose);

        sleep(1);

    }

    //stop and change the state to stop. Exit the if block.
    StopRobot();
    
    return 1;

}

//main Swarm decision making function
int DecisionMaking(){

        float objective[2], angle;
                robot->Read(); //Read odometry
    			ThisRobot.position_x = pp1->GetXPos();
                ThisRobot.position_y = pp1->GetYPos();  
        //Get position, goal distance values and state(stoped(for localization) or exploring) from other robots and if this robot has high goal distance value, it is the one to move, have to see if there are enough robots stopped to make the localization so it can walk.
        if(ExchangeValues()){

            //Scatter -> to get close to goal.
            if(HighGoalDistance()){ //if every robot has a goal distance above cut-off.
                ThisRobot.state = 1;

                ReadValues();
                UpdateRobotValues();
                SaveValues();

                Scatter(); //Starting to scatter

                ThisRobot.position_x = pp1->GetXPos();
                ThisRobot.position_y = pp1->GetYPos();            
                ThisRobot.state = 0;   //change its state to stoped and wait for your turn to explore

                ReadValues();
                UpdateRobotValues();
                SaveValues();

                return 1;
            }


            //Explore
            else{

                printf("--> Inside Exchange Values if \n");
                //change state to explore
                RobotValues[ThisRobot.ID-1].state = 1;     //State 1 = explore                
                ThisRobot.state = 1;

                ReadValues();
                UpdateRobotValues();
                SaveValues();

                printf("DecisionMaking  - 1 \n");
                GetObjectiveOrientation(objective , &angle);
                
                printf("DecisionMaking  - 2 \n");
                //if it is the time for this robot to move,go in the direction of the objective until the maximum distance allowed by definition
                explore(objective, &angle);

                printf("DecisionMaking  - 3 \n");

                //Localize itself with odometry and sonars trough triangulation with others
                //#########  Ricardo Function missing   ############
                //for now we just use odometry
                    
                ThisRobot.position_x = pp1->GetXPos();
                ThisRobot.position_y = pp1->GetYPos();            
                ThisRobot.state = 0;   //change its state to stopped and wait for your turn to explore
                
                ReadValues();
                UpdateRobotValues();
                SaveValues();

                return 1;
            }
        }
        
        return 0;
}



int main(int argc, char *argv[]){

	parse_args(argc,argv);

	using namespace PlayerCc;
                    
    robot=new PlayerClient("localhost", gPort);

   
	
    robot->SetDataMode(PLAYER_DATAMODE_PULL);
    robot->SetReplaceRule(TRUE, PLAYER_MSGTYPE_DATA);

    pp=new Position2dProxy(robot,gIndex);
    pp1=new Position2dProxy(robot,gIndex+12);
    lp=new LaserProxy(robot,gIndex);

    int x=1;

 //   pthread_t sensor_thread;
    
 //   pthread_create(&sensor_thread, NULL, &SensorMeasure, &x);

    pp->SetMotorEnable (true);

    //---------- Reading sensor values and updating-------------

    //Read existing values/info from other robots
    ReadValues();

    //Initialization
    ThisRobot.ID = gRobotID;    //default 1

    //Read robot sensor values (odometry)
    robot->Read();
    
    pp1->SetOdometry(X_pos, Y_pos, 0.0);

    ThisRobot.position_x = X_pos;
    ThisRobot.position_y = Y_pos;

    printf("Initial robot position = (%.2f , %.2f)\n",ThisRobot.position_x, ThisRobot.position_y );

    //Read goal distance values
   // ThisRobot.goalreach_value = (100.0 - sqrt(SQUARE(GoalDest[0] - ThisRobot.position_x ) + SQUARE(GoalDest[1] - ThisRobot.position_y)) )/100.0;
    
    ThisRobot.state = 0;

    UpdateRobotValues();    //Update This robot values to global robot values structure 
    
    SaveValues(); 
    //--------------------------------------------------------------

	while(1){

        printf("Beginning of while cicle\n    ------\n");

        DecisionMaking();

        //sleep
        sleep(1);

    }
}

