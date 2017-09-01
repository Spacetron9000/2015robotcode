/*
 * Autonroutines.c
 *
 *  Created on: Nov 28, 2014
 *      Author: Kyle Medeiros and Wenheng Lu
 */
#include "main.h"
#define FORWARD 0
#define BACKWARD 1
#define LEFT 0
#define RIGHT 1
#define UP 0
#define DOWN 1
#define BW 0
#define FW 1


void Autonroutines(int Routine){

	if (Routine == 1){ //First Auton Routine, Red Autoloader

		drivePID (500, 500, FORWARD, 127);

		lift (1000, 127);
		lift (1000, -127);
		liftP (AUTOLOADER, 1000, 50);
/*
		while (digitalRead (3)== HIGH){

						}

		turnPID (62, 1500,LEFT, 127);

		while (digitalRead (3)== HIGH){

						}

		drivePID (300, 500, BACKWARD, 127);
		drivePID (100, 100, FORWARD, 127);

		while (digitalRead (3)== HIGH){

						}

		lift (400, 127);
		drivePID (500, 1000, FORWARD, 127);
		turnPID (52, 1000, LEFT, 127);

		while (digitalRead (3)== HIGH){

						}
		drivePID (200, 500, BACKWARD, 127);

		while (digitalRead (3)== HIGH){

						}
		liftP (GROUND, 1000, 0);
		drivePID (300, 500, FORWARD, 127);
*/

	}//End of Red Skyrise Autonomous


	else if (Routine == 2){ //Second Auton , Blue

	}



	else if (Routine == 3){ //Third Auton Routine, Red Post


		//Starting orientation: Robot is facing (and lined up with) the cube by the medium post. Preload is placed such that
		//it is touching the "cheater post" and the rear of the lift.

		liftP (POSTL, 2000, -120); //Cheater Cube, 3 points


		drivePID (1000, 300, BACKWARD, 127);

		liftP (POSTL, 1000, -500);

		drivePID (900, 500, FORWARD, 127);


/*

		drivePID (500, 1000, FORWARD, 127); //Drive to get 1st cube


		drivePID (100, 600, BACKWARD, 127); //Slightly back up



		turnPID (400, 500, LEFT, 127);

		liftP (GROUND, 2000, 0); //Pick up a cube

		lift (500, -127);



		liftP (POSTM, 2600, 0);

		while (digitalRead (3)== HIGH){

		}



		turnPID (90, 1000, LEFT, 127);

		drivePID (400, 1000, FORWARD, 127);

		turnPID (90, 1000, RIGHT, 127);

		drivePID (500, 1100, FORWARD, 127);

		turnPID (90, 1000, RIGHT, 127);

		drivePID (1000, 1700, FORWARD, 127);



		while (digitalRead (3)== HIGH){

		}

		drivePID (1, 1700, FORWARD, 127);

		liftP (POSTM, 500, -90);

		liftP (POSTM ,500, 0);



		turnPID (180, 2000, RIGHT, 127);

		drivePID (400, 1000, FORWARD, 127);
		 //Position to intake a second cube

		while (digitalRead (3)== HIGH){

		}

		drivePID (500, 3000, FORWARD, 127); //Drive to cube



		turnPID (180, 3000, RIGHT, 127); //Turn around to face Medium Post

*/

/*		drivePID (1000, 3000, FORWARD, 127);

		liftP (POSTM, 1000, -100); //Press intake down onto post to score cubes (5 points)

		liftP (POSTM, 1000, 0);  //Raise lift

		//Point total: 8


*/
	}

	else if (Routine == 4){ //Fourth Auton Routine, Blue Post



	}

	else if (Routine == 5){ //Fifth Auton Routine, Programming Skills (Primary)

		Autonroutines (1);  // Run Red autonomous

		while (digitalRead (3)== HIGH){

		}


		//Pick up the cube to the left of the starting tile
		lift (550, -120);
		drivePID (400, 1000, FORWARD, 127 );
		lift (100, 127);
		lift (100, -127);
		lift (500, 127);

		//Turn to face Skyrise

		turnPID (160, 1500, RIGHT, 127);

		//Score a second cube

		lift (1300, 127);

		drivePID (460, 1000, FORWARD, 127); //Forward was originally 400.

		lift (800, -127);

		drivePID (400, 1000, BACKWARD, 127);

		while (digitalRead (3)== HIGH){

		}


		lift (800, -127);

		drivePID (800, 1500, FORWARD, 127);

		lift (1700, 127);

		turnPID (130, 1500, RIGHT, 127);

		drivePID (500, 1000, FORWARD, 127);

		lift (400, -127);

		drivePID (800, 1000, BACKWARD, 127);





		//Score a third cube





	}

	else if (Routine == 6){ //Sixth Auton Routine, Programming Skills (Backup)

	}


}
