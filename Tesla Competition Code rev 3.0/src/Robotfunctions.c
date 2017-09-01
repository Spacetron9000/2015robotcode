/*
` * Robotfunctions.c
 *
 *  Created on: Nov 23, 2014
 *      Author: Kyle Medeiros and Wenheng Lu
 */

#include "main.h"
/*
 * The purpose of this .c file is to store all of the frequently reused functions
 * in our code, such as PID controlled movement of the drivetrain and lift,
 * an LCD debugging system, and other useful functions. This file helps to reduce
 * clutter from the other main files. All of these functions are usable throughout the entire
 * project, since they were prototyped in main.h.
 */

//All of these #defines are to make the PID functions easier to use
//as opposed to memorizing which number equates to which direction.

#define FORWARD 0   //For drivePID
#define BACKWARD 1  //For drivePID
#define LEFT 0   	//for turnPID
#define RIGHT 1  	//for turnPID
#define UP 0
#define DOWN 1

	void drivePID (int TargetValue, int Time, int Direction, int Speedcap){
		/*TargetValue is based on encoder counts.
	 Time should allow for error to reach 0, but shouldn't delay the entire routine.
	  Time should be in (ms * 10), since the loop runs every 10 ms.
	 Direction determines which way the motors run, either FORWARD or BACKWARD.

	 **************************************************************************
	 ***Function written by Kyle Medeiros of 8899 (Tesla Robotics) ************
	 ***************************************************************************/
		//Tested values for Kp, Ki, and Kd (in addition to imposed speedcaps)

			/* Kp       Ki        Kd      Speedcap       Error (Negative is overshoot)
				0.43	0.005	   0.55		127				-15
				0.39	0.005		0.55	127				-5
				0.39	0.003		0.55	127				-15
				0.39	0.003		0.50	127				-13
				0.36	0.003		0.50	127			     -6
		*/

		float Kp = 0.36; //Proportional Constant
		float Ki = 0.003; // Integral Constant
		float Kd = 0.5;//Derivative Constant
		int Error; //Target Value - Sensor Reading
		int Integral = 0; //Running sum of previous errors
		int Derivative; //Predicts the future error
		int previous_error = 0; //Previous Error
		int motorSpeed; //The motor speed based on Kp, Ki, and Kd
		int leftCounts; //Sensor Value given from left encoder

		int StartTime = 0;

		encoderReset (encoderLeft);  //Resets the encoder for this function's use

		while (StartTime < Time){ //This runs the PID loop for however long is specified.




			 leftCounts = abs (encoderGet (encoderLeft)); /*Sensor value from left encoder.
			 Absolute value makes the error easier to calculate, since the distance travelled
			 is the only relevant factor.	*/


			Error = TargetValue - leftCounts; //Proportional value

			Integral = Integral + Error;  //Integral Value

			 if (Error == 0){
				 Integral = 0;  //Prevents the integral building up after error = 0
			 }

			 if (abs(Error)>40){
				 Integral = 0;     //The integral won't come into play until the error becomes small enough
			 }

			 Derivative = Error - previous_error; //Derivative Value

			 previous_error = Error; //Set previous error = error for each loop

			 motorSpeed = (Kp*Error)+(Ki*Integral)+(Kd*Derivative);  //Power given to motors

			 if (motorSpeed > Speedcap){
				 motorSpeed = Speedcap;  //Adjustable speedcap to limit motor speed (127 = no speedcap)
			 }

			 	 	 if (Direction == FORWARD){
				motorSet (9, motorSpeed);					motorSet (2, -motorSpeed);


				motorSet (10, -motorSpeed);					motorSet (1, motorSpeed);

				//Motors are set to move FORWARD until error reaches 0.

}

			 	 	 else if (Direction == BACKWARD){
				motorSet (9, -motorSpeed);					motorSet (2, motorSpeed);


				motorSet (10, motorSpeed);					motorSet (1, -motorSpeed);

				//Motors are set to move in REVERSE until error reaches 0.
				//Possible thanks to taking the absolute value of the encoder reading.

}				lcdPrint(uart1,1,"EncoderL:%d",leftCounts);     //Displays sensor reading on LCD. For debug purposes only.
				lcdPrint (uart1, 2, "Error:%d", Error);         //Displays remaining error on LCD. For debug purposes only.


				StartTime = StartTime + 10 ; // 10 is added each time because the loop runs every 10 ms.

				delay (10); //Runs the loop every 10 ms, so as not to hog the CPU

		} // End of while loop

		if (Direction == FORWARD){
		motorSet (1, -15);
		motorSet (2, 15);
		motorSet (9, -15);
		motorSet (10, 15);
		}

		else if (Direction == BACKWARD){
		motorSet (1, 15);
		motorSet (2, -15);
		motorSet (9, 15);
		motorSet (10, -15);
		}
		delay (100);

		motorStop (1);  //Stop the motors completely when the loop is ended. Error should be 0 when this happens.
		motorStop (2);
		motorStop (9);
		motorStop (10);
		encoderReset (encoderLeft); //Reset the encoder.

	} //end of drivePID function

//Tested values for Kp, Ki, and Kd (in addition to imposed speedcaps)

	/* Kp       Ki        Kd      Speedcap       Error (Negative is overshoot)

*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void turnPID (int TargetValue, int Time, int Direction, int Speedcap){
		/*TargetValue is based on the gyro.
	 Time should allow for error to reach 0, but shouldn't delay the entire routine.
	  Time should be in (ms * 10), since the loop runs every 10 ms.
	 Direction determines which way the motors run, either LEFT or RIGHT. */

		/***************************************************************************
		 ***Function written by Kyle Medeiros of 8899 (Tesla Robotics) ************
		 ***************************************************************************/

		//Tested values for Kp, Ki, and Kd (in addition to imposed speedcaps)

		/* Kp       Ki        Kd      Speedcap       Error (Negative is overshoot)
			0.9		0.003	  0.4		127				10
			0.95	0.003	  0.4		127				6
			1.00	0.003	  0.4		127				0
		 */

		float Kp = 1.60; //Proportional Constant
		float Ki = 0.004; // Integral Constant
		float Kd = 0;  //Derivative Constant
			int Error;   //Target Value - Sensor Reading
			int Integral = 0; //Running sum of previous errors
			int Derivative;  //Predicts the future error
			int previous_error = 0; //Previous Error
			int motorSpeed; //The motor speed based on Kp, Ki, and Kd


			int StartTime = 0;


			gyroReset (gyro);  //Reset the gyro

			while (StartTime < Time){ //This runs the PID loop for however long is specified.




				int gyroAngle = abs (gyroGet(gyro));  /*Sensor value from gyro.
			 Absolute value makes the error easier to calculate, since the angle rotated
			 is the only relevant factor.	*/


				Error = TargetValue - gyroAngle; //Proportional value

				Integral = Integral + Error; //Integral value

				 if (Error == 0){
					 Integral = 0;  //Prevents the integral building up after error = 0
				 }

				 if (abs(Error)>10){
					 Integral = 0;  //The integral won't come into play until the error is small enough
				 }

				 Derivative = Error - previous_error; //Derivative value

				 previous_error = Error;  //Set previous error = error for each loop

				 motorSpeed = (Kp*Error)+(Ki*Integral)+(Kd*Derivative); //Power given to motors

				 if (motorSpeed > Speedcap){
					 motorSpeed = Speedcap;  //Adjustable speedcap to limit motor speed (127 = no speedcap)
				 }

				 	 	 if (Direction == LEFT){
					motorSet (9, motorSpeed);					motorSet (2, motorSpeed);


					motorSet (10, -motorSpeed);					motorSet (1, -motorSpeed);

					//Motors are set to rotate LEFT until error reaches 0.
	}

				 	 	 else if (Direction == RIGHT){
					motorSet (9, -motorSpeed);					motorSet (2, -motorSpeed);


					motorSet (10, motorSpeed);					motorSet (1, motorSpeed);

					//Motors are set to rotate RIGHT until error reaches 0.
	}
							lcdPrint(uart1,1,"Gyro:%d",gyroAngle); //Displays gyro value on LCD. For debug purposes only.
							lcdPrint (uart1, 2, "Error:%d", Error); //Displays remaining error on LCD. For debug purposes only.

					StartTime = StartTime + 10 ; // 10 is added each time because the loop runs every 10 ms.

					delay (10);  //Runs the loop every 10 ms, so as not to hog the CPU

			} //End of while loop.

			motorStop (1); //Stop the motors completely when the loop is ended. Error should be 0 when this happens.
			motorStop (2);
			motorStop (9);
			motorStop (10);
			gyroReset (gyro);

		}											 //end of turnPID function




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	void liftP (ArmPosition position , int Time, int Armoffset){
		/*TargetValue is based on the arm potentiometer.
	 Time should allow for error to reach 0, but shouldn't delay the entire routine.
	  Time should be in (ms * 10), since the loop runs every 10 ms.

     The data type "ArmPosition" contains 12 arm positions and names them.
     "Armoffset" takes the arm position from the array and subtracts an offset value from it.
     Should only be used on the "score_skyrise" function. Set to 0 for default. */

		/**************************************************************************
		***Function written by Kyle Medeiros of 8899 (Tesla Robotics) ************
		***************************************************************************/


		int ArmPositions [5] = {1200,//Ground level (Also autoloader height)
								1400,//Autoloader at first skyrise height (1570 when skyrise is picked up)
								1900, //Height of low post 23"
								2320,//Height of Medium Post 39"
								2600,//Height of High Post 47"
								 }; //Various arm positions for the lift.

		//Be sure to transfer these armpositions to the PID_TOM function as well.

			float Kp = 0.4;
			float Ki = 0.02;
			//These constant values should be the same for both liftPID and PID_TOM.
			/* Kp     Ki     ERROR:  AL     PL     PM     PH
			 B 0.35   0              40     60     60
			 T
			 -----
			 B 0.37   0              51     58     36     57
			 T                       40     20     16     12
             -----
			 B 0.36   0              49     56     54     52
			 T                       23     26     38     2
			 -----
			 B 0.36   0.02           53     64     50     51
			 T                       40     20     34     4
			 -----
			 B 0.36   0.05           57     68     56     52
			 T                       20     21     31     7
			 -----
			 B 0.36   0.1            60     68     54     49
			 T                       10     20     38     10
			 -----
			 B 0.5    0.1            20     32     19     17
			 T                       30     33    -10     38
			 -----
			 B 0.5    0.05           28     34     23     26
			 T                       31     9      7      35
			 -----
			 B 0.55   0.1            12     23     13     15
			 T                       30     40I    32I    30
			 -----
			 B 0.55   0              19     20     12     14
			 T                       16     30    -13     34
			 -----
			 B 0.45   0              43     47     33     31
			 T                       10     3      14     44
			 -----After rubber bands
			 B 0.45   0              3      24     26    -7
			 T                       6     -40    -25     7
			 -----
			 B 0.42   0              1      15     5     -7
			 T                     >|20|   -16     18    -13
			 -----
			 B 0.4    0            o-6    u 19   o-3    o-3
			 T                     o 12   u-7    o 9    u-17
			 -----
			 B 0.4    0.02          -2      24     5      1
			 T                      -6      3      2     -24
			 */


			int Error;
			int Integral = 0;
			int Derivative;
			int previous_error = 0;
			int motorSpeed;
			int StartTime = 0;


			while (StartTime < Time){ //This runs the PID loop for however long is specified.


				int armPot =  (analogRead (1));


				Error = ArmPositions [position] + Armoffset - armPot;


				 motorSpeed = (Kp*Error + Ki*Integral);




					motorSet (3,  	-motorSpeed);
					motorSet (4, 	 motorSpeed);
					motorSet (7, 	 motorSpeed);
					motorSet (8,  	-motorSpeed);


					StartTime = StartTime + 10 ;
					delay (10);

			lcdPrint(uart1,1,"Armpot:%d",armPot); //Displays gyro value on LCD. For debug purposes only.
			lcdPrint (uart1, 2, "Error:%d", Error); //Displays remaining error on LCD. For debug purposes only.

			}

			motorSet (3,  	-20);
			motorSet (4, 	 20);
			motorSet (7, 	 20);
			motorSet (8,  	-20);

		} //end of liftPID function

	void lift (int time, int Speed)
	{
		motorSet (3,  -Speed);
		motorSet (4,   Speed);
		motorSet (7,   Speed);
		motorSet (8,  -Speed);

		delay (time);

		motorSet (3,  -10);
		motorSet (4,   10);
		motorSet (7,   10);
		motorSet (8,  -10);


	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void LCD_screen(){

	/**************************************************************************
	***Function written by Kyle Medeiros of 8899 (Tesla Robotics) ************
	***************************************************************************/

//   This function displays sensor values and whichever autonomous program
//   is currently selected.  Press the limit switch to see sensor values, and
//   keep it unpressed to see which auton routine is going to be run.

	int armPot =  (analogRead (1));
	int autoDial = analogRead (2);
	int leftCounts = encoderGet (encoderLeft);
	int rightCounts = encoderGet (encoderRight);
	int gyroAngle = gyroGet (gyro);
	int Limitswitch = digitalRead (10);


	if (Limitswitch == HIGH){

		lcdSetText(uart1,1,"Current Auto:");
		if (autoDial >= 0 && autoDial <= 700){

			lcdSetText(uart1,2,"SKYRISE RED");
		}
		else if (autoDial >= 701 && autoDial <= 1200){

			lcdSetText(uart1,2,"SKYRISE BLUE");
		}
		else if (autoDial >= 1201 && autoDial <= 1800){

			lcdSetText(uart1,2,"POST RED");
		}
		else if (autoDial >= 1801 && autoDial <= 2400){

			lcdSetText(uart1,2,"POST BLUE");
		}
		else if (autoDial >= 2401 && autoDial <= 3000){

			lcdSetText(uart1,2,"P. SKILLS");
		}
		else if (autoDial >= 3001 && autoDial <= 3600){

			lcdSetText(uart1,2,"MISC");
		}
	}

	else if (Limitswitch == LOW){


			if (autoDial >= 0 && autoDial <= 700){
				lcdSetText(uart1,1,"Gyro");
				lcdPrint(uart1,2,"%d",gyroAngle);
			}
			else if (autoDial >= 701 && autoDial <= 1200){

				lcdSetText(uart1,1,"ArmPot");
				lcdPrint (uart1,2, "%d", armPot);
			}
			else if (autoDial >= 1201 && autoDial <= 1800){

				lcdPrint(uart1,1,"EncoderL:%d",leftCounts);
				lcdPrint(uart1,2,"EncoderR:%d",rightCounts);
			}

			if (joystickGetDigital (1, 8, JOY_UP)){ //Debug purposes only. Delete prior to final build.
				gyroReset (gyro);
			}
			if (joystickGetDigital (1, 8, JOY_LEFT)){ //Debug purposes only. Delete prior to final build.
				encoderReset (encoderLeft);
			}
			if (joystickGetDigital (1, 8, JOY_RIGHT)){ //Debug purposes only. Delete prior to final build.
				encoderReset (encoderRight);
			}

		}

}// End of LCD_screen function

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void score_skyrise (ArmPosition Currentpos /*int Targetvalue*/, int Offset){

	/**************************************************************************
	***Function written by Kyle Medeiros of 8899 (Tesla Robotics) ************
	***************************************************************************/

	/*
	 * This function is a quick shortcut for skyrise scoring.
	 * This will take the current armpos, bring it down to score a skyrise,
	 *  and then bring it back up to the original position.
	 */

	liftP (Currentpos, 1000, -Offset);
// 	lift (Targetvalue - Offset, 300, DOWN, 127);
	drivePID (200, 900, BACKWARD, 127);

	liftP (Currentpos, 1000, 0);
//	lift (Targetvalue, 300, UP, 127);



}
//end of score_skyrise function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PID_TOM (){

	/**************************************************************************
	***Function written by Kyle Medeiros of 8899 (Tesla Robotics) ************
	***************************************************************************/

	/*
	 * (Stands for PID: Teleoperated Mode)
	 * This function is designed to control the lift during teleop.
	 * There are no inputs besides the joystick, since this loop will
	 * keep running until the end of the match.
	 */


	int ArmPositions [12] =
	{560, 795, 915, 1003, 1183, 1393, 1537, 1707, 1962, 1200, 1570, 1177};

	//Various arm positions for the lift.

	//These positions WILL change based upon our determination of the proper armpot values.




					int index = 0; // Arm Position Index
					int button5U = joystickGetDigital (1, 5, JOY_UP); //Joystick button 5, up
					int button5D = joystickGetDigital (1, 5, JOY_DOWN); //Joystick button 5, down

					//Arm control
					if (button5U){

						if (index < 12){

							liftP (index, 1000, 0);

							index++;
							//Increases the index number by 1 when Button5U is pressed.
							//Also updates the TargetValue to the current ArmPosition.
						}
					}

					if (button5D){
						if (index > 0){

							liftP (index, 1000, 0);
							index--;
							//Decreases the index number by 1 when Button5D is pressed.
							//Also updates the TargetValue to the current ArmPosition.
						}
					}




						}

void StopAllMotors(){
	motorStop (1);
	motorStop (2);
	motorStop (3);
	motorStop (4);
	motorStop (5);
	motorStop (6);
	motorStop (7);
	motorStop (8);
	motorStop (9);
	motorStop (10);
}

	int StopDirection;
	#define BW 0
	#define FW 1

void DriveDStop(int StopDirection){
	if (StopDirection == 0){
		//Dead Stop for Backward
		motorSet (1, 5);
		motorSet (2, -5);
		motorSet (9, 5);
		motorSet (10, -5); //Drive Forward

		wait(50);

		StopAllMotors();
	}

	else if (StopDirection == 1){
		//Dead Stop for Forward
		motorSet (1, -5);
		motorSet (2, 5);
		motorSet (9, -5);
		motorSet (10, 5); //Drive Backward

		wait(50);

		StopAllMotors();
	}
}


	void turn (int time, int Speed){ //positive speed for right

		motorSet (1,  Speed);
		motorSet (2, -Speed);
		motorSet (9, -Speed);
		motorSet (10, Speed);

		delay (time);

		StopAllMotors();


}

