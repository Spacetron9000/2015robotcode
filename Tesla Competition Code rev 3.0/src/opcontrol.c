     /** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"


/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {



	while (1) { //Keeps Teleop mode running indefinitely, so that motor and sensor values are constantly updated.


		if (digitalRead(3)== LOW){



			autonomous ();

			delay (6000); //remove when testing concludes
		}
		int Button5U = joystickGetDigital ( 1 , 5 , JOY_UP);
		int Button5D = joystickGetDigital ( 1 , 5 , JOY_DOWN);
		int Button7D = joystickGetDigital ( 1 , 7 , JOY_DOWN);
		int leftStick = (joystickGetAnalog (1,3)); //Acquire Joystick values
		int rightStick = (joystickGetAnalog (1,2));
		int leftSpeed;
		int rightSpeed;
		int liftSpeed;

	//Calculations

		if (Button5D){
			leftSpeed = leftStick / 2;
			rightSpeed = rightStick / 2;
			liftSpeed = 50;
		}

		else{
			leftSpeed = leftStick;
			rightSpeed = rightStick;
			liftSpeed = 127;
		}

	//End of Calculations

		//Preset Skyrise Height
		if (Button5U){
			liftP (AUTOLOADER, 1000, 0);
		}


	//Drivetrain Controls
		if (Button7D){ //Standard forward control
		motorSet (9, -rightSpeed);					motorSet (2, leftSpeed);


		motorSet (10, rightSpeed);					motorSet (1, -leftSpeed);
		}


		else{ //Inverted direction control
			motorSet (9, rightSpeed);					motorSet (2, -leftSpeed);


			motorSet (10, -rightSpeed);					motorSet (1, leftSpeed);
		}
		//Motors are set so that they will both move in the direction of each respective joystick.
	//End of Drivetrain Controls



	//Lift Controls
if (joystickGetDigital (1, 6, JOY_UP)){ //For debug purposes only. Will be replaced with PID code.
			motorSet (3,  -liftSpeed);
			motorSet (4,   liftSpeed);
			motorSet (7,   liftSpeed);
			motorSet (8,  -liftSpeed);

		}
		else if (joystickGetDigital (1, 6, JOY_DOWN)){ //For debug purposes only. Will be replaced with PID code.
			motorSet (3,   liftSpeed);
			motorSet (4,  -liftSpeed);
			motorSet (7,  -liftSpeed);
			motorSet (8,   liftSpeed);

		}

		else {
			motorSet (3, -10);      //For debug purposes only. Will be replaced with PID code.
			motorSet (4,  10);
			motorSet (7,  10);      //For debug purposes only. Will be replaced with PID code.
			motorSet (8, -10);
		}
	//End of Lift Controls



//Live debugging via the LCD Display.

		LCD_screen();

		delay(20); //Don't hog the CPU
	}
}
