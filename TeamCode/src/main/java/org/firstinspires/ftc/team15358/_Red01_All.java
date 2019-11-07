/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RED 01 (All Actions)", group="OnBot")
//
//@Disabled
public class _Red01_All extends LinearOpMode {

    /* Declare OpMode members. */



    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    AutoCommon auto=null;

    Boolean blue=false;

   @Override
    public void runOpMode() {

       /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
       auto = new AutoCommon(this);

       auto.resetEncoders();

       auto.liftClaw.robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       auto.liftClaw.disengageGrabbers();

       auto.liftClaw.robot.claw.setPosition(.8);


       auto.chassis.robot.leftGuide.setPosition(0);
       auto.chassis.robot.rightGuide.setPosition(1);

       // Wait for the game to start (driver presses PLAY)
       waitForStart();

       //Open claw after start otherwise outside allowed size
       auto.liftClaw.robot.claw.setPosition(0);

       //Drive to position to view blocks with camera and detect skystone
       auto.encoderDrive(.5, 600, 10, true);  // S1: Forward 47 Inches with 5 Sec timeout


       //Run vuforia to detect location of skystone
       VectorF blockLoc = auto.objectCheck(2);

       // Evaluate location of block and determine which path to take, currently only evaluate 2 right blocks in a set
       if (blockLoc != null) {

           //This value indicates the skystone is in the right position
           if (blockLoc.get(1) > 3) {

               //Drive to right positioned skystone and close claw, backup and turn towards build zone
               auto.GetRightOrLeftBlock(TURN_SPEED, false, true);

               //Drive to forward into the build zone and turn towards platform
               auto.DriveToPlatformPosition(TURN_SPEED,false, false, true,true);

           //This value indicates the skystone is in the center position
           } else if (blockLoc.get(1) < 3) {

               //Drive to center positioned skystone and close claw, backup and turn towards build zone
               auto.GetCenterBlock(false);

               //Drive to forward into the build zone and turn towards platform
               auto.DriveToPlatformPosition(TURN_SPEED,false, true, false,true);
           }
       //Default value, if not identified as center or right it is assumed to be the left position
       } else {

           //Drive to left positioned skystone and close claw, backup and turn towards build zone
           auto.GetRightOrLeftBlock(TURN_SPEED, false, false);

           //Drive to forward into the build zone and turn towards platform
           auto.DriveToPlatformPosition(TURN_SPEED,false, false, false, true);
       }

       //Drive forward and grab platform, backup then turn platform and push into corner
       auto.GetAndPlacePlatform(DRIVE_SPEED, TURN_SPEED, blue);

   }

}
