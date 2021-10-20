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

package org.firstinspires.ftc.team15358;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

/**
 * 15358 HT update to team configuration of Hardware, 5 motors, 4 servos.
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:       "left_front_drive"
 * Motor channel:  Right front drive motor:      "right_front_drive"
 * Motor channel:  Left rear drive motor:        "left_rear_drive"
 * Motor channel:  Right rear drive motor:       "right_rear_drive"
 * Motor channel:  Manipulator drive motor:      "lift"
 * Servo channel:  Servo to close claw:          "claw"
 * Servo channel:  Servo to close right grabber: "right_grabber"
 * Servo channel:  Servo to close left grabber:  "left_grabber"
 */
public class HardwarePushbotHT_20191012
{
    /* Public OpMode members. */
    //renamed all below per comments here
    public DcMotor  lfDrive   = null; // was leftDrive, now left front Drive Motor
    public DcMotor  rfDrive  = null;  // was rightDrive, now right front Drive Motor
    public DcMotor  lrDrive     = null;  // was leftArm, now left rear Drive Motor
    public DcMotor  rrDrive = null;  // new, now right rear Drive Motor
    public DcMotor  lift = null; // new, now lift pulley system Drive Motor
    public Servo    claw    = null; // was leftClaw, now claw servo
    public Servo    capstone   = null; // was rightClaw, now capstone servo
    public Servo    right_grabber   = null; // was rightClaw, now capstone servo
    public Servo    left_grabber   = null; // was rightClaw, now capstone servo

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotHT_20191012(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lfDrive = hwMap.get(DcMotor.class, "left_front_drive");
        rfDrive = hwMap.get(DcMotor.class, "right_front_drive");
        lrDrive = hwMap.get(DcMotor.class, "left_rear_drive");
        rrDrive = hwMap.get(DcMotor.class, "right_rear_drive");
        lift    = hwMap.get(DcMotor.class, "lift");

        //leftArm    = hwMap.get(DcMotor.class, "left_arm"); 15358 HT

        //not sure what these two lines do 15358 HT
        lfDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rfDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        lrDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rrDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        lift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors


        // Set all motors to zero power
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lrDrive.setPower(0);
        rrDrive.setPower(0);
        lift.setPower(0);
        //leftArm.setPower(0); 15358 HT

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 15358 HT

        // Define and initialize ALL installed servos.
        claw  = hwMap.get(Servo.class, "claw");
        capstone = hwMap.get(Servo.class, "capstone");
        right_grabber = hwMap.get(Servo.class, "right_grabber");
        left_grabber = hwMap.get(Servo.class, "left_grabber");
        claw.setPosition(MID_SERVO);
        capstone.setPosition(MID_SERVO);
    }
 }

