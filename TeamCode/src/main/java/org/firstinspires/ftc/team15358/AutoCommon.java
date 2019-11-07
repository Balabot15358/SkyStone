
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoCommon {

    public AutoHardware robot = null;
    public ChassisCommon chassis = null;
    public LiftClawCommon liftClaw = null;
    public VuforiaSkyStoneCommon vuforiaCom = null;


    public VectorF blockLoc = null;
    public CameraDevice vufCam = null;

    private ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode curOpMode = null;

    public AutoCommon(LinearOpMode owningOpMode) {

        curOpMode = owningOpMode;

        chassis = new ChassisCommon(curOpMode);
        liftClaw = new LiftClawCommon(curOpMode);
        robot = new AutoHardware();
        vuforiaCom = new VuforiaSkyStoneCommon(curOpMode);
        robot.init(curOpMode.hardwareMap);

        vufCam = CameraDevice.getInstance();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             int encoderValue,
                             double timeoutS, boolean pid) {
        double correction = 0;


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            // Determine new target position, and pass to motor controller;
            chassis.robot.driveLF.setTargetPosition(encoderValue);
            chassis.robot.driveRF.setTargetPosition(encoderValue);
            chassis.robot.driveLR.setTargetPosition(encoderValue);
            chassis.robot.driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));


            }


            // Stop all motion;
            chassis.robot.driveLF.setPower(0);
            chassis.robot.driveRF.setPower(0);
            chassis.robot.driveLR.setPower(0);
            chassis.robot.driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public VectorF objectCheck(double timeoutS) {

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();

            CameraDevice.getInstance().setFlashTorchMode(true);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {

                blockLoc = vuforiaCom.executeDetection();
            }

            CameraDevice.getInstance().setFlashTorchMode(false);


        }
        return blockLoc;
    }

    public void encoderDriveObjectCheck(double speed,
                                        int encoderValue,
                                        double timeoutS, boolean pid) {
        double correction = 0;

        resetEncoders();

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            // Determine new target position, and pass to motor controller;
            chassis.robot.driveLF.setTargetPosition(encoderValue);
            chassis.robot.driveRF.setTargetPosition(encoderValue);
            chassis.robot.driveLR.setTargetPosition(encoderValue);
            chassis.robot.driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));


            CameraDevice.getInstance().setFlashTorchMode(true);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));


                blockLoc = vuforiaCom.executeDetection();

                if (blockLoc != null) {

                    break;
                }

            }


            CameraDevice.getInstance().setFlashTorchMode(false);


            // Stop all motion;
            chassis.robot.driveLF.setPower(0);
            chassis.robot.driveRF.setPower(0);
            chassis.robot.driveLR.setPower(0);
            chassis.robot.driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDriveCloseClamps(double speed,
                                        int encoderValue,
                                        double timeoutS, boolean pid, int closeAtValue) {
        double correction = 0;

        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            // Determine new target position, and pass to motor controller;
            chassis.robot.driveLF.setTargetPosition(encoderValue);
            chassis.robot.driveRF.setTargetPosition(encoderValue);
            chassis.robot.driveLR.setTargetPosition(encoderValue);
            chassis.robot.driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));


                //if(liftClaw.robot.lift_check.getDistance(DistanceUnit.MM)<25)
                {
                    //    liftClaw.engageGrabbers();
                    //   curOpMode.sleep(1000);
                    //    break;
                }

                if (chassis.robot.driveLF.getCurrentPosition() >= closeAtValue) {
                    liftClaw.engageGrabbers();
                }

            }


            // Stop all motion;
            chassis.robot.driveLF.setPower(0);
            chassis.robot.driveRF.setPower(0);
            chassis.robot.driveLR.setPower(0);
            chassis.robot.driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderTurn(double speed,
                            int encoderValue,
                            double timeoutS) {

        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            double correction = chassis.pidDrive.performPID(chassis.getAngle());

            // Determine new target position, and pass to motor controller;
            chassis.robot.driveLF.setTargetPosition(-encoderValue);
            chassis.robot.driveRF.setTargetPosition(encoderValue);
            chassis.robot.driveLR.setTargetPosition(-encoderValue);
            chassis.robot.driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(-speed));
                chassis.robot.driveRF.setPower(Math.abs(speed));
                chassis.robot.driveLR.setPower(Math.abs(-speed));
                chassis.robot.driveRR.setPower(Math.abs(speed));


            }

            // Stop all motion;
            chassis.robot.driveLF.setPower(0);
            chassis.robot.driveRF.setPower(0);
            chassis.robot.driveLR.setPower(0);
            chassis.robot.driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            curOpMode.sleep(500);   // optional pause after each move

            chassis.rotation = chassis.getAngle();
            // reset angle tracking on new heading.
            chassis.resetAngle();
        }
    }


    public void resetEncoders() {
        //Reset the encoders
        chassis.robot.driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set all the motors to run using encoders
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafe(double slideSlowPower, double timeoutS) {

        double correction = chassis.pidDrive.performPID(chassis.getAngle());

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS)
        ) {
            chassis.rotation = chassis.getAngle();
            // reset angle tracking on new heading.
            chassis.resetAngle();
            //Front Motors
            chassis.robot.driveLF.setPower(-chassis.slideSlowPower - correction);
            chassis.robot.driveRF.setPower(chassis.slideSlowPower + correction);

            //Rear Motors
            chassis.robot.driveLR.setPower(-slideSlowPower + correction);
            chassis.robot.driveRR.setPower(slideSlowPower - correction);
        }

        // Stop all motion;
        chassis.robot.driveLF.setPower(0);
        chassis.robot.driveRF.setPower(0);
        chassis.robot.driveLR.setPower(0);
        chassis.robot.driveRR.setPower(0);


        // Turn off RUN_TO_POSITION
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    public void GetAndPlacePlatform(double DRIVE_SPEED, double TURN_SPEED, boolean blue) {

        encoderDriveCloseClamps(.4, 800, 10,true,700);

        encoderDrive(.7, -500, 10, false);  // S3

        encoderTurn(TURN_SPEED, 200, 10);

        encoderDrive(.7, -1100, 10, false);  // S3

        //resetEncoders();
        //encoderTurn(TURN_SPEED ,1400,10);

        encoderTurn(TURN_SPEED, 1200, 10);

        liftClaw.disengageGrabbers();

        curOpMode.sleep(200);     // pause for servos to move

        encoderTurn(TURN_SPEED, 300, 10);

        encoderDrive(.3, 800, 10, false);  // S3

        //Turn right to straighten robot to platform
        encoderTurn(TURN_SPEED, -300, 10);


    }

    public void GetCenterBlock(boolean blue) {

        encoderDrive(.4, 700, 10, true);  // S1: Forward 47 Inches with 5 Sec timeout

        chassis.robot.leftGuide.setPosition(.6);
        chassis.robot.rightGuide.setPosition(.4);
        curOpMode.sleep(500);
        //chassis.robot.leftGuide.setPosition(0);
        //chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();

        encoderDrive(.5, -400, 10, true);  // S1: Forward 47 Inches with 5 Sec timeout

        chassis.rotate(87, .4);

    }

    public void GetRightOrLeftBlock(double TURN_SPEED, boolean blue, boolean right) {

        if (right) {
            chassis.rotate(-45, .4);
        } else {
            chassis.rotate(45, .4);

        }

        encoderDrive(.2, 500, 10, false);  // S1: Forward 47 Inches with 5 Sec timeout


        if (right) {
            chassis.rotate(45, .4);
        } else {
            chassis.rotate(-45, .4);
        }

        chassis.robot.leftGuide.setPosition(.3);
        chassis.robot.rightGuide.setPosition(.7);

        encoderDrive(.2, 600, 10, false);  // S1: Forward 47 Inches with 5 Sec timeout

        chassis.robot.leftGuide.setPosition(.6);
        chassis.robot.rightGuide.setPosition(.4);
        curOpMode.sleep(500);
        //auto.chassis.robot.leftGuide.setPosition(0);
        //auto.chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();

        encoderDrive(.5, -600, 10, true);  // S1: Forward 47 Inches with 5 Sec timeout


        if (blue) {
            encoderTurn(TURN_SPEED, 800, 10);
        } else {
            encoderTurn(TURN_SPEED, -800, 10);
        }


    }

    //Drive to common position for start of approach to platform
    public void DriveToPlatformPosition(double TURN_SPEED, boolean blue, boolean center, boolean right, boolean setA) {

        int blockOffset;
        if (setA) {
            blockOffset = 0;
        } else {
            blockOffset = 1200;
        }

        if(center)
        {
            encoderDrive(1, 3300 + blockOffset, 10, true);
        }
        else {
            if (blue) {

                //First check if we are driving from Right or Left block position
                if (right) {

                    encoderDrive(1, 3600 + blockOffset, 10, true);

                } else {
                    encoderDrive(1, 3000 + blockOffset, 10, true);

                }
            } else {
                if (right) {
                    encoderDrive(1, 3000 + blockOffset, 10, true);
                } else {
                    encoderDrive(1, 3600 + blockOffset, 10, true);
                }

            }
        }

        if(blue) {
            encoderTurn(TURN_SPEED, -800, 10);

        }
        else
        {
            encoderTurn(TURN_SPEED, 800, 10);
        }

        chassis.robot.leftGuide.setPosition(0);
        chassis.robot.rightGuide.setPosition(1);
    }


    public void encoderDriveAndLift(double speed,
                             int encoderValue, int liftPosition, double liftSpeed,
                             double timeoutS, boolean pid) {

        double correction = 0;


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = chassis.pidDrive.performPID(chassis.getAngle());
            }

            // Determine new target position, and pass to motor controller;
            chassis.robot.driveLF.setTargetPosition(encoderValue);
            chassis.robot.driveRF.setTargetPosition(encoderValue);
            chassis.robot.driveLR.setTargetPosition(encoderValue);
            chassis.robot.driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.robot.driveLF.setPower(Math.abs(speed));
            chassis.robot.driveRF.setPower(Math.abs(speed));
            chassis.robot.driveLR.setPower(Math.abs(speed));
            chassis.robot.driveRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.



            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));



            }


            // Stop all motion;
            chassis.robot.driveLF.setPower(0);
            chassis.robot.driveRF.setPower(0);
            chassis.robot.driveLR.setPower(0);
            chassis.robot.driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void unsetEncoders()
    {
        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
