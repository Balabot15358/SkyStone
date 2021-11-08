
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class AutoCommon_ALT {

    public AutoHardware robot = null;
    public ChassisCommon chassis = null;
    public LiftClawCommon liftClaw = null;
    public VuforiaSkyStoneCommon vuforiaCom = null;


    public int BASE_DISTANCE = 2300;
    public int SKY_OFFSET_DISTANCE = 1040;
    public int ADJUSTMENT_DISTANCE = 0;
    public int ARM_OFFSET = -173;
    public int BLOCK_OFFSET_DISTANCE=346;

    public VectorF blockLoc = null;
    public CameraDevice vufCam = null;

    private ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode curOpMode = null;
    Map<Integer, Integer> blockLocations = new HashMap<Integer,Integer>();
    public ArrayList<Integer> patternA = new ArrayList<Integer>();
    public ArrayList<Integer> patternB = new ArrayList<Integer>();
    public ArrayList<Integer> patternC = new ArrayList<Integer>();

    public AutoCommon_ALT(LinearOpMode owningOpMode) {



        curOpMode = owningOpMode;

        chassis = new ChassisCommon(curOpMode);
        liftClaw = new LiftClawCommon(curOpMode);
        robot = new AutoHardware();
        vuforiaCom = new VuforiaSkyStoneCommon(curOpMode);
        robot.init(curOpMode.hardwareMap);

        
        liftClaw.robot.sideClaw.setPosition(0.1);
        vufCam = CameraDevice.getInstance();

        liftClaw.robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blockLocations.put(1,BASE_DISTANCE - (1*BLOCK_OFFSET_DISTANCE));
        blockLocations.put(2,BASE_DISTANCE - (0*BLOCK_OFFSET_DISTANCE));
        blockLocations.put(3,BASE_DISTANCE + (1*BLOCK_OFFSET_DISTANCE));
        blockLocations.put(4,BASE_DISTANCE + (2*BLOCK_OFFSET_DISTANCE));
        blockLocations.put(5,BASE_DISTANCE + (3*BLOCK_OFFSET_DISTANCE));
        blockLocations.put(6,BASE_DISTANCE + (4*BLOCK_OFFSET_DISTANCE));


        patternA.add(1);
        patternA.add(4);
        patternA.add(2);
        patternA.add(3);
        patternA.add(5);
        patternA.add(6);

        patternB.add(2);
        patternB.add(5);
        patternB.add(1);
        patternB.add(3);
        patternB.add(4);
        patternB.add(6);

        patternC.add(3);
        patternC.add(6);
        patternC.add(1);
        patternC.add(2);
        patternC.add(4);
        patternC.add(5);
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

            double currentPower = speed;

            int decelStart = (int)(encoderValue*.75);
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


                chassis.robot.driveLF.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRF.setPower(Math.abs(currentPower + correction));
                chassis.robot.driveLR.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRR.setPower(Math.abs(currentPower + correction));

              // currentPower=rampUpDown(speed,currentPower,.2,chassis.robot.driveLF.getCurrentPosition(),decelStart);


                if(pid) {
                    correction = chassis.pidDrive.performPID(chassis.getAngle());

                    if(correction>0)
                    {

                    }


                    if(encoderValue<0) {
                        correction=correction*(-1);
                    }
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


    public void encoderDriveToDistance(double speed,
                             int encoderValue,
                             double timeoutS, double distance, boolean pid) {
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

            double currentPower = speed;

            int decelStart = (int)(encoderValue*.75);
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


                chassis.robot.driveLF.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRF.setPower(Math.abs(currentPower + correction));
                chassis.robot.driveLR.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRR.setPower(Math.abs(currentPower + correction));

                // currentPower=rampUpDown(speed,currentPower,.2,chassis.robot.driveLF.getCurrentPosition(),decelStart);

                if(liftClaw.robot.lift_check.getDistance(DistanceUnit.CM)<distance)
                {
                    //curOpMode.sleep(500);
                    chassis.robot.leftGuide.setPosition(.6);
                    chassis.robot.rightGuide.setPosition(.4);

                    curOpMode.sleep(500);
                    break;
                }


                if(pid) {
                    correction = chassis.pidDrive.performPID(chassis.getAngle());

                    if(correction>0)
                    {

                    }


                    if(encoderValue<0) {
                        correction=correction*(-1);
                    }
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

    public void encoderDriveToBackDistance(double speed,
                                       int encoderValue,
                                       double timeoutS, double distance, boolean pid) {
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

            double currentPower = speed;

            int decelStart = (int)(encoderValue*.75);
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


                chassis.robot.driveLF.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRF.setPower(Math.abs(currentPower + correction));
                chassis.robot.driveLR.setPower(Math.abs(currentPower - correction));
                chassis.robot.driveRR.setPower(Math.abs(currentPower + correction));

                // currentPower=rampUpDown(speed,currentPower,.2,chassis.robot.driveLF.getCurrentPosition(),decelStart);

                if(chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)<distance)
                {

                    break;
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


    public void encoderDriveWithDrift(double leftSpeed,double rightSpeed,
                             int encoderValue,
                             double timeoutS) {


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

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
            chassis.robot.driveLF.setPower(Math.abs(leftSpeed));
            chassis.robot.driveRF.setPower(Math.abs(rightSpeed));
            chassis.robot.driveLR.setPower(Math.abs(leftSpeed));
            chassis.robot.driveRR.setPower(Math.abs(rightSpeed));

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

                chassis.robot.driveLF.setPower(Math.abs(leftSpeed));
                chassis.robot.driveRF.setPower(Math.abs(rightSpeed));
                chassis.robot.driveLR.setPower(Math.abs(leftSpeed));
                chassis.robot.driveRR.setPower(Math.abs(rightSpeed));


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

                correction = chassis.pidDrive.performPID(chassis.getAngle());

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));


                if(liftClaw.robot.lift_check.getDistance(DistanceUnit.CM)<2)
                {
                    liftClaw.engageGrabbers();

                    break;
                }

                //if (chassis.robot.driveLF.getCurrentPosition() >= closeAtValue) {
                //    liftClaw.engageGrabbers();
                //}

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

    public void driveToBlockPosition(double blockOffset){
        int encoderValue = (int)((blockOffset/25.4)*43.3);


        encoderValue=encoderValue-ARM_OFFSET;

        ADJUSTMENT_DISTANCE=encoderValue;

        curOpMode.telemetry.addData("Offset:", blockOffset);
        curOpMode.telemetry.addData("Encoder Value", encoderValue);

        curOpMode.telemetry.update();

        encoderDrive(.3, encoderValue, 5, false);


    }

    public VectorF encoderStrafe(double power, double timeoutS, int encoderValue, boolean strafeLeft,
                                    boolean pid, boolean objectDetection) {

        chassis.rotation = chassis.getAngle();        // reset angle tracking on new heading.
        chassis.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction=0;

        double currentPower=.1;

        runtime.reset();

        VectorF blockLoc = null;

        resetEncoders();
        CameraDevice.getInstance().setFlashTorchMode(true);

        int setA = 1;
        int setB = 1;

        if(strafeLeft)
        {
            setA=-1;
            setB=1;
        }
        else
        {
            setA=1;
            setB=-1;
        }

        // Determine new target position, and pass to motor controller;
        chassis.robot.driveLF.setTargetPosition(encoderValue*setA);
        chassis.robot.driveRF.setTargetPosition(encoderValue*setB);
        chassis.robot.driveLR.setTargetPosition(encoderValue*setB);
        chassis.robot.driveRR.setTargetPosition(encoderValue*setA);


        // Turn On RUN_TO_POSITION
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        int startDecelerationAt = (int) (encoderValue * .95);


        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy())
        )
        {

            currentPower=power;//rampUpDown(power,currentPower,.1,chassis.robot.driveRR.getCurrentPosition(),startDecelerationAt);

            chassis.robot.driveRR.setPower(currentPower);
            chassis.robot.driveLR.setPower(currentPower);
            chassis.robot.driveLF.setPower(currentPower);
            chassis.robot.driveRF.setPower(currentPower);

            if(objectDetection) {
                blockLoc = vuforiaCom.executeDetection();

                if (blockLoc != null) {
                    break;
                }
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

        CameraDevice.getInstance().setFlashTorchMode(false);

        return blockLoc;
    }


    public double rampUpDown(double maxPower, double curPower, double minPower, int curPosition, int decelStartPosition)
    {

        double returnPower;

            if(Math.abs(curPosition)<Math.abs(decelStartPosition) && curPower<maxPower)
            {
                returnPower=curPower + .01;
            }
            else if(Math.abs(curPosition)>=Math.abs(decelStartPosition) && curPower > minPower)
            {
                returnPower=curPower - .01;
            }
            else
            {
                returnPower=curPower;
            }

            return returnPower;
    }


    public VectorF strafeToDistance(double slideSlowPower, double timeoutS, double distance,
                                        DistanceSensor sensor, boolean objectDetection) {

        chassis.rotation = chassis.getAngle();        // reset angle tracking on new heading.
        chassis.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction=0;

        runtime.reset();

        VectorF blockLoc = null;

        if(objectDetection) {
            CameraDevice.getInstance().setFlashTorchMode(true);
        }

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) )
       {


           if((slideSlowPower>0 && sensor.getDistance(DistanceUnit.CM) > distance) ||
                   (slideSlowPower<0 && sensor.getDistance(DistanceUnit.CM) < distance))
           {
               break;
           }


           correction = 0;//chassis.pidDrive.performPID(chassis.getAngle());
           //Front Motors
           powerLeftFront = -slideSlowPower-correction;
           powerRightFront = slideSlowPower+correction;

           //Rear Motors
           powerRightRear = -slideSlowPower+correction;
           powerLeftRear = slideSlowPower-correction;

           chassis.robot.driveRR.setPower(powerRightRear);
           chassis.robot.driveLR.setPower(powerLeftRear);
           chassis.robot.driveLF.setPower(powerLeftFront);
           chassis.robot.driveRF.setPower(powerRightFront);

           if(objectDetection) {
               blockLoc = vuforiaCom.executeDetection();

               if (blockLoc != null) {
                   break;
               }
           }
        }

        if(objectDetection) {
            CameraDevice.getInstance().setFlashTorchMode(false);
        }

        chassis.robot.driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        return blockLoc;
    }

    public VectorF strafeAwayDistance(double slideSlowPower, double timeoutS, double distance,
                                    DistanceSensor sensor, boolean objectDetection) {

        chassis.rotation = chassis.getAngle();        // reset angle tracking on new heading.
        chassis.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction=0;

        runtime.reset();

        VectorF blockLoc = null;

        CameraDevice.getInstance().setFlashTorchMode(true);

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (sensor.getDistance(DistanceUnit.CM) < distance))
        {

            correction = 0;//chassis.pidDrive.performPID(chassis.getAngle());
            //Front Motors
            powerLeftFront = -slideSlowPower-correction;
            powerRightFront = slideSlowPower+correction;

            //Rear Motors
            powerRightRear = -slideSlowPower+correction;
            powerLeftRear = slideSlowPower-correction;

            chassis.robot.driveRR.setPower(powerRightRear);
            chassis.robot.driveLR.setPower(powerLeftRear);
            chassis.robot.driveLF.setPower(powerLeftFront);
            chassis.robot.driveRF.setPower(powerRightFront);

            if(objectDetection) {
                blockLoc = vuforiaCom.executeDetection();

                if (blockLoc != null) {
                    break;
                }
            }
        }

        CameraDevice.getInstance().setFlashTorchMode(false);

        chassis.robot.driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.robot.driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        return blockLoc;
    }

    public void GetAndPlacePlatform(double DRIVE_SPEED, double TURN_SPEED, boolean blue, boolean withBlock) {

        if(withBlock) {
            encoderDriveAndLift(.3, 200, 500, 1, 10, false);
        }
        else
        {
            if(blue)
            {
                encoderStrafe(-.3,5,-450,false,false,false);
                /**encoderDrive(.5, 200, 5, false);
                encoderTurn(.4, 400, 5);

                encoderDrive(.5, 500, 5, true);
                encoderTurn(.4, -400, 5);**/
            }
            else
            {

                encoderStrafe(.3,5,450,false,false,false);

            }
        }

        encoderDriveCloseClamps(.3, 2000, 10,true,900);

        if(withBlock) {
            liftClaw.openClaw();
            curOpMode.sleep(500);
        }

        else
        {
            curOpMode.sleep(500);
        }

        encoderDrive(.7, -500, 10, false);  // S3


        if(blue) {
            encoderTurn(TURN_SPEED, 200, 10);
        }
        else
        {
            encoderTurn(TURN_SPEED, -200, 10);
        }

        encoderDrive(.7, -900, 10, false);  // S3

        //resetEncoders();
        //encoderTurn(TURN_SPEED ,1400,10);

        if(blue) {
            encoderTurn(TURN_SPEED, 1200, 10);
        }
        else
        {
            encoderTurn(TURN_SPEED, -1200, 10);
        }

        liftClaw.disengageGrabbers();

        //curOpMode.sleep(200);     // pause for servos to move

        if(blue) {
            //encoderTurn(TURN_SPEED, 200, 10);
        }
        else
        {
            //encoderTurn(TURN_SPEED, -200, 10);
        }

        encoderDrive(.3, 350, 10, false);  // S3

        //Turn right to straighten robot to platform
        if(blue) {
            encoderTurn(TURN_SPEED, -50, 10);
        }
        else {
            encoderTurn(TURN_SPEED, 50, 10);
        }

        if(blue)
        {
            chassis.turnToAngle(.1,90);

            encoderStrafe(-.3,5,-500,false,false,false);
        }
        else
        {
            chassis.turnToAngle(.1,-90);
            encoderStrafe(.3,5,500,false,false,false);
        }


        if(withBlock) {
            encoderDrive(.7, -1400, 10, false);
        }
        else
        {
            //strafeToDistance(.5,2);
            encoderDrive(.7, -1700, 10, false);
        }


    }

    public void getBlockFromSide(double timeoutS){
            liftClaw.robot.sideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftClaw.robot.sideClaw.setPosition(.6);

            liftClaw.robot.sideLift.setTargetPosition(-470);
            liftClaw.robot.sideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
            liftClaw.robot.sideLift.setPower(.4);

            while (liftClaw.robot.sideLift.isBusy() && runtime.seconds()<timeoutS );
            {
                liftClaw.robot.sideLift.setPower(.4);

            }

            curOpMode.telemetry.addData("Arm Value:", liftClaw.robot.sideLift.getCurrentPosition());
            curOpMode.telemetry.update();

            liftClaw.robot.sideLift.setPower(0);

            curOpMode.sleep(1000);

            liftClaw.robot.sideClaw.setPosition(0);

            curOpMode.sleep(1000);

            liftClaw.robot.sideLift.setTargetPosition(-50);

            liftClaw.robot.sideLift.setPower(.6);

            runtime.reset();

            while (liftClaw.robot.sideLift.isBusy()&& runtime.seconds()<timeoutS);
            {
                liftClaw.robot.sideLift.setPower(.6);
            }

            liftClaw.robot.sideLift.setPower(0);

            if(liftClaw.robot.sideLift.getCurrentPosition()<-300)
            {
                liftClaw.robot.sideClaw.setPosition(.6);

                liftClaw.robot.sideLift.setTargetPosition(-50);

                liftClaw.robot.sideLift.setPower(.6);

                while (liftClaw.robot.sideLift.isBusy()&& runtime.seconds()<timeoutS);
                {
                    liftClaw.robot.sideLift.setPower(.6);
                }
            }

            liftClaw.robot.sideLift.setPower(0);

            liftClaw.robot.sideLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void dropBlockFromSide(double timeoutS){

        liftClaw.robot.sideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftClaw.robot.sideLift.setTargetPosition(-470);
        liftClaw.robot.sideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftClaw.robot.sideLift.setPower(.6);

        runtime.reset();

        while (liftClaw.robot.sideLift.isBusy() && runtime.seconds()<timeoutS);
        {

        }

        liftClaw.robot.sideLift.setPower(0);

        curOpMode.sleep(500);

        liftClaw.robot.sideClaw.setPosition(.6);

        //curOpMode.sleep(1000);
        liftClaw.robot.sideLift.setTargetPosition(-50);

        liftClaw.robot.sideLift.setPower(.6);

        runtime.reset();
        while (liftClaw.robot.sideLift.isBusy()&& runtime.seconds()<timeoutS);
        {

        }

        liftClaw.robot.sideLift.setPower(0);
    }

    public void GetCenterBlock(double TURN_SPEED, boolean blue) {

        chassis.robot.leftGuide.setPosition(.3);
        chassis.robot.rightGuide.setPosition(.7);

        int forwardValue = (int)((liftClaw.robot.lift_check.getDistance(DistanceUnit.INCH)*44.6)+100);
        encoderDriveToDistance(.3,800, 10, 2.5,false);  // S1: Forward 47 Inches with 5 Sec timeout

        chassis.robot.leftGuide.setPosition(.6);
        chassis.robot.rightGuide.setPosition(.4);
        curOpMode.sleep(1000);
        //chassis.robot.leftGuide.setPosition(0);
        //chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();
        curOpMode.sleep(500);

        int backupValue = (int)((chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)-26)*44.6);

        curOpMode.telemetry.addData("distance", chassis.robot.leftCheck.getDistance(DistanceUnit.INCH));
        curOpMode.telemetry.addData("backup", backupValue);
        curOpMode.telemetry.update();


        encoderDrive(.5, -backupValue, 80, false);  // S1: Forward 47 Inches with 5 Sec timeout

        if(blue) {
            //encoderStrafe(-1,10,-2600,false,false,false);

            encoderTurn(.2,840,5);

        }
        else
        {
            encoderTurn(.2,-840,5);

        }
        encoderDrive(.5,2200,5,false);

        chassis.robot.leftGuide.setPosition(.05);
        chassis.robot.rightGuide.setPosition(.98);


        liftClaw.openClaw();
        //liftClaw.returnToBottom();

        curOpMode.sleep(1000);
       // liftClaw.robot.claw.setPosition(.5);

        //backupValue = (int)((chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)-19)*44.6);

       // encoderDrive(.5, -backupValue, 5,false);  // S1: Forward 47 Inches with 5 Sec timeout

        chassis.turnToAngle(.1,0);
        //liftClaw.robot.claw.setPosition(.5);


            encoderDriveToBackDistance(.5,-4000,10,12,false);
            //encoderStrafe(1, 10, 3700, false, false, false);


       // backupValue = (int)((chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)-12)*44.6);

        //encoderDrive(.2,-backupValue,10,false);

        if(blue) {
            encoderTurn(.2, -840, 5);
        }
        else
        {
            encoderTurn(.2, 840, 5);
        }

        chassis.robot.leftGuide.setPosition(.3);
        chassis.robot.rightGuide.setPosition(.7);

        forwardValue = (int)((liftClaw.robot.lift_check.getDistance(DistanceUnit.INCH)*44.6)+100);
        encoderDriveToDistance(.3,forwardValue,5,2.5,false);

        chassis.robot.leftGuide.setPosition(.6);
        chassis.robot.rightGuide.setPosition(.4);
        curOpMode.sleep(1000);
        //chassis.robot.leftGuide.setPosition(0);
        //chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();
        curOpMode.sleep(500);

        backupValue = (int)((chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)-26)*44.6);

        encoderDrive(.5, -backupValue, 5, false);  // S1: Forward 47 Inches with 5 Sec timeout

        if(blue)
        {
            encoderTurn(.3, 860, 4);
        }
        else {
            encoderTurn(.3, -860, 4);
        }

        encoderDrive(1,3300,5,false);

        encoderDrive(1,-700,5,false);

        curOpMode.sleep(20000);
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

        encoderDriveToDistance(.3, 2000, 10, 2.5,false);  // S1: Forward 47 Inches with 5 Sec timeout

        //chassis.robot.leftGuide.setPosition(0);
        //chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();
        curOpMode.sleep(1000);

        int backupValue = (int)((chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)-25)*44.6);

        encoderDrive(.5, -backupValue, 5,false);  // S1: Forward 47 Inches with 5 Sec timeout


        if(blue)
        {
            encoderTurn(.3,840,5);
        }
        else
        {
            encoderTurn(.3,-840,5);
        }

        if(right) {

            if(blue)
            {

                encoderDrive(.5,2500,5,false);

            }
            else {
                encoderDrive(.5,1500,5,false);
            }
        }
        else
        {
            if(blue)
            {
                encoderDrive(.5,1500,5,false);

            }
            else {
                encoderDrive(.5,2500,5,false);

            }

        }


        chassis.robot.leftGuide.setPosition(.05);
        chassis.robot.rightGuide.setPosition(.98);


        liftClaw.openClaw();

        curOpMode.sleep(1000);


        if(right) {

            if(blue) {
                encoderDriveToBackDistance(.5, -4000, 10, 42, false);
            }
            else
            {
                encoderDriveToBackDistance(.5, -4000, 10, 20, false);
            }
        }
        else
        {
            if(blue)
            {
                encoderDriveToBackDistance(.5, -4000, 10, 20, false);
            }
            else
            {
                encoderDriveToBackDistance(.5, -4000, 10, 42, false);
            }
        }

        // liftClaw.robot.claw.setPosition(.5);

        if(blue)
        {
            encoderTurn(.3,-840,5);
        }
        else
        {
            encoderTurn(.3,840,5);
        }


        chassis.robot.leftGuide.setPosition(.3);
        chassis.robot.rightGuide.setPosition(.7);

        encoderDriveToDistance(.3,1000,5,2.5,false);

        chassis.robot.leftGuide.setPosition(.6);
        chassis.robot.rightGuide.setPosition(.4);
        curOpMode.sleep(1000);
        //chassis.robot.leftGuide.setPosition(0);
        //chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();
        curOpMode.sleep(500);

        backupValue = (int)((chassis.robot.leftCheck.getDistance(DistanceUnit.INCH)-28)*44.6);

        encoderDrive(.5, -backupValue, 5, false);  // S1: Forward 47 Inches with 5 Sec timeout

        if(blue)
        {
            encoderTurn(.4, 860, 4);
        }
        else {
            encoderTurn(.4, -850, 4);
        }
        //chassis.turnToAngle(.1,-90);

        if(right) {

            if(blue)
            {
                encoderDrive(1, 2200, 5, false);
            }
            else {
                encoderDrive(1, 2800, 5, false);
            }
        }
        else
        {
            if(blue)
            {
                encoderDrive(1, 2800, 5, false);
            }
            else {
                encoderDrive(1, 1500, 5, false);
            }
        }
        encoderDrive(1,-700,5,false);

        curOpMode.sleep(20000);
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


    public void runBlocks(ArrayList<Integer> blockPattern)
    {

        double DRIVE_SPEED=1;

        getBlockFromSide(2);

        encoderDrive(DRIVE_SPEED, blockLocations.get(blockPattern.get(0)),15,false);

        dropBlockFromSide(2);


        for(int i=1;i<6;i++)
        {

            encoderDrive(DRIVE_SPEED, - blockLocations.get(blockPattern.get(i)),15,false);

            double currentError =chassis.robot.leftCheck.getDistance(DistanceUnit.CM)-20;



            if(Math.abs(currentError)>5)
            {

                if(currentError>0) {
                    strafeToDistance(.2, 5, 25, chassis.robot.leftCheck, false);
                }
                else
                {
                    strafeToDistance(-.2, 5, 20 ,chassis.robot.leftCheck, false);
                }
            }

            curOpMode.telemetry.addData("Distance:", chassis.robot.leftCheck.getDistance(DistanceUnit.CM));
            curOpMode.telemetry.addData("Error:", currentError);
            curOpMode.telemetry.update();

           curOpMode.sleep(20000);



            getBlockFromSide(2);

            encoderDrive(DRIVE_SPEED,  blockLocations.get(blockPattern.get(i)),15,false);

            dropBlockFromSide(2);

        }

    }


    //Drive to common position for start of approach to platform
    public void DriveToDropPosition(double TURN_SPEED, boolean blue, boolean center, boolean right, boolean setA){
        int blockOffset;
        if (setA) {
            blockOffset = 0;
        } else {
            blockOffset = 1040;
        }

        if(center)
        {


            encoderStrafe(1, 10,2200,false,false ,false);

            curOpMode.sleep(20000);
            encoderDrive(1, 2200 + blockOffset, 10, true);
        }
        else {
            if (blue) {

                //First check if we are driving from Right or Left block position
                if (right) {

                    encoderDrive(1, 2500 + blockOffset, 10, true);

                } else {
                    encoderDrive(1, 2000 + blockOffset, 10, true);

                }
            } else {
                if (right) {
                    encoderDrive(1, 2000 + blockOffset, 10, true);
                } else {
                    encoderDrive(1, 2500 + blockOffset, 10, true);
                }

            }
        }

        liftClaw.openClaw();
        chassis.robot.leftGuide.setPosition(0);
        chassis.robot.rightGuide.setPosition(1);


    }

    //Drive to common position for start of approach to platform
    public void DriveToPickupPosition(double DRIVE_SPEED, boolean blue, boolean center, boolean right, boolean setA) {

        int blockOffset;
        if (setA) {
            blockOffset = 0;
        } else {
            blockOffset = 1040;
        }

             if(center)
        {
            encoderDrive(DRIVE_SPEED, -(2200 + blockOffset), 10, true);
        }
        else {
            if (blue) {
            }
        }

    }

    //Drive to common position for start of approach to platform
    public void PickupBlock(double TURN_SPEED, boolean blue, boolean center, boolean right, boolean setA) {

        if(blue)
        {
            encoderTurn(.5,-790,10);
        }
        else
        {
            encoderTurn(.5,790,10);
        }

        encoderDrive(.4, 700, 10, true);  // S1: Forward 47 Inches with 5 Sec timeout

        chassis.robot.leftGuide.setPosition(.6);
        chassis.robot.rightGuide.setPosition(.4);
        curOpMode.sleep(500);
        //chassis.robot.leftGuide.setPosition(0);
        //chassis.robot.rightGuide.setPosition(1);
        liftClaw.closeClaw();

        encoderDrive(.5, -400, 10, true);  // S1: Forward 47 Inches with 5 Sec timeout

        if(blue) {
            encoderTurn(TURN_SPEED, 790, 10);
        }
        else
        {
            encoderTurn(TURN_SPEED, -790, 10);

        }



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


            liftClaw.robot.lift.setPower(liftSpeed);

            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.robot.driveLF.isBusy() && chassis.robot.driveRF.isBusy()
                            && chassis.robot.driveLR.isBusy() && chassis.robot.driveRR.isBusy()
                    )) {

                chassis.robot.driveLF.setPower(Math.abs(speed - correction));
                chassis.robot.driveRF.setPower(Math.abs(speed + correction));
                chassis.robot.driveLR.setPower(Math.abs(speed - correction));
                chassis.robot.driveRR.setPower(Math.abs(speed + correction));


                if(liftClaw.robot.lift.getCurrentPosition()>=liftPosition)
                {
                    liftClaw.robot.lift.setPower(0);
                }
            }


            // Stop all motion;
            chassis.robot.driveLF.setPower(0);
            chassis.robot.driveRF.setPower(0);
            chassis.robot.driveLR.setPower(0);
            chassis.robot.driveRR.setPower(0);

            if(liftClaw.robot.lift.getCurrentPosition()<liftPosition) {
                while (curOpMode.opModeIsActive() && runtime.seconds() < timeoutS &&
                        liftClaw.robot.lift.getCurrentPosition() < liftPosition) {

                    liftClaw.robot.lift.setPower(liftSpeed);
                }
            }

            // Turn off RUN_TO_POSITION
            chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftClaw.robot.lift.setPower(0);
            //  sleep(250);   // optional pause after each move
        }
    }


    public void unsetEncoders()
    {
        // Stop all motion;
        chassis.robot.driveLF.setPower(0);
        chassis.robot.driveRF.setPower(0);
        chassis.robot.driveLR.setPower(0);
        chassis.robot.driveRR.setPower(0);

        chassis.robot.driveRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.robot.driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
