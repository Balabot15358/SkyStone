package org.firstinspires.ftc.teamcode;

/**
 *      LiftClawCommon
 *
 *   Assumptions:
 *   .  Robot "claw" is about 2" above the floor when the lift is at its lowest point.
 *   .  The claw must be about 5.5" above the floor to clear a stone resting on the floor.
 *   .  The stone must be raised about 3" above the floor to clear the foundation when placing
 *      the first stone.
 *   .  The first stone must be lowered about 0.75" when placing it on the foundation.
 *   .  After placing the stone, the claw must be raised 3" to clear the top tabs. (this needs to
 *      be checked).
 *   .  The lift needs to be lowered to go under the bridge.  (the height needs to be determined.)
 *   .  All of the above apply to the first stone.  For the subsequent stones, the height when
 *      placing the stone needs to be adjusted by 4" per stone already in place.
 *   Trajectory to a stone.
 *   .  Claw does not need to be raised until in proximity with stone.  If the robot starts on
 *      the building side of the skybridge, the claw MUST NOT be raised.
 *   .  At the appropriate time, the claw must be raised a minimum of 3.5" to clear the stone.
 *   Picking up a stone (assuming the claw has been positioned over the stone).
 *   .  Tthe claw must drop to its lowest point (2" above the floor)
 *   .  The claw must be closed
 *   .  The claw should be raised 0.5" to a carrying level which will allow moving under the
 *      skybridge.
 *   Trajectory to the foundation.
 *   .  On approching the foundation the claw should be raised 2.5" to clear the height of the
 *      foundation plus the height of the tabs, plus the height of any previous stones.
 *   Dropping the stone.
 *   .  The claw should be lowered 0.75" to place the stone on the foundation.
 *   .  The claw should release the stone.
 *   .  The claw should be raised 3" to clear the tabs on top of the stone.
 *   Trajectory after releasing a stone.
 *   .  The claw should be lowered to its lowest point for moving under the skybridge to the
 *      next stone.
 *   Servo to English conversion:
 *   .  1" = 88.2 counts on the servo.  All computation uses int values, with the desired value
 *      rounded to the nearest integral count value.
 *
 *
 *    Game controller:
 *    .  left bumper open claw
 *    .  right bumper close claw
 *    .  left trigger pick up stone
 *    .  right trigger deposit stone
 *    .  left joystick up raise lift
 *    .  left joystick down lower lift
 *    .  gamepad a raise lift to pick up stone
 *    .  gamepad b lower lift to transit to new stone
 *    .  gamepad y raise lift to deposit stone
 *
 *
 */

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftClawCommon {

    public LiftClawHardware robot = new LiftClawHardware();

    private double servoValue=0;
    private int block_count;
    private int lift_position;
    private static final double DEFAULT_LIFT_SPEED = 0.5;

    private LinearOpMode curOpMode=null;

    private ElapsedTime     runtime = new ElapsedTime();

    public LiftClawCommon(LinearOpMode owningOpMode){

        curOpMode=owningOpMode;
        initLiftClawHardware();

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        block_count = 0;
        lift_position = 0;
    }
    //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)
    public void executeTeleop(){

        if (Math.abs(curOpMode.gamepad2.left_stick_y)>0)
        {
            robot.lift.setPower(-curOpMode.gamepad2.left_stick_y);
        }
        else
        {
            robot.lift.setPower(0);
        }

        if(curOpMode.gamepad2.left_bumper && servoValue>0)
        {
            openClaw();
        }
        else if(curOpMode.gamepad2.right_bumper && servoValue!=.6)
        {
            closeClaw();
        }

        if(curOpMode.gamepad2.left_trigger > 0){
            pickUpStone();
        }

        if(curOpMode.gamepad2.right_trigger > 0){
            depositStone();
        }

        if(curOpMode.gamepad2.a)
        {
            liftToRetrieve();
        }

        if(curOpMode.gamepad2.b)
        {
            returnToBottom();
        }

        if(curOpMode.gamepad2.y)
        {
            liftToDeposit();
        }

        if(curOpMode.gamepad2.x)
        {
            block_count = 0;
        }

        curOpMode.telemetry.addLine().addData("encoder1:", robot.lift.getCurrentPosition());
    }

    private void initLiftClawHardware(){

        robot.init(curOpMode.hardwareMap);
    }

    public void closeClaw(){
        robot.claw.setPosition(0.8);
        curOpMode.sleep(50);
    }

    public void openClaw(){
        robot.claw.setPosition(0);
        curOpMode.sleep(50);
    }

    public void liftToRetrieve()  //  move arm up to obtain new stone
    {
        //  move the lift to 3.5" to clear the tabs on the stone.
        lift_position = 309;
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
    }

    public void pickUpStone()
    {
        //  move the lift to 0 to position the claw, engage the claw, move lift to 0.5"
        lift_position = 0;  //  move from wherever to 0
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        closeClaw();
        lift_position = 200;  //  0.5" clearance under the stone
        encoderDrive(1,lift_position,15);
    }

    public void liftToDeposit()  //  move arm up to deposit stone
    {
        if (block_count == 0){
            lift_position = 530;  // set to 3"
            encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
            block_count = 1;
        } else {
            lift_position = 530 + block_count * 740;  //  up 4" more
            encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
            block_count += 1;
        }
    }

    public void depositStone()
    {
        //  move the lift down to position the claw, disengage the claw, move up
        lift_position -= 132;  // down 0.75"
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        openClaw();
    }

    public void returnToBottom(){
        lift_position = 0;
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
    }

    public void encoderDrive(double speed,
                             int encoderValue,
                             double timeoutS) {

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            robot.lift.setTargetPosition(encoderValue);

            // Turn On RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.lift.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    robot.lift.isBusy()) {

                // Display it for the driver.

                curOpMode.telemetry.addData("Path2",  "Running at %7d",
                        robot.lift.getCurrentPosition()
                );
                curOpMode.telemetry.update();
            }

            // Stop all motion;
            robot.lift.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }



}
