package org.firstinspires.ftc.teamcode;

/**
 *      LiftClawCommon  This class contains methods for operating the
 *
 *   Assumptions:
 *   .  The lowest point of the lift just clears the floor when the lift is at its lowest
 *      point and is just low enough to go under the skybridge.
 *   .  When open, the claw is about 5.5" above the floor and clears a stone resting on
 *      the floor.
 *   .  The stone must be raised about 3" above the floor to clear the foundation when
 *      placing the first stone.
 *   .  The first stone must be lowered about 0.75" when placing it on the foundation.
 *   .  After placing the stone, when claw is opened it clears the top tabs.
 *   .  The lift needs to be lowered to its lowest level to go under the skybridge.
 *   .  All of the above apply to the first stone.  For the subsequent stones, the height
 *      when placing the stone needs to be adjusted by 4" per stone already in place.
 *   Trajectory to a stone.
 *   .  Claw does not need to be opened until in proximity with stone.  If the robot
 *      starts on the building side of the skybridge, the lift MUST NOT be raised.
 *   .  At the appropriate time, the claw must be opened to clear the stone.
 *   Picking up a stone (assuming the claw has been positioned over the stone).
 *   .  With the lift at its lowest point, the claw only needs to be opened.
 *   .  To pick up the stone, the claw must be closed.
 *   .  The claw should be raised 0.25" to a carrying level which will allow moving under
 *      the skybridge.
 *   Trajectory to the foundation.
 *   .  On approching the foundation the claw should be raised 3.0" to clear the height
 *      of the foundation plus the height of the tabs. It then must be raised in 4"
 *      increments to clear previously deposited stones in the tower being built.
 *   Dropping the stone.
 *   .  The claw should be lowered 0.75" to place the stone on the foundation or stones
 *      in a tower.
 *   .  The claw should release the stone.
 *   .  The claw should be raised 3" to clear the tabs on top of the stone.  The robot
 *      should now be moved away from the foundation.
 *   Trajectory after releasing a stone.
 *   .  The claw should be lowered to its lowest point for moving under the skybridge
 *      to the next stone.
 *   Servo to English conversion:
 *   .  180 counts on the servo = 1".  All computation uses int values, with the desired
 *      value rounded to the nearest integral count value.
 *
 *
 *    Game controller:
 *    .  left bumper close claw
 *    .  right bumper open claw
 *    .  left trigger pick up stone
 *    .  right trigger deposit stone
 *    .  left joystick up raise lift
 *    .  left joystick down lower lift
 *    .  down arrow engage foundation grabber
 *    .  up arrow disengage foundation grabber
 *    .  gamepad x lower lift one stone height
 *    .  gamepad b lower lift to transit to new stone (base level)
 *    .  gamepad y raise lift to deposit stone
 *       .  press once to raise lift to foundation level
 *       .  press again as many times as needed to clear blocks on an existing tower
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

    private double claw_servoValue = 0;
    private double grabber_servoValue = 0;
    private int block_count;
    private int lift_position;
    private static final double DEFAULT_LIFT_SPEED = 0.5;
    private static final int ONE_INCH = 180;
    private static final int STONE_HEIGHT = 4 * ONE_INCH;
    private static final int FOUNDATION_HEIGHT = 3 * ONE_INCH;

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

    public void executeTeleop(){

        if (Math.abs(curOpMode.gamepad2.left_stick_y)>0) {
            if ((robot.lift_check.getDistance(DistanceUnit.INCH) >= 0) &&
            (curOpMode.gamepad2.left_stick_y > 0)) {  // test for move down request
            }
            else
            {  // moving up is always ok.
                robot.lift.setPower(-curOpMode.gamepad2.left_stick_y);
            }
        }
        else
        {
            robot.lift.setPower(0);
        }

        if(curOpMode.gamepad2.left_bumper && claw_servoValue>0)
        {
            openClaw();
        }
        else if(curOpMode.gamepad2.right_bumper && claw_servoValue!=.6)
        {
            closeClaw();
        }

        if(curOpMode.gamepad2.left_trigger > 0){
            pickUpStone();
        }

        if(curOpMode.gamepad2.right_trigger > 0){
            depositStone();
        }

        if(curOpMode.gamepad2.b)
        {
            returnToBottom();
        }

        if(curOpMode.gamepad2.y)
        {
            liftToDeposit(0);
        }

        if(curOpMode.gamepad2.x)
        {
            lowerOneBlock();
        }

        if(curOpMode.gamepad2.arrowDown)
        {
            engageGrabbers();
        }

        if(curOpMode.gamepad2.arrowUp)
        {
            disengageGrabbers();
        }

        curOpMode.telemetry.addLine().addData("encoder1:", robot.lift.getCurrentPosition());
    }

    private void initLiftClawHardware(){

        robot.init(curOpMode.hardwareMap);
    }

    /**
     *    engageGrabbers()
     *
     *    Lower the pair of grabbers to engage the foundation so it may be dragged into
     *    place.
     *
     */
    public void engageGrabbers(){
        robot.left_grabber.setPosition(0.8);
        robot.right_grabber.setPosition(0.8);
        curOpMode.sleep(50);
    }

    /**
     *    disengageGrabbers()
     *
     *    Raise the pair of grabbers used to engage the foundation.  Note that the
     *    grabbers must be raised to place blocks on the foundation.
     *
     */
    public void disengageGrabbers(){
        robot.left_grabber.setPosition(0);
        robot.right_grabber.setPosition(0);
        curOpMode.sleep(50);
    }

    /**
     *
     *    close Claw
     *
     *    Activate the claw servo so that a stone may be grasped.
     *
     */
    public void closeClaw(){
        robot.claw.setPosition(0.8);
        curOpMode.sleep(50);
    }

    /**
     *
     *    open Claw
     *
     *    Deactivate the claw servo so that a stone may be released.
     *
     */
   public void openClaw(){
        robot.claw.setPosition(0);
        curOpMode.sleep(50);
    }

    /**
     *    pickUpStone()
     *
     *    Compound action:
     *      1.  Move the lift to the bottom location (assumes the stone is on the
     *          floor).
     *      2.  Engage the claw to grasp the stone.
     *      3.  Move the lift up slightly so that the stone will clear the floor.
     *
     */
    public void pickUpStone()
    {
        //  move the lift to 0 to position the claw, engage the claw, move lift to 0.5"
        lift_position = 0;  //  move from wherever to 0
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        closeClaw();
        lift_position = 200;  //  0.5" clearance under the stone
        encoderDrive(1,lift_position,15);
    }

    /**
     *    liftToDeposit(level)
     *
     *    Compound action:
     *      1.  Move the lift so that the bottom of the stone will clear:
     *              a.  the side of the foundation and the lugs on the foundation
     *                 or
     *              b.  one block higher than its previous position.
     *                 or
     *              c.  if level != 0 the height computed using level as the
     *                  number of blocks to clear (possibly used by the
     *                  autonomous mode).
     *          Note:  When operating manually, the one block higher option
     *          allows the operator to push a controller button as many times
     *          as needed to clear the top of a tower under construction.
     *      2.  Disengage the claw to release the stone.
     */
    public void liftToDeposit(int level)  //  move arm up to deposit stone
    {
        if (block_count == 0){
            lift_position = FOUNDATION_HEIGHT + STONE_HEIGHT * level;  // set to 3", and if level
            //        is not 0, the number of blocks higher that that.
            encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
            block_count = 1;
        }
        else
        {
            lift_position += STONE_HEIGHT;  //  up 4" more
            encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
            block_count += 1;
        }
    }

    /**
     *    lowerOneBlock()
     *
     *    Move the lift down the height of one stone.
     */
    public void lowerOneBlock(){  // move lift down one block
        lift_position -= STONE_HEIGHT;  // down 4"
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        block_count -= 1;
    }

    /**
     *    depositStone()
     *
     *    Compound action:
     *    1.  Lower the lift so that the stone is close to its target position.
     *    2.  Disengage the claw so that the stone is released.
     *
     */
    public void depositStone()
    {
        //  move the lift down to position the claw, disengage the claw, move up
        lift_position -= 132;  // down 0.75"
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        openClaw();
    }

    /**
     *    returnToBottom()
     *
     *    Move the lift to its lowest position.
     *
     */
    public void returnToBottom(){
        lift_position = 0;
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        block_count = 0;
    }

    /**
     *   encoderDrive()
     *
     *   Drive the lift to the level requested by encoderValue.
     *
     * @param speed
     * @param encoderValue
     * @param timeoutS
     */
    public void encoderDrive(double speed,
                             int encoderValue,
                             double timeoutS) {

        //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)

            // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            robot.lift.setTargetPosition(encoderValue);
            int currentPosition = robot.lift.getCurrentPosition();

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
            if (encoderValue > currentPosition) {  // if going up, no need to check
                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        robot.lift.isBusy()) {

                    // Display it for the driver.

                    curOpMode.telemetry.addData("Path2", "Running at %7d",
                            robot.lift.getCurrentPosition()
                    );
                    curOpMode.telemetry.update();
                }
            }
            else
            {  // moving down, don't pass zero!
                int tempTarget = Math.max(currentPosition - 5, 0);
                while ((tempTarget > encoderValue) &&
                        (robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)){
                    while (curOpMode.opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            robot.lift.isBusy()) {

                        // Display it for the driver.

                        curOpMode.telemetry.addData("Path2", "Running at %7d",
                                robot.lift.getCurrentPosition()
                        );
                        curOpMode.telemetry.update();
                    }
                    tempTarget = Math.max(tempTarget -5, 0);
                }
            }
            // Stop all motion;
            robot.lift.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

}
