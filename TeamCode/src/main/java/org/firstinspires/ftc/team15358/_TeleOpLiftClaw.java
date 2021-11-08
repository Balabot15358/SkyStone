
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop LiftClaw")
@Disabled
public class _TeleOpLiftClaw extends LinearOpMode {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
        LiftClawCommon liftClaw = new LiftClawCommon(this);
        AutoCommon auto = new AutoCommon(this);
        ChassisCommon chassis = new ChassisCommon(this);
        boolean Deadman_Switch; /* HT 15358 this is for endgamespin so the code doesn't run too long*/


        liftClaw.robot.sideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftClaw.robot.sideLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.dpad_up) {
                liftClaw.robot.sideLift.setPower(.5);
            } else if (gamepad1.dpad_down) {
                liftClaw.robot.sideLift.setPower(-.5);
            } else {
                liftClaw.robot.sideLift.setPower(0);
            }

            if (gamepad1.left_bumper) {
                liftClaw.robot.sideClaw.setPosition(0);
            } else if (gamepad1.right_bumper) {
                liftClaw.robot.sideClaw.setPosition(.8);
            }

            if (gamepad1.y) {
                auto.getBlockFromSide(2);
            }

            if (gamepad1.x)
            {
                auto.dropBlockFromSide(2);
            }

            telemetry.addData("Side Lift:", liftClaw.robot.sideLift.getCurrentPosition());
            telemetry.addData("Distance:", chassis.robot.leftCheck.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}
