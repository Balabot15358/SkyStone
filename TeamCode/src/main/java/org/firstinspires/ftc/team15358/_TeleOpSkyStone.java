
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop")
//@Disabled
public class _TeleOpSkyStone extends LinearOpMode {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
        LiftClawCommon liftClaw = new LiftClawCommon(this);
        ChassisCommon chassis = new ChassisCommon(this);

        liftClaw.robot.claw.setPosition(.8);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.left_trigger>0)
            {
                chassis.robot.leftGuide.setPosition(.6);
                chassis.robot.rightGuide.setPosition(.4);

                //sleep(500);
                //liftClaw.pickUpStone();
                // chassis.robot.leftGuide.setPosition(0);
               // chassis.robot.rightGuide.setPosition(1);
            }
            else
            {
                chassis.robot.leftGuide.setPosition(0);
                chassis.robot.rightGuide.setPosition(1);
            }


            if(gamepad1.left_bumper) {
                liftClaw.engageGrabbers();
            }

            if (gamepad1.right_bumper) {
                liftClaw.disengageGrabbers();
            }

            liftClaw.executeTeleop();
            chassis.executeTeleop();

            //vuforiaCom.executeDetection();
            telemetry.update();
        }
    }

}
