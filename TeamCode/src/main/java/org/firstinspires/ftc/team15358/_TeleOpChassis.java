
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop Chassis")
@Disabled
public class _TeleOpChassis extends LinearOpMode {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
       // LiftClawCommon liftClaw = new LiftClawCommon(this);
        ChassisCommon chassis = new ChassisCommon(this);
        boolean Deadman_Switch; /* HT 15358 this is for endgamespin so the code doesn't run too long*/


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad2.left_trigger>0 || gamepad2.right_trigger>0 )
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

            chassis.executeTeleop();



            //vuforiaCom.executeDetection();
            telemetry.update();
        }
    }

}
