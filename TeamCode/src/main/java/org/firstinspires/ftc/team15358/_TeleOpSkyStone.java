
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop")
//@Disabled
public class _TeleOpSkyStone extends LinearOpMode {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
        LiftClawCommon liftClaw = new LiftClawCommon(this);
        ChassisCommon chassis = new ChassisCommon(this);
        AutoCommon auto = new AutoCommon(this);

       // boolean Deadman_Switch; /* HT 15358 this is for endgamespin so the code doesn't run too long*/

        liftClaw.robot.claw.setPosition(0);
        liftClaw.disengageGrabbers();

        liftClaw.chassis=chassis;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

      

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

          //while (gamepad1.a) { /* HT 15358 this allows endgamespin to only run while the button is depressed */
               // Deadman_Switch = true;
              //  chassis.endgamespin();
            //}
          // while (!gamepad1.a) { /* Double make sure endgamespin stops when button not depresed */
             //  Deadman_Switch = false;
           //}

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
                chassis.robot.leftGuide.setPosition(.05);
                chassis.robot.rightGuide.setPosition(.98);
            }


            if(gamepad1.left_bumper) {
                liftClaw.engageGrabbers();
            }

            if (gamepad1.right_bumper) {
                liftClaw.disengageGrabbers();
            }

            liftClaw.executeTeleop();
            chassis.executeTeleop();


            if(gamepad1.x)
            {
                auto.dropBlockFromSide(3);
            }

            telemetry.addData("LF:",chassis.robot.driveLF.getCurrentPosition());
            telemetry.addData("RF:",chassis.robot.driveRF.getCurrentPosition());
            telemetry.addData("LR:",chassis.robot.driveLR.getCurrentPosition());
            telemetry.addData("RR:",chassis.robot.driveRR.getCurrentPosition());

            telemetry.addData("Front Distance", liftClaw.robot.lift_check.getDistance(DistanceUnit.CM));
            telemetry.addData("Rear Distance", chassis.robot.leftCheck.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}
