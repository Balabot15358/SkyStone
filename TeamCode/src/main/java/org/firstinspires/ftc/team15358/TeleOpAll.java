
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop All")
//@Disabled
public class TeleOpAll extends LinearOpMode {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
        LiftClawCommon liftClaw = new LiftClawCommon(this);
        ChassisCommon chassis = new ChassisCommon(this);
        VuforiaSkyStoneCommon vuforiaCom = new VuforiaSkyStoneCommon(this);
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        // Send telemetry message to signify robot waiting;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            liftClaw.executeTeleop();
            chassis.executeTeleop();

            vuforiaCom.executeDetection();
            telemetry.update();
        }
    }

}
