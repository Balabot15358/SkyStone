
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop Chassis")
@Disabled

public class TeleOpChassis extends LinearOpMode {

    @Override
    public void runOpMode() {


        ChassisCommon chassis = new ChassisCommon(this);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            chassis.executeTeleop();

            telemetry.update();
        }
    }

}
