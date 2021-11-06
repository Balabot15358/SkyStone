
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test Board")
//@Disabled
public class _TestBoard extends LinearOpMode {

    /* Declare OpMode members. */
    double curAngle = 0;
    PIDController pidRotate= new PIDController(0,0,0);
    TestBoardHardware testBoard;

     Telemetry.Item heading;
    Telemetry.Item distance;
    Telemetry.Item color;
    @Override
    public void runOpMode() {
       // LiftClawCommon liftClaw = new LiftClawCommon(this);
        testBoard= new TestBoardHardware();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

       // telemetry.setAutoClear(false);
        testBoard.init(hardwareMap);

        heading= telemetry.addData("Heading",curAngle);
        distance = telemetry.addData("Distance",0);
        color = telemetry.addData("RGBA",0 +" ," +  0 + " ," + 0 + " ," + 0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            curAngle= testBoard.imu.getAngularOrientation().firstAngle;

            heading.setValue(curAngle);
            distance.setValue(Math.round(testBoard.distanceCheck.getDistance(DistanceUnit.INCH)));
            color.setValue(testBoard.colorCheck.red() + " ," +  testBoard.colorCheck.blue() + " ," + testBoard.colorCheck.green() + " ," + testBoard.colorCheck.alpha());

            telemetry.update();

            if (testBoard.digitalTouch.getState() == true) {
               // telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                //telemetry.addData("Digital Touch", "Is Pressed");

                rotateToZero(.5);
            }
        }
    }


    public void rotateToZero(double power)
    {

        pidRotate.reset();

        double p = .003;
        double i = .00003;

        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(0);

        pidRotate.setOutputRange(.05, power);

        pidRotate.setTolerance(.5);
        pidRotate.enable();

        do
        {
            power = pidRotate.performPID(-curAngle); // power will be - on right turn.

            getAngle();
            heading.setValue(curAngle);
            telemetry.update();

        } while (opModeIsActive() && !pidRotate.onTarget());

    }

    private void getAngle()
    {

        curAngle = testBoard.imu.getAngularOrientation().firstAngle;
    }

}
