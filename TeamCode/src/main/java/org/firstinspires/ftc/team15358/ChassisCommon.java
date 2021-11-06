
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Func;

public class ChassisCommon {

    public ChassisHardware robot = new ChassisHardware();
    LiftClawCommon liftClaw = null;
    // LiftClawHardware(); /* HT 15358 */

    // LiftClawCommon(); /* HT 15358 */

    private double servoValue = 0;

    private LinearOpMode curOpMode = null;

    Orientation angles;

    Orientation lastAngles = new Orientation();
    double globalAngle, currentAngle, power = .30, correction, rotation;
    PIDController pidRotate, pidDrive;

    double powerRightRear;
    double powerLeftRear;
    double powerLeftFront;
    double powerRightFront;


    double yVal = 0;
    double xVal = 0;

    double turnVal = 0;
    double diagVal = 0;


    double min = 0;
    double setPointAngle = 0;

    double slowPower = .15;
    double slideSlowPower = .15;

    double max = 1;
    double turnMax = 1;

    public ChassisCommon(LinearOpMode owningOpMode) {

        curOpMode = owningOpMode;
        initHardware();
        liftClaw = new LiftClawCommon(curOpMode);

        pidRotate = new PIDController(0, 0, 0);
        pidDrive = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(setPointAngle);
        pidDrive.setOutputRange(0, .9);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();

        resetAngle();

        curOpMode.telemetry.addData("Mode", "calibrating...");
        curOpMode.telemetry.update();


    //    while (!curOpMode.isStopRequested() && !robot.imu.isGyroCalibrated()) {
    //        curOpMode.sleep(50);
     //       curOpMode.idle();
     //   }

        composeTelemetry();
        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        curOpMode.telemetry.update();
    }


    public void executeTeleop() {


        if (curOpMode.gamepad1.y) {
            ReturnToZero();
        }


        yVal = -curOpMode.gamepad1.left_stick_y;
        xVal = curOpMode.gamepad1.left_stick_x;


        if (Math.abs(yVal) < .7) {
            yVal = (yVal * .71);
        } else {
            yVal = ((Math.abs(yVal) * 1.67) - .67) * Math.signum(yVal);
            ;
        }

        if (Math.abs(xVal) < .7) {
            xVal = (xVal * .71);
        } else {
            xVal = ((Math.abs(xVal) * 1.67) - .67) * Math.signum(xVal);
            ;
        }


        correction = 0;


        boolean yDir = false;

        if (Math.abs(yVal) > Math.abs(xVal)) {

            yDir = true;
        }

        if (Math.abs(yVal) > 0 && Math.abs(yVal) < min) {

            yVal = Math.signum(yVal) * min;
        }

        if (Math.abs(xVal) > 0 && Math.abs(xVal) < min) {

            xVal = Math.signum(xVal) * min;
        }


        if (Math.abs(yVal) > 0 && Math.abs(yVal) > max) {

            yVal = Math.signum(yVal) * max;
        }

        if (Math.abs(xVal) > 0 && Math.abs(xVal) > max) {

            xVal = Math.signum(xVal) * max;
        }


        double leftTurnCorrection = 0;
        double rightTurnCorrection = 0;

        if (Math.abs(curOpMode.gamepad1.right_stick_x) > 0) {
            if (Math.abs(yVal) == 0) {
                while (Math.abs(curOpMode.gamepad1.right_stick_x) > 0) {

                    turnVal = curOpMode.gamepad1.right_stick_x;

                    if (Math.abs(turnVal) < .7) {
                        turnVal = (turnVal * .71);
                    } else {
                        turnVal = ((Math.abs(turnVal) * 1.67) - .67) * Math.signum(turnVal);
                        ;
                    }

                    if (Math.abs(turnVal) > 0 && Math.abs(turnVal) > turnMax) {

                        turnVal = Math.signum(turnVal) * turnMax;
                    }

                    powerRightRear = -turnVal;
                    powerLeftRear = turnVal;
                    powerLeftFront = turnVal;
                    powerRightFront = -turnVal;

                    robot.driveRR.setPower(powerRightRear);
                    robot.driveLR.setPower(powerLeftRear);
                    robot.driveLF.setPower(powerLeftFront);
                    robot.driveRF.setPower(powerRightFront);
                }

                robot.driveRR.setPower(0);
                robot.driveLR.setPower(0);
                robot.driveLF.setPower(0);
                robot.driveRF.setPower(0);

            } else {

                turnVal = curOpMode.gamepad1.right_stick_x;

                if (Math.abs(turnVal) < .7) {
                    turnVal = (turnVal * .71);
                } else {
                    turnVal = ((Math.abs(turnVal) * 1.67) - .67) * Math.signum(turnVal);
                    ;
                }

                if (Math.abs(turnVal) > 0 && Math.abs(turnVal) > turnMax) {

                    turnVal = Math.signum(turnVal) * turnMax;
                }

                if (turnVal > 0) {
                    rightTurnCorrection = turnVal;
                } else {
                    leftTurnCorrection = turnVal;
                }
            }

            rotation = getAngle();        // reset angle tracking on new heading.
            resetAngle();
        }

        if (yVal > 0 && yDir) {


            //Left motors
            powerLeftRear = yVal + leftTurnCorrection;
            powerLeftFront = yVal + leftTurnCorrection;

            //Right Motors
            powerRightRear = yVal - rightTurnCorrection;
            powerRightFront = yVal - rightTurnCorrection;


        } else if (yVal < 0 && yDir) {
            //Left motors
            powerLeftRear = yVal + rightTurnCorrection;
            powerLeftFront = yVal + rightTurnCorrection;

            //Right Motors
            powerRightRear = yVal - leftTurnCorrection;
            powerRightFront = yVal - leftTurnCorrection;


        }

        //Slide Left/Right
        else if (Math.abs(xVal) > 0) {

            correction = 0;//pidDrive.performPID(getAngle());//*Math.max(Math.abs(yVal),Math.abs(xVal));

            //Front Motors
            powerLeftFront = xVal - correction;
            powerRightFront = -xVal + correction;

            //Rear Motors
            powerRightRear = xVal + correction;
            powerLeftRear = -xVal - correction;


        } else {
            powerRightRear = 0;
            powerLeftRear = 0;
            powerLeftFront = 0;
            powerRightFront = 0;
        }

        if (curOpMode.gamepad1.dpad_up) {
            //Left motors
            powerLeftRear = slowPower;
            powerLeftFront = slowPower;

            //Right Motors
            powerRightRear = slowPower;
            powerRightFront = slowPower;
        } else if (curOpMode.gamepad1.dpad_left) {

            correction = 0;// pidDrive.performPID(getAngle());//*Math.max(Math.abs(yVal),Math.abs(xVal));

            if (correction < 0) {
                correction = Math.abs(correction);

                //Front Motors
                powerLeftFront = -slideSlowPower + correction;
                powerRightFront = slideSlowPower - correction;

                //Rear Motors
                powerRightRear = -slideSlowPower - correction;
                powerLeftRear = slideSlowPower + correction;


            } else {
                //Front Motors
                powerLeftFront = -slideSlowPower - correction;
                powerRightFront = slideSlowPower + correction;

                //Rear Motors
                powerRightRear = -slideSlowPower + correction;
                powerLeftRear = slideSlowPower - correction;
            }

        } else if (curOpMode.gamepad1.dpad_right) {

            correction = 0;//pidDrive.performPID(getAngle());//*Math.max(Math.abs(yVal),Math.abs(xVal));

            //Front Motors
            powerLeftFront = slideSlowPower - correction;
            powerRightFront = -slideSlowPower + correction;

            //Rear Motors
            powerRightRear = slideSlowPower + correction;
            powerLeftRear = -slideSlowPower - correction;
        } else if (curOpMode.gamepad1.dpad_down) {
            //Left motors
            powerLeftRear = -slowPower;
            powerLeftFront = -slowPower;

            //Right Motors
            powerRightRear = -slowPower;
            powerRightFront = -slowPower;
        }


        robot.driveRR.setPower(powerRightRear);
        robot.driveLR.setPower(powerLeftRear);
        robot.driveLF.setPower(powerLeftFront);
        robot.driveRF.setPower(powerRightFront);
    }

    private void initHardware() {

        robot.init(curOpMode.hardwareMap);
    }

    public void ReturnToZero() {

        pidRotate.setTolerance(.01);
        rotate(-angles.firstAngle,.5);

    }


    public void turnToAngle(double speed, double targetAngle)
    {
        double curAngle = angles.firstAngle;

        double error = curAngle-targetAngle;


        while (curOpMode.opModeIsActive())
        {
            powerRightRear = -speed*Math.signum(error);
            powerLeftRear = speed*Math.signum(error);
            powerLeftFront = speed*Math.signum(error);
            powerRightFront = -speed*Math.signum(error);

            robot.driveRR.setPower(powerRightRear);
            robot.driveLR.setPower(powerLeftRear);
            robot.driveLF.setPower(powerLeftFront);
            robot.driveRF.setPower(powerRightFront);

            error =  angles.firstAngle -targetAngle;

            if(Math.abs(error)<=2)
            {
                break;
            }

            curOpMode.telemetry.addData("error", error);
            curOpMode.telemetry.update();
        }

        robot.driveRR.setPower(0);
        robot.driveLR.setPower(0);
        robot.driveLF.setPower(0);
        robot.driveRF.setPower(0);

    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

        setPointAngle=lastAngles.firstAngle;


    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // If input degrees > 359, we cap at 359 with same sign as input.
        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. We compute the p and I
        // values based on the input degrees and starting power level. We compute the tolerance %
        // to yield a tolerance value of about 1 degree.
        // Overshoot is dependant on the motor and gearing configuration, starting power, weight
        // of the robot and the on target tolerance.

        pidRotate.reset();

        double p = Math.abs((float)power*((float)degrees/(float)180));
        double i = p / 100.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(-power, power);
        //pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
        pidRotate.setTolerance((1.0 / Math.abs(degrees)) * 100.0);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (curOpMode.opModeIsActive() && getAngle() == 0)
            {
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);
                //curOpMode.sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.driveLF.setPower(0);
        robot.driveLR.setPower(0);

        robot.driveRF.setPower(0);
        robot.driveRR.setPower(0);



        // wait for rotation to stop.
        curOpMode.sleep(500);
    rotation = getAngle();        // reset angle tracking on new heading.
        resetAngle();
    }



    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        curOpMode.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
        });

        curOpMode.telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    /* HT 15358 some of this code belongs higher up, but these are all endgame related */

        public void endgamespin () /*code by HT 15358 */
        {


            while (curOpMode.gamepad1.a) { /* HT 15358 This is the endgamespin actual code, only active
                     while the gamepad button is depressed */
                    double bumpVal; /* HT 15358 first drive into the build platform to be sure squared up */
                double revbumpAdj;  /* HT 15358 adj for backing away from platform */
                bumpVal = 0.2;
                powerRightRear = bumpVal;
                powerLeftRear = bumpVal;
                powerLeftFront = bumpVal;
                powerRightFront = bumpVal;

                robot.driveRR.setPower(powerRightRear);
                robot.driveLR.setPower(powerLeftRear);
                robot.driveLF.setPower(powerLeftFront);
                robot.driveRF.setPower(powerRightFront);

                /* HT 15358  Next set the build platform clamps */
                liftClaw.engageGrabbers();

                /* HT 15358  Third spin about 120 degrees, maybe absolute value around -40 degrees */

                /* HT 15358 Fourth open platform clamps and claw */
                liftClaw.disengageGrabbers();
                liftClaw.openClaw();

                /* HT 15358 Fifth back away from the platform */
                revbumpAdj = 0.4;
                powerRightRear  = -revbumpAdj;
                powerLeftRear   = -revbumpAdj;
                powerLeftFront  = -revbumpAdj;
                powerRightFront = -revbumpAdj;

                robot.driveRR.setPower(powerRightRear);
                robot.driveLR.setPower(powerLeftRear);
                robot.driveLF.setPower(powerLeftFront);
                robot.driveRF.setPower(powerRightFront);

                /* leftover HT 15358 that we might copy later
                            lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                            globalAngle = 0;

                            setPointAngle=lastAngles.firstAngle;
                */

            }

        } /*end of endgamespin */



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
