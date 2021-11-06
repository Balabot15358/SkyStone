package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This is not an OpMode
 *
 * This is class that you put all of your reusable functions for the Drivetrain
 */

public class DrivetrainCommon {

    private LinearOpMode curOpMode;
    private DrivetrainHardware robot;

    private boolean yDir;
    private double correction;
    private double xVal, yVal, turnVal;
    private double pwrRF, pwrRR, pwrLF, pwrLR;
    private double max, min, turnMax;
    
    public DrivetrainCommon(LinearOpMode selOpMode){
        curOpMode = selOpMode;
        robot = new DrivetrainHardware();

        yDir = false;
        correction = 0;
        max = 1;
        turnMax = .5;
        min = .2;
    }

    public void drive(){
        if(Math.abs(yVal)>Math.abs(xVal))
        {

            yDir=true;
        }

        if(Math.abs(yVal)>0 && Math.abs(yVal)<min)
        {

            yVal=Math.signum(yVal)*min;
        }

        if(Math.abs(xVal)>0 && Math.abs(xVal)<min)
        {

            xVal=Math.signum(xVal)*min;
        }


        if(Math.abs(yVal)>0 && Math.abs(yVal)>max)
        {

            yVal=Math.signum(yVal)*max;
        }

        if(Math.abs(xVal)>0 && Math.abs(xVal)>max)
        {

            xVal=Math.signum(xVal)*max;
        }



        if(Math.abs(curOpMode.gamepad1.right_stick_x) > 0)
        {
            while(Math.abs(curOpMode.gamepad1.right_stick_x) > 0) {

                turnVal = curOpMode.gamepad1.right_stick_x;

                if(Math.abs(turnVal)>0 && Math.abs(turnVal)>turnMax)
                {

                    turnVal=Math.signum(turnVal)*turnMax;
                }

                pwrRR = -turnVal;
                pwrLR = turnVal;
                pwrLF = turnVal;
                pwrRF = -turnVal;

                robot.driveRR.setPower(pwrRR);
                robot.driveLR.setPower(pwrLR);
                robot.driveLF.setPower(pwrLF);
                robot.driveRF.setPower(pwrRF);
            }

            robot.driveRR.setPower(0);
            robot.driveLR.setPower(0);
            robot.driveLF.setPower(0);
            robot.driveRF.setPower(0);



            // wait for rotation to stop.
            //curOpMode.sleep(500);

            //rotation = getAngle();
            // reset angle tracking on new heading.
            //resetAngle();

        }
        else if(yVal>0 && yDir) {

            //Left motors
            pwrLR = yVal-correction;
            pwrLF = yVal-correction;

            //Right Motors
            pwrRR = yVal+correction;
            pwrRF = yVal+correction;

        }
        else if(yVal<0 && yDir) {
            //Left motors
            pwrLR = yVal-correction;
            pwrLF = yVal-correction;

            //Right Motors
            pwrRR = yVal+correction;
            pwrRF = yVal+correction;
        }

        //Slide Left/Right
        else if(Math.abs(xVal)>0) {

            //Front Motors
            pwrLF = xVal-correction;
            pwrRF = -xVal+correction;

            //Rear Motors
            pwrRR = xVal+correction;
            pwrLR = -xVal-correction;



        }
        else {
            pwrRR = 0;
            pwrLR = 0;
            pwrLF = 0;
            pwrRF = 0;
        }
    }
}