
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class LiftClawHardware
{
    /* Public OpMode members. */

    public DcMotor  lift  = null;

    public Servo    claw   = null;

    public DistanceSensor lift_check;


    public static final double MID_SERVO       =  0.3 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public LiftClawHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        lift = hwMap.get(DcMotor.class, "lift");

        lift.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        lift.setPower(0);


        // Set motor to run with encoder
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        claw  = hwMap.get(Servo.class, "claw");

        //  Initialize lift distance sensor
        lift_check = hwMap.get(DistanceSensor.class, "lift_check");

    }
}
