package org.firstinspires.ftc.team15358;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftControl {
    // Instance Variables
    public DcMotor lift = null; // new, now lift pulley system Drive Motor

    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    private int current_height;
    private int current_count;
    private boolean zero_set;

    // Constructor Declaration of Class
    public LiftControl()
    {
        this.zero_set = false;
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // method 1
    public void setZero()
    {
        int previous_position = lift.getCurrentPosition();
        lift.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        lift.setPower(0);
    }

}
