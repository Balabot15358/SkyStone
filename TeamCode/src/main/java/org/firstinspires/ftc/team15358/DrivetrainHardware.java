package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel: Front Left drive motor: "left_drive_front"
 * Motor channel: Ront Right drive motor: "right_drive_front"
 * Motor channel: Rear Left drive motor: "left_drive_rear"
 * Motor channel: Rear Right drive motor: "right_drive_rear"
 */
public class DrivetrainHardware
{
    /* Public OpMode members. */
    public DcMotor driveLF = null;
    public DcMotor driveLR = null;
    public DcMotor driveRF = null;
    public DcMotor driveRR = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public DrivetrainHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        driveLF = hwMap.get(DcMotor.class, "left_drive_front");
        driveLR = hwMap.get(DcMotor.class, "right_drive_front");
        driveRF = hwMap.get(DcMotor.class, "left_drive_rear");
        driveRR = hwMap.get(DcMotor.class, "right_drive_rear");
        driveLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveLR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        driveRF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        driveLF.setPower(0);
        driveLR.setPower(0);
        driveRF.setPower(0);
        driveRR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
    }
 }

