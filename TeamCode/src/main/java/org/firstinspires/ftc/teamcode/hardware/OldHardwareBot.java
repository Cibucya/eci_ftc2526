package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO: check chat with gemini and fix whatever it says to fix
public class OldHardwareBot {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor frontShoot;
    private CRServo leftPush;
    private CRServo rightPush;
    private CRServo centerPush;
    public boolean isTraveling = false;
    public boolean isRotating = false;

    // Drive wheels radius
    public static final double WHEEL_DIAMETER_MM = 50;
    // encoder ticks per revolution for drive motors
    public static final double TPR_DRIVE = 1120;
    // circumference is ~ 157 mm


    /* local OpMode members. */
    HardwareMap hwMap = null;

    // Constructor
    public OldHardwareBot() {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Init motors
        leftDrive = hwMap.get(DcMotor.class, "motor_left");
        rightDrive = hwMap.get(DcMotor.class, "motor_right");
        frontShoot = hwMap.get(DcMotor.class, "motor_shoot");

        leftPush = hwMap.get(CRServo.class, "servo_left_push");
        rightPush = hwMap.get(CRServo.class, "servo_right_push");
        centerPush = hwMap.get(CRServo.class, "servo_center_push");

        // Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontShoot.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        frontShoot.setPower(0);
        isTraveling = false;

        // Set servo directions
        leftPush.setDirection(CRServo.Direction.FORWARD);
        rightPush.setDirection(CRServo.Direction.REVERSE);
        centerPush.setDirection(CRServo.Direction.FORWARD);

        // Set modes
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set zero power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveBot(double drive, double turn) {
        double left = drive + turn;
        double right = drive - turn;

        // Scale the values so that nothing exceeds +- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        setDrivePower(left, right);
    }

    /* Distance is in millimeters, power ranges from -1 to 1 */
    public void startTravel(int distance, double power) {
        // ticksNeeded = (distance * ticks_per_revolution) / (wheel_circumference)
        int ticksNeeded = (int) Math.round((distance * TPR_DRIVE) / (Math.PI * WHEEL_DIAMETER_MM));

        setDriveTargetPosition(
                leftDrive.getCurrentPosition() + ticksNeeded,
                rightDrive.getCurrentPosition() + ticksNeeded
        );

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION, DcMotor.RunMode.RUN_TO_POSITION);

        // Absolute value because the direction is handled by the target
        setDrivePower(Math.abs(power), Math.abs(power));
        isTraveling = true;
    }

    public void updateDriveLogic() {
        if (isTraveling) { // Precise distance
            if (!leftDrive.isBusy() && !rightDrive.isBusy()) { // Achieved destination
                setDrivePower(0, 0);
                setDriveMode(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        DcMotor.RunMode.RUN_USING_ENCODER
                );
                isTraveling = false;
            }
        }
        else if (isRotating) {
            if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                setDrivePower(0, 0);
                setDriveMode(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        DcMotor.RunMode.RUN_USING_ENCODER
                );
                isRotating = false;
            }
        }
    }

    public void startRotateBot(double degrees, double power) {
        int ticksNeeded = (int) Math.round((degrees / 360) * TPR_DRIVE);
        setDriveTargetPosition(
                leftDrive.getCurrentPosition() + ticksNeeded,
                rightDrive.getCurrentPosition() - ticksNeeded
        );

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION, DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(Math.abs(power), Math.abs(power));
        isRotating = true;
    }

    public void setDrivePower(double leftWheel, double rightWheel) {
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }

    public void setDriveMode(DcMotor.RunMode runModeLeft, DcMotor.RunMode runModeRight) {
        leftDrive.setMode(runModeLeft);
        rightDrive.setMode(runModeRight);
    }

    public void setDriveTargetPosition(int leftTarget, int rightTarget) {
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);
    }

    public void setPushPower(double pushLeft, double pushRight, double pushCenter) {
        // Scale the values so that nothing exceeds +- 1.0
        double max = Math.max(Math.abs(pushLeft), Math.max(Math.abs(pushRight), Math.abs(pushCenter)));
        if (max > 1.0) {
            pushLeft /= max;
            pushRight /= max;
            pushCenter /= max;
        }

        leftPush.setPower(pushLeft);
        rightPush.setPower(pushRight);
        centerPush.setPower(pushCenter);
    }

    public void setShootPower(double power) {
        if (Math.abs(power) > 1) {
            power /= Math.abs(power);
        }
        frontShoot.setPower(power);
    }
}
