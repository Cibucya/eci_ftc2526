package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBot {
    // --- Hardware Members ---
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx frontShoot;
    private CRServo leftPush, rightPush, centerPush;

    // --- State Management ---
    public enum RobotState { IDLE, TRAVELING, ROTATING }
    private RobotState currentState = RobotState.IDLE;

    // --- Robot Geometry & Physics Constants ---
    // Update TRACK_WIDTH_MM by measuring the distance between the center of the left and right wheels.
    public static final double WHEEL_DIAMETER_MM = 100.0;
    public static final double TPR_DRIVE = 1120.0;
    public static final double TRACK_WIDTH_MM = 460.0;

    // --- Drive Constants ---
    private static final double CUBIC_WEIGHT = 0.7; // 70% Cubic, 30% Linear blend
    private double speedMultiplier = 1.0;           // Used for Slow Mode

    // Pre-calculated constant for efficiency
    private static final double TICKS_PER_MM = TPR_DRIVE / (Math.PI * WHEEL_DIAMETER_MM);

    public HardwareBot() { }

    /**
     * Initialize the robot hardware.
     * @param ahwMap The HardwareMap from the OpMode.
     */
    public void init(HardwareMap ahwMap) {
        // Initialize Motors as DcMotorEx for enhanced features
        leftDrive   = ahwMap.get(DcMotorEx.class, "motor_left");
        rightDrive  = ahwMap.get(DcMotorEx.class, "motor_right");
        frontShoot  = ahwMap.get(DcMotorEx.class, "motor_shoot");

        // Initialize Servos
        leftPush    = ahwMap.get(CRServo.class, "servo_left_push");
        rightPush   = ahwMap.get(CRServo.class, "servo_right_push");
        centerPush  = ahwMap.get(CRServo.class, "servo_center_push");

        // Set Directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontShoot.setDirection(DcMotor.Direction.REVERSE);

        leftPush.setDirection(CRServo.Direction.FORWARD);
        rightPush.setDirection(CRServo.Direction.REVERSE);
        centerPush.setDirection(CRServo.Direction.FORWARD);

        // Set Zero Power Behavior (Brake makes Autonomous much more accurate)
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset system to a clean state
        stopAndReset();
    }

    /**
     * Resets encoders and returns the robot to an IDLE state.
     */
    public void stopAndReset() {
        setDrivePower(0, 0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Immediately return to a state where encoders can be used
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentState = RobotState.IDLE;
    }

    /**
     * Logic to be called repeatedly in the OpMode loop.
     * Automatically handles transitions from moving to idle.
     */
    public void update() {
        if (currentState != RobotState.IDLE) {
            // Check if motors have reached their targets
            if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
                stopAndReset();
            }
        }
    }

    /*
     * Drive the robot using a gamepad
     */
    public void driveBotLinear(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Drive the robot with exponential scaling and a sensitivity blend.
     * @param drive Input from joystick Y (forward/back)
     * @param turn  Input from joystick X (left/right)
     * @param isSlowMode If true, reduces overall power for precision tasks.
     */
    public void driveBotExponential(double drive, double turn, boolean isSlowMode) {
        // 1. Set the speed multiplier (Slow Mode = 30% power)
        speedMultiplier = isSlowMode ? 0.3 : 1.0;

        // 2. Apply exponential scaling to both inputs
        double scaledDrive = scaleInput(drive) * speedMultiplier;
        double scaledTurn  = scaleInput(turn) * speedMultiplier;

        // 3. Pass to the standard drive logic
        driveBotLinear(scaledDrive, scaledTurn);
    }

    /**
     * Mathematical helper to blend linear and cubic inputs.
     * This provides a "soft" center for precision but keeps it responsive.
     */
    private double scaleInput(double value) {
        // Equation: f(x) = (w * x^3) + ((1 - w) * x)
        return (CUBIC_WEIGHT * Math.pow(value, 3)) + ((1 - CUBIC_WEIGHT) * value);
    }

    /**
     * Travel a specific distance in millimeters.
     */
    public void startTravel(int distanceMm, double power) {
        if (currentState != RobotState.IDLE) return; // Ignore if busy

        int ticksNeeded = (int) Math.round(distanceMm * TICKS_PER_MM);

        // For linear travel, both wheels move forward together
        setDriveTargets(ticksNeeded, ticksNeeded);
        executeMove(power);
        currentState = RobotState.TRAVELING;
    }

    /**
     * Rotate the robot using the track-width geometry.
     * @param degrees Positive for right turn, negative for left.
     */
    public void startRotateBot(double degrees, double power) {
        if (currentState != RobotState.IDLE) return;

        // Math: Wheel Distance = Robot Circumference * (degrees / 360)
        double wheelDistanceMm = (Math.PI * TRACK_WIDTH_MM) * (degrees / 360.0);
        int ticksNeeded = (int) Math.round(wheelDistanceMm * TICKS_PER_MM);

        // For rotation, wheels move in opposite directions
        setDriveTargets(ticksNeeded, -ticksNeeded);
        executeMove(power);
        currentState = RobotState.ROTATING;
    }

    // --- Internal Helpers ---

    private void setDriveTargets(int leftDelta, int rightDelta) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + leftDelta);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + rightDelta);
    }

    private void executeMove(double power) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(Math.abs(power), Math.abs(power));
    }

    public void setDrivePower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void setShootPower(double power) {
        // Clamp power between -1.0 and 1.0
        frontShoot.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void setPushPower(double leftPwr, double centerPwr, double rightPwr) {
        leftPush.setPower(leftPwr);
        centerPush.setPower(centerPwr);
        rightPush.setPower(rightPwr);
    }

    public RobotState getState() {
        return currentState;
    }
}