package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode - Example for controlling robot drive, launcher, and servos.
 *
 * Controls:
 *  - Right joystick: Drive robot (motor_left, motor_right)
 *  - Gamepad A: Toggle launcher motor ON/OFF
 *  - Gamepad B: Trigger ball shot (1 press = 1 shot, max 3 shoots)
 *
 * Field: 24" x 24" (60.96 cm x 60.96 cm)
 */
@TeleOp()
public class DontUseMe extends LinearOpMode {

    // === Hardware ===
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLaunch;

    private Servo servoLeftPush;
    private Servo servoRightPush;
    private CRServo servoPrelaunch;

    // === State variables ===
    private boolean isLauncherOn = false;
    private long lastAPress = 0;
    private long lastBPress = 0;
    private static final int PRESS_DELAY_MS = 200;
    private static final double SERVO_LEFT_PUSH_INIT_POSITION = 0.0;
    private static final double SERVO_RIGHT_PUSH_INIT_POSITION = 0.0;
    private static final double SERVO_PRELAUNCH_INIT_POSITION = 1.0;
    private int ballsToLaunch = 0;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        initHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- Main control loop ---
        while (opModeIsActive()) {
            handleDrive();
            handleLauncherToggle();
            handleShooting();

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Servo", servoLeftPush.getPosition());
            telemetry.addData("Right Servo", servoRightPush.getPosition());
            telemetry.addData("ballsToLaunch", ballsToLaunch);
            telemetry.update();
        }
    }

    /** Initialize motors and servos, directions, and starting positions. */
    private void initHardware() {
        // Map hardware
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLaunch = hardwareMap.dcMotor.get("motor_shoot");

        servoLeftPush = hardwareMap.servo.get("servo_left_push");
        servoRightPush = hardwareMap.servo.get("servo_right_push");
        servoPrelaunch = hardwareMap.crservo.get("servo_center_push");

        // Set motor directions
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLaunch.setDirection(DcMotor.Direction.FORWARD);

        // Set servo directions
        servoLeftPush.setDirection(Servo.Direction.FORWARD);
        servoRightPush.setDirection(Servo.Direction.FORWARD);
        servoPrelaunch.setDirection(CRServo.Direction.FORWARD);

        // Initialize servo positions
        setServoInitialPositions();

        // Reset and enable encoders
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Handle robot driving using right joystick. */
    /** For some reason forward/backward works in the opposite direction. Changing the sign doesn't help. */
    private void handleDrive() {
        double drive = -gamepad1.right_stick_y; // Forward/backward
        double turn = gamepad1.right_stick_x;   // Rotation

        motorLeft.setPower(drive + turn);
        motorRight.setPower(drive - turn);
    }

    /** Toggle launcher motor power when 'A' is pressed. */
    private void handleLauncherToggle() {
        if (gamepad1.a && timeSince(lastAPress) >= PRESS_DELAY_MS) {
            isLauncherOn = !isLauncherOn;
            motorLaunch.setPower(isLauncherOn ? 1 : 0);
            lastAPress = System.currentTimeMillis();
        }
    }

    /** Fire one ball when 'B' is pressed. */
    private void handleShooting() {
        if (gamepad1.b && timeSince(lastBPress) >= PRESS_DELAY_MS) {
            lastBPress = System.currentTimeMillis();
            if (ballsToLaunch < 3) {
                ballsToLaunch++;
            }

            // If have ball to launch and servo is ready to launch the ball
            if (ballsToLaunch > 0) {
                shootBall();
            }

            /*
            // Ball was launched
            if (servoPrelaunch.getPosition() == 1.0 - SERVO_PRELAUNCH_INIT_POSITION) {
                //setServoInitialPositions();
            }
            */
        }
    }

    private void shootBall() {
        servoLeftPush.setPosition(1.0 - SERVO_LEFT_PUSH_INIT_POSITION);
        servoRightPush.setPosition(1.0 - SERVO_LEFT_PUSH_INIT_POSITION);
        servoPrelaunch.setPower(1);
        //servoPrelaunch.setPosition(1.0 - SERVO_PRELAUNCH_INIT_POSITION);
        motorLaunch.setPower(1);
    }

    /** Return milliseconds since a previous timestamp. */
    private long timeSince(long timestamp) {
        return System.currentTimeMillis() - timestamp;
    }

    /** Set servos to default (safe) positions. */
    private void setServoInitialPositions() {
        servoLeftPush.setPosition(SERVO_LEFT_PUSH_INIT_POSITION);
        servoRightPush.setPosition(SERVO_RIGHT_PUSH_INIT_POSITION);
        servoPrelaunch.setPower(1);
        //servoPrelaunch.setPosition(SERVO_PRELAUNCH_INIT_POSITION);
    }
}
