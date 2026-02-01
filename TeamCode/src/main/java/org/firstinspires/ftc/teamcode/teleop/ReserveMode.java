package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Mode - Controlling robot drive, launcher, and servos.
 *
 * Controls:
 *  - Right joystick: Drive robot (motor_left, motor_right)
 *  - Gamepad A: Toggle launcher motor ON/OFF
 *  - Gamepad B: Trigger ball shot (1 press = 1 shot, max 3 shoots)
 *
 * Field: 24" x 24" (60.96 cm x 60.96 cm)
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp()
public class ReserveMode extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorShoot;
    private CRServo servoLeftPush;
    private CRServo servoRightPush;
    private CRServo servoPreshoot;

    private int timeSinceA = 0;
    private int ballsToShoot = 0;

    private static final int PRESS_DELAY_MS = 100;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            handleDriveGamepad1();
            handleDriveGamepad2();
            handleShooting();
            handlePushBall();
        }
    }

    /** Handle robot driving using right joystick. */
    private void handleDriveGamepad1() {
        double drive = -gamepad1.right_stick_y; // Forward/backward
        double turn = gamepad1.right_stick_x;   // Rotation

        motorLeft.setPower(drive + turn);
        motorRight.setPower(drive - turn);
    }

    private void handleDriveGamepad2() {
        double drive = -gamepad2.right_stick_y;
        double turn = gamepad2.left_stick_x;
        telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
        telemetry.addData("gamepad2.left_stick_x", gamepad2.left_stick_x);
        telemetry.update();

        motorLeft.setPower(drive+turn);
        motorRight.setPower(drive-turn);
    }

    void initHardware() {
        // Map hardware
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorShoot = hardwareMap.dcMotor.get("motor_shoot");

        servoLeftPush = hardwareMap.crservo.get("servo_left_push");
        servoRightPush = hardwareMap.crservo.get("servo_right_push");
        servoPreshoot = hardwareMap.crservo.get("servo_center_push");

        // Set motor directions
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorShoot.setDirection(DcMotor.Direction.REVERSE);

        // Set servo directions
        servoLeftPush.setDirection(CRServo.Direction.FORWARD);
        servoRightPush.setDirection(CRServo.Direction.REVERSE);
        servoPreshoot.setDirection(CRServo.Direction.FORWARD);
    }

    /* Determines whether to shoot a ball */
    void handleShooting() {
        if (gamepad1.a) {
            shootBall();
        }
        else {
            turnOffServos();
            motorShoot.setPower(0);
        }
    }

    /** Shoot a ball */
    void shootBall() {
        servoLeftPush.setPower(1);
        servoRightPush.setPower(1);
        servoPreshoot.setPower(1);
        motorShoot.setPower(1);
    }

    void handlePushBall() {
        if (gamepad1.b) {
            push_ball();
        }
    }

    void push_ball() {
        servoLeftPush.setPower(1);
        servoRightPush.setPower(1);
        servoPreshoot.setPower(1);
    }

    void turnOffServos() {
        servoLeftPush.setPower(0);
        servoRightPush.setPower(0);
        servoPreshoot.setPower(0);
    }

    private long timeSince(long timestamp) {
        return System.currentTimeMillis() - timestamp;
    }
}