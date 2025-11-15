package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Autonomous OpMode", group="Linear OpMode")
public class AutonomousOpMode extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // --- Constants for Encoder-Based Driving ---
    // This is the number of encoder ticks for one full revolution of the motor shaft.
    // This value is specific to your motors (e.g., REV HD Hex).
    static final double TICKS_PER_REV = 537.7; 

    // The diameter of your robot's wheels in centimeters.
    static final double WHEEL_DIAMETER_CM = 9.6; // For official FTC wheels

    // Calculated value for how many ticks it takes to travel 1 centimeter.
    static final double TICKS_PER_CM = TICKS_PER_REV / (WHEEL_DIAMETER_CM * Math.PI);
    
    // This is a tuning value for turning. You will need to adjust it for your robot.
    // Start with a value around 10.0 and increase/decrease it until a 90-degree turn is accurate.
    static final double DEGREES_TO_TICKS_CONVERSION = 11.5;


    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders at the start of initialization
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders by default
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        String selectedCase = "";

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Select Placement:");
            telemetry.addData("A", "Straight (Large Zone)");
            telemetry.addData("B", "Angled Left");
            telemetry.addData("X", "Angled Right");
            telemetry.addData("Y", "Straight (Small Zone)");
            telemetry.addLine("\n---SELECTED---");

            if (gamepad1.a) selectedCase = "A";
            else if (gamepad1.b) selectedCase = "B";
            else if (gamepad1.x) selectedCase = "C";
            else if (gamepad1.y) selectedCase = "D";

            telemetry.addData("Selection", selectedCase);
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        if (opModeIsActive()) {
            switch (selectedCase) {
                case "A": // Straight in Large Zone - Requires ~96 inches
                    driveForwardEncoder(243.84, 0.5);
                    break;

                case "B": // Angled Left, needs to turn right to straighten out - Requires ~96 inches
                    turnRightEncoder(45, 0.5);   // Turn 45 degrees
                    driveForwardEncoder(243.84, 0.5);
                    break;

                case "C": // Angled Right, needs to turn left to straighten out - Requires ~96 inches
                    turnLeftEncoder(45, 0.5);
                    driveForwardEncoder(243.84, 0.5);
                    break;

                case "D": // Straight in Small Zone - Requires ~48 inches
                    driveForwardEncoder(121.92, 0.5);
                    break;

                default:
                    telemetry.addLine("No case selected.");
                    telemetry.update();
                    break;
            }
        }
    }

    
    private void driveForwardEncoder(double distanceCm, double power) {
        int targetTicks = (int)(distanceCm * TICKS_PER_CM);

        // Set the target position for both motors
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + targetTicks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + targetTicks);

        // Set motors to RUN_TO_POSITION mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motors
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // Wait until both motors have reached their target
        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("Status", "Driving to position...");
            telemetry.addData("Target", "%d ticks", targetTicks);
            telemetry.addData("Left Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position", rightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Reset motor mode for future use
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void turnLeftEncoder(double degrees, double power) {
        int turnTicks = (int)(degrees * DEGREES_TO_TICKS_CONVERSION);

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - turnTicks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + turnTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("Status", "Turning left...");
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void turnRightEncoder(double degrees, double power) {
        int turnTicks = (int)(degrees * DEGREES_TO_TICKS_CONVERSION);

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + turnTicks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - turnTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("Status", "Turning right...");
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
