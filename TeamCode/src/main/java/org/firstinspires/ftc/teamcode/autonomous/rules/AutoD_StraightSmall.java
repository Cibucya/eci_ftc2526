package org.firstinspires.ftc.teamcode.autonomous.rules;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="D: Straight Small", group="Launch Zone Exits")
public class AutoD_StraightSmall extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    static final double TICKS_PER_REV = 537.7; 
    static final double WHEEL_DIAMETER_CM = 9.6;
    static final double TICKS_PER_CM = TICKS_PER_REV / (WHEEL_DIAMETER_CM * Math.PI);
    static final double DEGREES_TO_TICKS_CONVERSION = 11.5;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "motor_left");
        rightMotor = hardwareMap.get(DcMotor.class, "motor_right");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready for Case D: Straight Small Zone");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveForwardEncoder(121.92, 0.5);
        }
    }

    private void driveForwardEncoder(double distanceCm, double power) {
        int targetTicks = (int)(distanceCm * TICKS_PER_CM);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + targetTicks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + targetTicks);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("Status", "Driving to position...");
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
