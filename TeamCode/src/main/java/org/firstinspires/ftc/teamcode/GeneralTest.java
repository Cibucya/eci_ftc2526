package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class GeneralTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo leftPush    = hardwareMap.get(CRServo.class, "servo_left_push");
        CRServo rightPush   = hardwareMap.get(CRServo.class, "servo_right_push");
        CRServo centerPush  = hardwareMap.get(CRServo.class, "servo_center_push");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                leftPush.setPower(1);
                rightPush.setPower(1);
                centerPush.setPower(1);
            }
            else {
                leftPush.setPower(0);
                rightPush.setPower(0);
                centerPush.setPower(0);
            }
        }
    }
}
