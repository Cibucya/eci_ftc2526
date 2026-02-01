package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareBot;

@TeleOp()
public class Teleoperation extends LinearOpMode {
    HardwareBot bot = new HardwareBot();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleShoot();
            handlePush();
        }
    }

    private void handleDrive() {
        bot.driveBotExponential(-gamepad1.right_stick_y, gamepad1.right_stick_x, false);
    }

    private void handleShoot() {
        if (gamepad1.a) {
            bot.setPushPower(1,1,1);
            bot.setShootPower(1);
        }
        else {
            bot.setPushPower(0,0,0);
            bot.setShootPower(0);
        }
    }

    private void handlePush() {
        if (gamepad1.b) {
            bot.setPushPower(1, 1, 1);
        }
        else {
            bot.setPushPower(0, 0, 0);
        }
    }
}
