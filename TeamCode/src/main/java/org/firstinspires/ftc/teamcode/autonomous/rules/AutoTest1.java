package org.firstinspires.ftc.teamcode.autonomous.rules;

/* Field: 24" x 24" (60.96 cm x 60.96 cm) */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.BaseAuto;
import org.firstinspires.ftc.teamcode.hardware.OldHardwareBot;

@Autonomous()
public class AutoTest1 extends BaseAuto {
    long lastPress = System.currentTimeMillis();
    static final int pressDelay = 200;

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            handleRotate();
        }
    }

    private void handleRotate() {
        if (gamepad1.a && timeSince(lastPress) > pressDelay) {
            bot.startRotateBot(45, 1);
            waitUntilFinished();
            lastPress = System.currentTimeMillis();
        }
        else if (gamepad1.b && timeSince(lastPress) > pressDelay) {
            bot.startRotateBot(135, 1);
            waitUntilFinished();
            lastPress = System.currentTimeMillis();
        }
        else {
            bot.setDrivePower(0, 0);
        }
    }

    private long timeSince(long timestamp) {
        return System.currentTimeMillis() - timestamp;
    }
}
