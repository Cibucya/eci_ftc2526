package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareBot;

public abstract class BaseAuto extends LinearOpMode {
    protected HardwareBot bot = new HardwareBot();

    protected void waitUntilFinished() {
        while (opModeIsActive() && bot.getState() != HardwareBot.RobotState.IDLE) {
            bot.update();
            telemetry.addData("Status", "Executing " + bot.getState());
            telemetry.update();
            idle(); // Give the hardware a tiny bit of breathing room
        }
    }

    protected void waitUntilFinished(int sleepDuration) {
        waitUntilFinished();
        sleep(sleepDuration);
    }
}
