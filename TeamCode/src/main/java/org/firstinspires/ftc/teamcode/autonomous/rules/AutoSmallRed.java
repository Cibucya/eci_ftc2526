package org.firstinspires.ftc.teamcode.autonomous.rules;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.autonomous.BaseAuto;

@Autonomous
public class AutoSmallRed extends BaseAuto {
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
            // Forward 3 blocks to the white line of big area
            bot.startTravel((int) Math.round(3 * Field.FIELD_BLOCK_SIDE_LENGTH_MM), 1);
            waitUntilFinished(100);

            // Rotate 45 degrees
            bot.startRotateBot(45, 0.5);
            waitUntilFinished(100);

            // Travel 1 block closer to the goal
            bot.startTravel((int) Math.round(Field.FIELD_BLOCK_DIAGONAL_LENGTH_MM), 1);
            waitUntilFinished(100);

            // Shoot balls (3 seconds)
            bot.setShootPower(1);
            bot.setPushPower(1, 1, 1);
            sleep(3000);
            bot.setShootPower(0);
            bot.setPushPower(0, 0, 0);

            // Face arena
            bot.startRotateBot(90, 0.5);
            waitUntilFinished(100);

            // Move out of the starting zone
            bot.startTravel((int) Math.round(Field.FIELD_BLOCK_SIDE_LENGTH_MM), 1);
            waitUntilFinished(100);
        }
    }
}
