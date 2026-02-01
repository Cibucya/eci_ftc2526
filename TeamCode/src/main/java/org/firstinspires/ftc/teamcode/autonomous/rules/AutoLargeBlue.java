package org.firstinspires.ftc.teamcode.autonomous.rules;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.autonomous.BaseAuto;

/*
 * Field: 24" x 24" (60.96 cm x 60.96 cm)
 * Square: 4" x 4" (10.16 cm x 10.16 cm)
 */

@Autonomous
public class AutoLargeBlue extends BaseAuto {
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
            // Move backward 1 block
            bot.startTravel(-Field.FIELD_BLOCK_SIDE_LENGTH_MM/2, 1);
            waitUntilFinished(100);

            // Rotate -45 degrees
            bot.startRotateBot(-45, 0.5);
            waitUntilFinished(100);

            // Shoot balls (3 seconds)
            bot.setShootPower(1);
            bot.setPushPower(1, 1, 1);
            sleep(7500);
            bot.setShootPower(0);
            bot.setPushPower(0, 0, 0);

            // Rotate -135 degrees to face arena
            bot.startRotateBot(-90, 0.5);
            waitUntilFinished(100);

            // Move out of the starting zone
            bot.startTravel(Field.FIELD_BLOCK_SIDE_LENGTH_MM/2, 1);
            waitUntilFinished(100);
        }
    }
}