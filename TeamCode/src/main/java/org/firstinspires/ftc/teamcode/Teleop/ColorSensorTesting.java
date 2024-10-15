package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Tuner.ColorSensorCSVTuner;

import java.io.IOException;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

@TeleOp
public class ColorSensorTesting extends LinearOpMode {
    private HWMap hwMap;
    private RevColorSensorV3 csLeft;
    private RevColorSensorV3 csRight;

    private GamepadEx gamepadEx;
    private Timing.Timer loopTimer;
    private static final int DISTANCE_THRESHOLD = 30;

    private double redLeft, blueLeft, greenLeft, alphaLeft, redRight, blueRight, greenRight, alphaRight;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(hardwareMap);
            gamepadEx = new GamepadEx(gamepad1);
            csLeft = hwMap.getColorSensor1();
            csRight = hwMap.getColorSensor2();
            loopTimer = new Timing.Timer(10000000, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            throw new RuntimeException(e.getMessage());
        }
        waitForStart();
        while (opModeIsActive()) {
            loopTimer.start();

            redLeft = csLeft.red();
            blueLeft = csLeft.blue();
            greenLeft = csLeft.green();
            alphaLeft = csLeft.alpha();

            redRight = csRight.red();
            blueRight = csRight.blue();
            greenRight = csRight.green();
            alphaRight = csRight.alpha();

            if (csLeft.getDistance(DistanceUnit.CM) < DISTANCE_THRESHOLD) {
                if (greenLeft > redLeft && redLeft > blueLeft)
                    telemetry.addData("-", "YELLOW by left");
                else if (redLeft > greenLeft && greenLeft > blueLeft)
                    telemetry.addData("-", "RED by left");
                else if (blueLeft > greenLeft && greenLeft > redLeft)
                    telemetry.addData("-", "BLUE by left");
            }

            if (csRight.getDistance(DistanceUnit.CM) < DISTANCE_THRESHOLD) {
                if (greenRight > redRight && redLeft > blueRight)
                    telemetry.addData("-", "YELLOW by right");
                else if (redRight > greenRight && greenRight > blueRight)
                    telemetry.addData("-", "RED by right");
                else if (blueRight > greenRight && greenRight > redRight)
                    telemetry.addData("-", "BLUE by right");

            }
            telemetry.addData("-", "Left Distance: " + csLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("-", "Right Distance: " + csRight.getDistance(DistanceUnit.MM));
            telemetry.addData("-", "Loop Time: " + loopTimer.elapsedTime());
            telemetry.update();
        }
    }

}

