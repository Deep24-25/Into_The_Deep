package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Tuner.ColorSensorCSVTuner;

import java.io.IOException;

@TeleOp
public class TunerTeleOp extends LinearOpMode {
    private HWMap hwMap;
    private ColorSensorCSVTuner tuner;
    private GamepadEx gamepadEx;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(hardwareMap);
            gamepadEx = new GamepadEx(gamepad1);
            tuner = new ColorSensorCSVTuner(telemetry, hwMap.getColorSensor1(), hwMap.getColorSensor2(), gamepadEx);
        } catch (Exception e) {
            throw new RuntimeException(e.getMessage());
        }
        waitForStart();
        while (opModeIsActive()) {
            tuner.dataCollection();
            tuner.log();
            if (gamepadEx.isDown(GamepadKeys.Button.A))
                tuner.exportData();
        }
    }

}
