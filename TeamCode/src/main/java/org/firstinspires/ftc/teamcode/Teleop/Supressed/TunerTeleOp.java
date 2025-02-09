package org.firstinspires.ftc.teamcode.Teleop.Supressed;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Tuner_Classes.Misc_Tuners.ColorSensorCSVTuner;

@TeleOp
@Disabled
public class TunerTeleOp extends LinearOpMode {
    private HWMap hwMap;
    private ColorSensorCSVTuner tuner;
    private GamepadEx gamepadEx;

    private RevColorSensorV3 colorSensorV3Left;

    private RevColorSensorV3 colorSensorV3Right;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(hardwareMap, false);
            gamepadEx = new GamepadEx(gamepad1);
            //tuner = new ColorSensorCSVTuner(telemetry, hwMap.getColorSensor1(), hwMap.getColorSensor2(), gamepadEx);
            tuner = new ColorSensorCSVTuner(telemetry, colorSensorV3Left, colorSensorV3Right, gamepadEx);

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
