package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private HWMap hwMap;
    private Logger logger;
    private LimbFSM limbFSM;
    private MonkeyPawFSM monkeyPawFSM;
    private boolean leftTriggerWasJustPressed;
    private boolean rightTriggerWasJustPressed;
    private double prevLeftTrigger;
    private double prevRightTrigger;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);
            hwMap = new HWMap(hardwareMap);
            logger = new Logger(telemetry);
            limbFSM = new LimbFSM(hwMap, logger);
            monkeyPawFSM = new MonkeyPawFSM(hwMap, logger, limbFSM);
        } catch (Exception e) {
            logger.log("-", e.getMessage(), Logger.LogLevels.PRODUCTION);
            logger.print();
            while (opModeInInit()) {
                try {
                    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                    gamePad1 = new GamepadEx(gamepad1);
                    gamePad2 = new GamepadEx(gamepad2);
                    hwMap = new HWMap(hardwareMap);
                    logger = new Logger(telemetry);
                    monkeyPawFSM = new MonkeyPawFSM(hwMap, logger, limbFSM);
                } catch (Exception exception) {
                    logger.log("-", exception.getMessage(), Logger.LogLevels.PRODUCTION);
                    logger.print();
                }
            }
            waitForStart();
            while (opModeIsActive()) {
                gamePad1.readButtons();
                gamePad2.readButtons();


                limbFSM.updateState(gamePad2.wasJustPressed(GamepadKeys.Button.Y), gamePad2.wasJustPressed(GamepadKeys.Button.A), gamePad2.wasJustPressed(GamepadKeys.Button.X), gamePad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), rightTriggerWasJustPressed, gamePad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), leftTriggerWasJustPressed, false);
                // monkeyPawFSM.updateState(gamePad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),gamePad2.wasJustPressed(GamepadKeys.Button.X),gamePad2.wasJustPressed(GamepadKeys.Button.B));


                monkeyPawFSM.updatePID();
                log();
                logger.print();

            }
        }
    }

    private void log() {
        monkeyPawFSM.log();
    }

    private void triggersWasJustPressed() {
        leftTriggerWasJustPressed = gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prevLeftTrigger != 1.0;
        rightTriggerWasJustPressed = gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prevRightTrigger != 1.0;


        prevLeftTrigger = gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        prevRightTrigger = gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }
}
