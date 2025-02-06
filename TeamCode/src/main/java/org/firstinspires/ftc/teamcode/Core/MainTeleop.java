package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ArmFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ShoulderFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.DeviatorFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Main Tele-op V1.4.1")
public class MainTeleop extends LinearOpMode {
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private Logger logger;
    private LimbFSM limbFSM;
    private MonkeyPawFSM monkeyPawFSM;

    private HWMap hwMap;
    private FieldCentricDrive fieldCentricDrive;
    private boolean leftTriggerWasJustPressed;
    private boolean rightTriggerWasJustPressed;
    private double prevLeftTrigger;
    private double prevRightTrigger;

    private boolean yWasJustPressed;
    private boolean aWasJustPressed;
    private boolean xWasJustPressed;
    private boolean leftBumperWasJustPressed;


    private boolean prevYPressed;
    private boolean prevAPressed;
    private boolean prevXPressed;
    private boolean prevLeftBumperPressed;
    public static double MULTIPLIER = 0.7;
    public static double strafeMultiplierWhileMovingIntake = 1;
    public static double forwardMultiplierWhileMovingIntake = 1;
    public static double turningMultiplierWhileMovingIntake = 1;

    private Timing.Timer loopTimer;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);
            hwMap = new HWMap(hardwareMap, false);
            logger = new Logger(telemetry);
            ShoulderFSM shoulderFSM = new ShoulderFSM(hwMap, logger, limbFSM, false);
            ElbowFSM elbowFSM = new ElbowFSM(hwMap, logger, shoulderFSM);
            ArmFSM armFSM = new ArmFSM(hwMap, logger, shoulderFSM, elbowFSM, false);
            DeviatorFSM deviatorFSM = new DeviatorFSM(hwMap, logger);
            WristFSM wristFSM = new WristFSM(hwMap, logger, elbowFSM);
            limbFSM = new LimbFSM(hwMap, shoulderFSM, armFSM, monkeyPawFSM, logger);
            monkeyPawFSM = new MonkeyPawFSM(hwMap, logger, limbFSM, elbowFSM, deviatorFSM, wristFSM, armFSM);
            fieldCentricDrive = new FieldCentricDrive(hwMap);

            limbFSM.setMonkeyPawFSM(monkeyPawFSM);
            shoulderFSM.setLimbFSM(limbFSM);
            elbowFSM.setArmFSM(armFSM);

            loopTimer = new Timing.Timer(999999999, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        loopTimer.start();
        while (opModeInInit()) {
            hwMap.clearCache();
            if (!HWMap.initialized) {
                HWMap.initializeIMU();
                monkeyPawFSM.updateState(false, false, false, false, false, false, false, false, false, false, false);
                monkeyPawFSM.updatePID();
            }
            logger.log("loop timer", loopTimer.elapsedTime(), Logger.LogLevels.PRODUCTION);
            logger.print();
        }
        waitForStart();
        while (opModeIsActive()) {
            hwMap.clearCache();
            loopTimer.start();
            gamePad1.readButtons();
            gamePad2.readButtons();
            triggersWasJustPressed();

            double rightX;
            if (limbFSM.MOVING_TO_INTAKE_POS()) {
                if (-gamePad1.getRightY() > 0.15 || -gamePad1.getRightY() < -0.15) {
                    rightX = 0;
                } else {
                    rightX = gamePad1.getRightX();
                }
                strafeMultiplierWhileMovingIntake = 0.7;
                forwardMultiplierWhileMovingIntake = 0.7;
                turningMultiplierWhileMovingIntake = 0.7;
            } else {
                rightX = gamePad1.getRightX();
                strafeMultiplierWhileMovingIntake = 1;
                forwardMultiplierWhileMovingIntake = 1;
                turningMultiplierWhileMovingIntake = 1;
            }
            if (gamePad2.wasJustPressed(GamepadKeys.Button.Y)) {
                HWMap.initializeIMU();
            }

            logger.updateLoggingLevel(gamePad1.wasJustPressed(GamepadKeys.Button.BACK));
            fieldCentricDrive.drive(gamePad1.getLeftX() * strafeMultiplierWhileMovingIntake, gamePad1.getLeftY() * forwardMultiplierWhileMovingIntake, (rightX * MULTIPLIER) * turningMultiplierWhileMovingIntake, HWMap.readFromIMU());
            fieldCentricDrive.drive(gamePad1.getLeftX() * strafeMultiplierWhileMovingIntake, gamePad1.getLeftY() * forwardMultiplierWhileMovingIntake, (rightX * MULTIPLIER) * turningMultiplierWhileMovingIntake, HWMap.readFromIMU());
            monkeyPawFSM.updateState(rightTriggerWasJustPressed, leftTriggerWasJustPressed, gamePad1.wasJustPressed(GamepadKeys.Button.Y), gamePad1.wasJustPressed(GamepadKeys.Button.X), gamePad1.wasJustPressed(GamepadKeys.Button.DPAD_UP), gamePad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), gamePad1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT), gamePad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT), gamePad2.wasJustPressed(GamepadKeys.Button.DPAD_UP), gamePad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), false);
            limbFSM.updateState(gamePad2.isDown(GamepadKeys.Button.DPAD_RIGHT), gamePad2.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT), gamePad2.isDown(GamepadKeys.Button.DPAD_LEFT), gamePad2.wasJustReleased(GamepadKeys.Button.DPAD_LEFT), yWasJustPressed, aWasJustPressed, xWasJustPressed, rightTriggerWasJustPressed, leftBumperWasJustPressed, leftTriggerWasJustPressed, -gamePad1.getRightY(), false, false);


            updatePID();
            monkeyPawFSM.updatePID();
            log();
            logger.print();

        }
    }

    private void log() {
        logger.log("Loop time: ", loopTimer.elapsedTime(), Logger.LogLevels.PRODUCTION);
        logger.log("Voltage: ", hwMap.getVoltageSensor().getVoltage(), Logger.LogLevels.PRODUCTION);
        logger.log("IMU angle", HWMap.getIMUangle(), Logger.LogLevels.DEBUG);
        monkeyPawFSM.log();
        limbFSM.log();
    }

    private void triggersWasJustPressed() {
        yWasJustPressed = gamePad1.isDown(GamepadKeys.Button.Y) & !prevYPressed;
        aWasJustPressed = gamePad1.isDown(GamepadKeys.Button.A) & !prevAPressed;
        xWasJustPressed = gamePad1.isDown(GamepadKeys.Button.X) & !prevXPressed;
        leftBumperWasJustPressed = gamePad1.isDown(GamepadKeys.Button.LEFT_BUMPER) & !prevLeftBumperPressed;
        leftTriggerWasJustPressed = gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prevLeftTrigger != 1.0;
        rightTriggerWasJustPressed = gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prevRightTrigger != 1.0;


        prevYPressed = gamePad1.isDown(GamepadKeys.Button.Y);
        prevAPressed = gamePad1.isDown(GamepadKeys.Button.A);
        prevXPressed = gamePad1.isDown(GamepadKeys.Button.X);
        prevLeftBumperPressed = gamePad1.isDown(GamepadKeys.Button.LEFT_BUMPER);

        prevLeftTrigger = gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        prevRightTrigger = gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public void updatePID() {
        limbFSM.updatePID(false);
    }
}
