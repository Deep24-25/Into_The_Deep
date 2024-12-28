package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ArmFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ShoulderFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.DeviatorFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;

@Config
@TeleOp(name = "Main Tele-op V1.2.0")
public class MainTeleop extends LinearOpMode {
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private HWMap hwMap;
    private Logger logger;
    private LimbFSM limbFSM;
    private ShoulderFSM shoulderFSM;
    private ArmFSM armFSM;
    private MonkeyPawFSM monkeyPawFSM;

    private ElbowFSM elbowFSM;
    private WristFSM wristFSM;
    private DeviatorFSM deviatorFSM;
    private FieldCentricDrive fieldCentricDrive;
    private boolean leftTriggerWasJustPressed;
    private boolean rightTriggerWasJustPressed;
    private double prevLeftTrigger;
    private double prevRightTrigger;

    private boolean yWasJustPressed;
    private boolean aWasJustPressed;
    private boolean xWasJustPressed;
    private boolean bWasJustPressed;
    private boolean leftBumperWasJustPressed;
    private boolean rightBumperWasJustPressed;


    private boolean prevYPressed;
    private boolean prevAPressed;
    private boolean prevXPressed;
    private boolean prevBPressed;
    private boolean prevLeftBumperPressed;
    private boolean prevRightBumperPressed;
    public static double MULTIPLIER = 0.4;
    private int counter = 0;
    private double rightX;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);
            hwMap = new HWMap(hardwareMap);
            logger = new Logger(telemetry);
            shoulderFSM = new ShoulderFSM(hwMap, logger, limbFSM);
            elbowFSM = new ElbowFSM(hwMap, logger, shoulderFSM);
            armFSM = new ArmFSM(hwMap, logger, shoulderFSM, elbowFSM);
            deviatorFSM = new DeviatorFSM(hwMap, logger);
            wristFSM = new WristFSM(hwMap, logger, elbowFSM);
            limbFSM = new LimbFSM(shoulderFSM, armFSM, monkeyPawFSM, logger);
            monkeyPawFSM = new MonkeyPawFSM(hwMap, logger, limbFSM, elbowFSM, deviatorFSM, wristFSM, armFSM);
            fieldCentricDrive = new FieldCentricDrive(hwMap);

            limbFSM.setMonkeyPawFSM(monkeyPawFSM);
            shoulderFSM.setLimbFSM(limbFSM);
            elbowFSM.setArmFSM(armFSM);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        while (opModeInInit()) {
            HWMap.initializeIMU();
        }
        waitForStart();
        while (opModeIsActive()) {
            gamePad1.readButtons();
            gamePad2.readButtons();
            triggersWasJustPressed();

            if (limbFSM.MOVING_TO_INTAKE_POS()) {
                if (-gamePad1.getRightY() > 0.3 || -gamePad1.getRightY() < -0.3) {
                    rightX = 0;
                } else {
                    rightX = gamePad1.getRightX();
                }
            } else {
                rightX = gamePad1.getRightX();
            }
            logger.updateLoggingLevel(gamePad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT));
            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), rightX * MULTIPLIER, HWMap.readFromIMU());
            monkeyPawFSM.updateState(rightTriggerWasJustPressed, leftTriggerWasJustPressed, gamePad1.wasJustPressed(GamepadKeys.Button.Y), gamePad1.wasJustPressed(GamepadKeys.Button.X));
            limbFSM.updateState(gamePad1.isDown(GamepadKeys.Button.DPAD_DOWN), gamePad1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN), gamePad1.isDown(GamepadKeys.Button.DPAD_UP), gamePad1.wasJustReleased(GamepadKeys.Button.DPAD_UP), yWasJustPressed, aWasJustPressed, xWasJustPressed, rightTriggerWasJustPressed, leftBumperWasJustPressed, leftTriggerWasJustPressed, -gamePad1.getRightY(), false);


            updatePID();
            monkeyPawFSM.updatePID();
            log();
            logger.print();

        }
    }

    private void log() {
        logger.log("Right X: ", rightX, Logger.LogLevels.PRODUCTION);
        monkeyPawFSM.log();
        limbFSM.log();
    }

    private void triggersWasJustPressed() {
        yWasJustPressed = gamePad1.isDown(GamepadKeys.Button.Y) & !prevYPressed;
        aWasJustPressed = gamePad1.isDown(GamepadKeys.Button.A) & !prevAPressed;
        xWasJustPressed = gamePad1.isDown(GamepadKeys.Button.X) & !prevXPressed;
        bWasJustPressed = gamePad1.isDown(GamepadKeys.Button.B) & !prevBPressed;
        rightBumperWasJustPressed = gamePad1.isDown(GamepadKeys.Button.RIGHT_BUMPER) & !prevRightBumperPressed;
        leftBumperWasJustPressed = gamePad1.isDown(GamepadKeys.Button.LEFT_BUMPER) & !prevLeftBumperPressed;
        leftTriggerWasJustPressed = gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prevLeftTrigger != 1.0;
        rightTriggerWasJustPressed = gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prevRightTrigger != 1.0;


        prevYPressed = gamePad1.isDown(GamepadKeys.Button.Y);
        prevAPressed = gamePad1.isDown(GamepadKeys.Button.A);
        prevXPressed = gamePad1.isDown(GamepadKeys.Button.X);
        prevBPressed = gamePad1.isDown(GamepadKeys.Button.B);
        prevRightBumperPressed = gamePad1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        prevLeftBumperPressed = gamePad1.isDown(GamepadKeys.Button.LEFT_BUMPER);

        prevLeftTrigger = gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        prevRightTrigger = gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public void updatePID() {
        // monkeyPawFSM.updatePID();
        limbFSM.updatePID();

    }
}
