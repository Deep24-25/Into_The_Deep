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
public class MainTeleop extends LinearOpMode {
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private HWMap hwMap;
    private Logger logger;
    private LimbFSM limbFSM;
    private MonkeyPawFSM monkeyPawFSM;
    private FieldCentricDrive fieldCentricDrive;
    private static final double MULTIPLIER = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);
            hwMap = new HWMap(hardwareMap);
            logger = new Logger(telemetry);
            limbFSM = new LimbFSM();
            monkeyPawFSM = new MonkeyPawFSM(hwMap,logger,limbFSM);
            fieldCentricDrive = new FieldCentricDrive(hwMap);
        }catch (Exception e){
            logger.log("-", e.getMessage(), Logger.LogLevels.PRODUCTION);
            logger.print();
        }
        waitForStart();
        while (opModeIsActive()){
            gamePad1.readButtons();
            gamePad2.readButtons();

            if (gamePad1.isDown(GamepadKeys.Button.X))
                HWMap.initializeIMU();

            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX() * MULTIPLIER, HWMap.readFromIMU());
            monkeyPawFSM.updateState(gamePad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),gamePad2.wasJustPressed(GamepadKeys.Button.X),gamePad2.wasJustPressed(GamepadKeys.Button.B), gamePad2.wasJustPressed(GamepadKeys.Button.A), gamePad2.wasJustPressed(GamepadKeys.Button.DPAD_UP),gamePad2.wasJustPressed(GamepadKeys.Button.Y), gamePad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),gamePad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),gamePad1.wasJustPressed(GamepadKeys.Button.X), gamePad1.wasJustPressed(GamepadKeys.Button.Y), gamePad1.wasJustPressed(GamepadKeys.Button.A));


            monkeyPawFSM.updatePID();
            log();
            logger.print();

        }
    }

    private void log() {
        monkeyPawFSM.log();
    }
}
