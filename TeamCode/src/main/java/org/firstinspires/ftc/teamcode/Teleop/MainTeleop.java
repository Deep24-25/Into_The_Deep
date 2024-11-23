package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Core.HWMap;

@TeleOp(name="Main Teleop")
public class MainTeleop extends LinearOpMode {
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private FieldCentricDrive fieldCentricDrive;

    private HWMap hwMap;


    public void runOpMode() {
        try {
            hwMap = new HWMap(hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);

            fieldCentricDrive = new FieldCentricDrive(hwMap);
        }
        catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            gamePad1.readButtons();
            gamePad2.readButtons();
            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX(), HWMap.readFromIMU());

            telemetry.addData("Back Right Power", hwMap.getBackRightMotor().get());
            telemetry.addData("Back Left Power", hwMap.getBackleftMotor().get());
            telemetry.addData("Front Right Power", hwMap.getFrontRightMotor().get());
            telemetry.addData("Front Left Power", hwMap.getFrontLeftMotor().get());
            telemetry.addData("IMU Angle",HWMap.readFromIMU());
            telemetry.update();
        }
    }
}
