package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;

@TeleOp
@Config
public class WristServoTest extends LinearOpMode {
    AxonServoWrapper wristServoWrapper;
    HWMap hwMap;
    public static double targetAngle = 180;
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(), hwMap.getWristFlexEncoder(), false,false, WristFSM.ENCODER_OFFSET);
        waitForStart();
        while (opModeIsActive()) {
            wristServoWrapper.set(targetAngle);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("Current angle", wristServoWrapper.readPos());
            telemetry.update();
        }
    }
}
