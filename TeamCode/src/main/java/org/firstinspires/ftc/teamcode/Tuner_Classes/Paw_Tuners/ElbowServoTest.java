package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;

@TeleOp
@Config
public class ElbowServoTest extends LinearOpMode {
    AxonServoWrapper elbowServoWrapper;
    HWMap hwMap;
    public static double targetAngle = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(), hwMap.getElbowEncoder(), false, false, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            elbowServoWrapper.set(targetAngle);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("Current angle", elbowServoWrapper.readPos());
            telemetry.update();
        }
    }
}
