package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;

@TeleOp
@Config
public class WristServoTest extends LinearOpMode {
    AxonServoWrapper wristServoWrapper;
    HWMap hwMap;
    public static double targetAngle = 180;
    private static final double RATIO = (30.0/20)*(12.0/15);
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap, false);
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(), hwMap.getWristFlexEncoder(), false,false, 0, RATIO);
        waitForStart();
        while (opModeIsActive()) {
            hwMap.clearCache();
            wristServoWrapper.set(targetAngle);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("Current angle", wristServoWrapper.readPos());
            telemetry.update();
        }
    }
}
