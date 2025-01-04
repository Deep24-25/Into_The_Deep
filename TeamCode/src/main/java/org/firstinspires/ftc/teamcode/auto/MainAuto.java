package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.HWMap;

import java.util.concurrent.TimeUnit;

@Autonomous
public class MainAuto extends LinearOpMode {
    private HWMap hwMap;
    private Timing.Timer timer;
    private boolean autoDone = false;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
        hwMap = new HWMap(hardwareMap);
        timer = new Timing.Timer(2500, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        while (opModeInInit()) {
            HWMap.initializeIMU();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (!timer.isTimerOn() && !autoDone) {
                timer.start();
                hwMap.getBackleftMotor().set(-0.25);
                hwMap.getBackRightMotor().set(0.25);
                hwMap.getFrontRightMotor().set(-0.25);
                hwMap.getFrontLeftMotor().set(0.25);
            }
            if (timer.done()) {
                hwMap.getBackleftMotor().set(0);
                hwMap.getBackRightMotor().set(0);
                hwMap.getFrontRightMotor().set(0);
                hwMap.getFrontLeftMotor().set(0);
                timer.pause();
                autoDone = true;
            }
            telemetry.addData("timer", timer.elapsedTime());
            telemetry.update();
        }
    }
}
