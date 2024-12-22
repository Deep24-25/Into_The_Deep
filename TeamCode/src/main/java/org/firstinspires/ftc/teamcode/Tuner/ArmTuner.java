package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;

@Config
@TeleOp()
public class ArmTuner extends LinearOpMode {
    private ArmMotorsWrapper armMotorsWrapper;
    public static double P = 0, I = 0, D = 0, A = 0;
    private PIDFController pidfController;

    public static double targetPos = 0;

    @Override
    public void runOpMode() {
        try {
            HWMap hwMap = new HWMap(hardwareMap);
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            armMotorsWrapper = new ArmMotorsWrapper(hwMap);
            pidfController = new PIDFController(P, I, D, A);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            pidfController.setPIDF(P, I, D, A);
            double currentAngle = armMotorsWrapper.readPositionInCM();
            double error = targetPos - currentAngle;


            double power = pidfController.calculate(0, error);
            armMotorsWrapper.set(power);

            telemetry.addData("target pos: ", targetPos);
            telemetry.addData("current pos: ", armMotorsWrapper.getLastReadPositionInCM());
            telemetry.update();

        }
    }
}
