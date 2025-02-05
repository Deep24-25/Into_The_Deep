package org.firstinspires.ftc.teamcode.Tuner_Classes.Limb_Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;

@Config
@TeleOp(name = "Shoulder Tuner")
public class ShoulderTuner extends LinearOpMode {
    private ShoulderWrapper shoulderWrapper;
    public static double P = 0, I = 0, D = 0, A = 0;
    private PIDFController pidfController;

    public static double targetAngle = 0;
    private HWMap hwMap;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(hardwareMap, false);
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            shoulderWrapper = new ShoulderWrapper(hwMap, true);
            pidfController = new PIDFController(P, I, D, A);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            hwMap.clearCache();
            pidfController.setPIDF(P, I, D, A);
            double currentAngle = shoulderWrapper.readAngle();
            double error = targetAngle - currentAngle;


            double power = pidfController.calculate(0, error);
            shoulderWrapper.set(power);

            telemetry.addData("target angle: ", targetAngle);
            telemetry.addData("current angle: ", shoulderWrapper.getLastReadAngle());
            telemetry.update();

        }
    }
}
