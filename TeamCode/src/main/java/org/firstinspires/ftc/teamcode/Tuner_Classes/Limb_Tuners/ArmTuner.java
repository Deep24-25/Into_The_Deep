package org.firstinspires.ftc.teamcode.Tuner_Classes.Limb_Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ShoulderFSM;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;

@Config
@TeleOp()
public class ArmTuner extends LinearOpMode {
    private ArmMotorsWrapper armMotorsWrapper;
    private ShoulderFSM shoulderFSM;
    public static double P = 0, I = 0, D = 0, A = 0;
    private PIDFController pidfController;

    public static double targetPos = 0;

    public static double pivotShoulder = 0;

    @Override
    public void runOpMode() {
        try {
            HWMap hwMap = new HWMap(hardwareMap, false);
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            armMotorsWrapper = new ArmMotorsWrapper(hwMap, true);
            shoulderFSM = new ShoulderFSM(hwMap, new Logger(telemetry));

            pidfController = new PIDFController(P, I, D, A);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            pidfController.setPIDF(P, I, D, A);
            if (pivotShoulder == 0) {
                shoulderFSM.moveToIntakeAngle();
            } else if (pivotShoulder == 1) {
                shoulderFSM.setBasketTargetAngle();
            }
            double currentAngle = armMotorsWrapper.readPositionInCM();
            double error = targetPos - currentAngle;


            double power = pidfController.calculate(0, error);
            armMotorsWrapper.set(power);

            shoulderFSM.updatePID();

            telemetry.addData("target pos: ", targetPos);
            telemetry.addData("current pos: ", armMotorsWrapper.getLastReadPositionInCM());
            telemetry.update();

        }
    }
}
