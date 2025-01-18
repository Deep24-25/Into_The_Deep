package org.firstinspires.ftc.teamcode.Tuner_Classes.Limb_Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;

@TeleOp
@Config
public class LinearizationTest extends LinearOpMode {
    private ArmMotorsWrapper armMotorsWrapper;
    public static double PLinearizing = 0.12, ILinearizing = 0.1, DLinearizing = 0.004, FLinearizing = 0;
    private PIDFController pidfController;


    public static double pivotShoulder = 0;

    AxonServoWrapper elbowServoWrapper;

    private double SAMPLE_PICKUP_LINEARIZATION_OFFSET = 0;
    public static double targetPosition = 20;
    public static double targetAngle = 135;
    public static double actualTargetPos = 0;
    private double prevTargetAngle = 1;
    private int yes = 0;

    private double lastLastReadPos = 0;
    @Override
    public void runOpMode() {
        try {
            HWMap hwMap = new HWMap(hardwareMap, false);
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            armMotorsWrapper = new ArmMotorsWrapper(hwMap);
            elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(), hwMap.getElbowEncoder(), false, false, ElbowFSM.ENCODER_OFFSET);
            pidfController = new PIDFController(PLinearizing, ILinearizing, DLinearizing, FLinearizing);
            armMotorsWrapper.resetEncoder();

        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            pidfController.setPIDF(PLinearizing, ILinearizing, DLinearizing, FLinearizing);
            elbowServoWrapper.readPos();

            offset();


            targetPosition = actualTargetPos - SAMPLE_PICKUP_LINEARIZATION_OFFSET;
            double currentPos = armMotorsWrapper.readPositionInCM();
            double error = targetPosition - currentPos;


            double power = pidfController.calculate(0, error);
            armMotorsWrapper.set(power);
            elbowServoWrapper.set(targetAngle);


            telemetry.addData("target pos(arm): ", targetPosition);
            telemetry.addData("current pos(arm): ", armMotorsWrapper.getLastReadPositionInCM());

            telemetry.addData("target pos(elbow): ", targetAngle);
            telemetry.addData("current pos(elbow): ", elbowServoWrapper.getLastReadPos());
            telemetry.addData("Last current pos(elbow): ", lastLastReadPos);

            telemetry.addData("Sample offset: ", SAMPLE_PICKUP_LINEARIZATION_OFFSET);


            telemetry.update();
            prevTargetAngle = targetAngle;
            lastLastReadPos = elbowServoWrapper.getLastReadPos();


        }
    }

    public void offset() {
        SAMPLE_PICKUP_LINEARIZATION_OFFSET = ((15.5 * Math.cos(Math.toRadians(180 - elbowServoWrapper.getLastReadPos())))) - ((15.5 * Math.cos(Math.toRadians(Math.toRadians(180 - lastLastReadPos)))));
    }
}
