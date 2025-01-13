package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;

@TeleOp
public class GlobalAngleDemo extends LinearOpMode {
    HWMap hwMap;
    private AxonServoWrapper elbowServoWrapper;
    private AxonServoWrapper wristServoWrapper;
    public static double ELBOW_ENCODER_OFFSET = 0;

    public static double wristCompensation = 2;

    public static double wristGlobalTargetAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            hwMap = new HWMap(hardwareMap);
            elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(), hwMap.getElbowEncoder(), false, false, ELBOW_ENCODER_OFFSET); // check if you need to reverse axons
            wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(), hwMap.getWristFlexEncoder(), true, true, WristFSM.ENCODER_OFFSET); // check if you need to reverse axons
            wristGlobalTargetAngle = 180;
        } catch (Exception e) {
            telemetry.addData("message", e);
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            elbowServoWrapper.readPos();
            updatePID();

        }
    }

    public void updatePID() { // This method is used to update position every loop.
        wristServoWrapper.readPos();
        double encoderTargetAngle;
        encoderTargetAngle = convertGlobalAngleToEncoder(wristGlobalTargetAngle, elbowServoWrapper.getLastReadPos());

        if(encoderTargetAngle >=320) {
        encoderTargetAngle = 320;}
        else if(encoderTargetAngle <=140) {
        encoderTargetAngle = 140;
    }
        wristServoWrapper.set(encoderTargetAngle);
}

    private double convertGlobalAngleToEncoder(double globalWristAngle, double elbowCurrentAngle) {
        return ((globalWristAngle - elbowCurrentAngle) + 180) + wristCompensation;
    }
}
