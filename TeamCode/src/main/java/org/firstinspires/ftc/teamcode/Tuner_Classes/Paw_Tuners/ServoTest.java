package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    private Servo wristFlexServo;
    private Servo elbowServo;

    private AnalogInput elbowEncoder;

    private AnalogInput wristFlexEncoder;

    public static double wristPOS = 1;

    public static double elbowPOS = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        elbowServo = hardwareMap.get(Servo.class, "ES");
        wristFlexServo = hardwareMap.get(Servo.class, "WFS");
        elbowEncoder = hardwareMap.get(AnalogInput.class, "EE");
        wristFlexEncoder = hardwareMap.get(AnalogInput.class, "WFE");



        waitForStart();
        while (opModeIsActive()) {
                wristFlexServo.setPosition(wristPOS);
                elbowServo.setPosition(elbowPOS);
            telemetry.addData("Angle", getWristRawPos());
            telemetry.update();
        }
    }

    public double getWristRawPos() {
        return (wristFlexEncoder.getVoltage() / 3.3);
    }

    public double getElbowRawPos() {
        return (elbowEncoder.getVoltage() / 3.3);
    }

}
