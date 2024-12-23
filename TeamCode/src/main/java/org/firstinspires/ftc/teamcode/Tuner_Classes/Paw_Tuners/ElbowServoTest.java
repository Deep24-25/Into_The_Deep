package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HWMap;
@TeleOp
@Config
public class ElbowServoTest extends LinearOpMode {
    Servo elbowServo;
    public static double targetAngle = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        elbowServo = hardwareMap.get(Servo.class, "ES");
        waitForStart();
        while (opModeIsActive()) {
            elbowServo.setPosition(targetAngle);
            telemetry.addData("Current angle", elbowServo.getPosition() * 360);
            telemetry.update();
        }
    }
}
