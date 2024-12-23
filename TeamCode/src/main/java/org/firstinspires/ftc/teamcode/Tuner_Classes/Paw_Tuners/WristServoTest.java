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
public class WristServoTest extends LinearOpMode {
    Servo wristServo;
    public static double targetAngle = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        wristServo = hardwareMap.get(Servo.class, "WFS");
        waitForStart();
        while (opModeIsActive()) {
            wristServo.setPosition(targetAngle);
            telemetry.addData("Current angle", wristServo.getPosition() * 360);
            telemetry.update();
        }
    }
}
