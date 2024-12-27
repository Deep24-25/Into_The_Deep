package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class DeviatorServoTest extends LinearOpMode {
    Servo deviatorServo;
    AnalogInput elbowEncoder;
    public static double targetAngle = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        deviatorServo = hardwareMap.get(Servo.class, "WDS");
        waitForStart();
        while (opModeIsActive()) {
            deviatorServo.setPosition(targetAngle);
            telemetry.addData("Current angle", deviatorServo.getPosition() * 355);
            telemetry.update();
        }
    }
}
