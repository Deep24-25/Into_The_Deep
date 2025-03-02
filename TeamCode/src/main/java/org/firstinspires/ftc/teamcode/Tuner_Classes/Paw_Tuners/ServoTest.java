package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HWMap;
@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    private Servo deviatorServo;
    private AnalogInput encoder;

    public static double POS = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        deviatorServo = hardwareMap.get(Servo.class, "WDS");
        encoder = hardwareMap.get(AnalogInput.class, "WDE");
        waitForStart();
        while (opModeIsActive()) {
            getRawPos();
            deviatorServo.setPosition(POS);
            telemetry.addData("Angle", getRawPos());
            telemetry.update();
        }
    }
    public double getRawPos() {
        return (encoder.getVoltage() / 3.3);
    }
}
