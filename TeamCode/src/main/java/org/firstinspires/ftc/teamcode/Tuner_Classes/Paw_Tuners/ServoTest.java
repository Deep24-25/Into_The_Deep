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

    public static double fingerPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        HWMap hwMap = new HWMap(hardwareMap, false);
        Servo fingerServo = hwMap.getFingerServo();
        waitForStart();
        while (opModeIsActive()) {
            fingerServo.setPosition(fingerPos);
            telemetry.addData("Position", fingerPos);
            telemetry.update();
        }
    }


}
