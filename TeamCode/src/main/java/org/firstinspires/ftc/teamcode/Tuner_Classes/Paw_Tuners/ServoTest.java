package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HWMap;
@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    private HWMap hwMap;
    private Servo deviatorServo;

    public static double POS = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        deviatorServo = hwMap.getFingerServo();
        waitForStart();
        while (opModeIsActive()) {
                deviatorServo.setPosition(POS);
            telemetry.addData("Angle", deviatorServo.getPosition());
            telemetry.update();
        }
    }
}
