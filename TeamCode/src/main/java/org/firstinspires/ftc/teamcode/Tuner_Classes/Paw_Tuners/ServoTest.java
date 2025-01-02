package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HWMap;
@TeleOp
public class ServoTest extends LinearOpMode {
    private HWMap hwMap;
    private Servo deviatorServo;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        deviatorServo = hwMap.getFingerServo();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                deviatorServo.setPosition(300);
            }
            if(gamepad1.y) {
                deviatorServo.setPosition(90);
                deviatorServo.setPosition(-140);
            }
            if(gamepad1.x) {
                deviatorServo.setPosition(90);
            }
            telemetry.addData("Angle", deviatorServo.getPosition());
            telemetry.update();
        }
    }
}
