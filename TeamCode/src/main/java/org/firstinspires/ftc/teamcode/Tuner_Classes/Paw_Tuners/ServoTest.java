package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
@TeleOp
public class ServoTest extends LinearOpMode {
    private HWMap hwMap;
    private ServoEx deviatorServo;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        deviatorServo = hwMap.getFingerServo();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                deviatorServo.turnToAngle(300);
            }
            if(gamepad1.y) {
                deviatorServo.turnToAngle(90);
                deviatorServo.turnToAngle(-140);
            }
            if(gamepad1.x) {
                deviatorServo.turnToAngle(90);
            }
            telemetry.addData("Angle", deviatorServo.getAngle());
            telemetry.update();
        }
    }
}
