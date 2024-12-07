package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
                deviatorServo.turnToAngle(140);
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
