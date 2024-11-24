package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Core.HWMap;

@TeleOp
public class fingerTest extends LinearOpMode {
    HWMap hwMap;
    CRServo FE;
    AnalogInput FingerEncoder;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        hwMap = new HWMap(hardwareMap);
        FE = hwMap.getFingerServo();
        FingerEncoder = hwMap.getFingerEncoder();
        while (opModeIsActive()) {
            FE.set(1);
        }
    }
}