package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class FingerServoWrapper {
    private final ServoEx fingerServo;

    public FingerServoWrapper(HWMap hwMap) {
        fingerServo = hwMap.getFingerServo();
    }

    public void setAngle(double angle) {
        fingerServo.turnToAngle(angle, AngleUnit.DEGREES);
    }

    public double readAngle() {
        return fingerServo.getAngle();
    }

    public double getPos() {
        return fingerServo.getPosition();
    }
}
