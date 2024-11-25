package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class DeviatorServoWrapper {
    private final ServoEx deviatorServo;
    private double lastReadAngle;

    public DeviatorServoWrapper(HWMap hwMap) {
        deviatorServo = hwMap.getWristDeviServo();
    }

    public void setAngle(double angle) {
        deviatorServo.turnToAngle(angle, AngleUnit.DEGREES);
    }

    public double readAngle() {
        lastReadAngle = deviatorServo.getAngle();
        return lastReadAngle;
    }
    public double getLastReadAngle() {
        return lastReadAngle;
    }
}
