package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HWMap;

public class DeviatorServoWrapper {
    private final Servo deviatorServo;

    public DeviatorServoWrapper(HWMap hwMap) {
        deviatorServo = hwMap.getWristDeviServo();
    }

    public void set(double angle) {
        deviatorServo..((angle/360));
    }

    public void get() {
        deviatorServo.get()
    }
}
