package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Core.HWMap;

public class FingerAxonServoWrapper {
    private CRServo axon;
    private AnalogInput encoder;
    private double lastReadPosition;
    private double sign = 1;
    private double encoderOffset = 0.0;
    public FingerAxonServoWrapper(HWMap hwMap, boolean inversePower, boolean inverseEncoder) {
        this.axon = hwMap.getFingerServo();
        this.encoder = hwMap.getFingerEncoder();
        if (inversePower) {
            sign = -1;
        }
        if (inverseEncoder) {
            encoderOffset = 360;
        }
    }

    public void set(double power){
        axon.set(power * sign);
    }

    public double readPos() {
        lastReadPosition = Math.abs(encoderOffset - (encoder.getVoltage() / 3.3 * 360));
        return lastReadPosition;
    }
    public double getLastReadPos() {
        return lastReadPosition;
    }
}
