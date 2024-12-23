package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AxonServoWrapper {
    private CRServo axon;
    private AnalogInput encoder;
    private double lastReadPosition;
    private double sign = 1;
    private double encoderOffset;
    private double inverseEncoderOffset;

    public AxonServoWrapper(CRServo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset) {
        this.axon = axon;
        this.encoder = encoder;
        if (inversePower) {
            sign = -1;
        }
        if (inverseEncoder) {
            inverseEncoderOffset = 360;
        }

        this.encoderOffset = encoderOffset;
    }

    public void set(double power) {
        axon.set(power * sign);
    }

    public double get() {
        return axon.get() * sign;
    }

    public double readPos() {

        lastReadPosition = (Math.abs(inverseEncoderOffset - (((encoder.getVoltage() / 3.3 * 360)) + encoderOffset))) % 360;
        return lastReadPosition;
    }

    public double readRawPos() {
        return (encoder.getVoltage() / 3.3 * 360);
    }

    public double getRawPos() {
        return (encoder.getVoltage() / 3.3 * 360);
    }

    public double getLastReadPos() {
        return lastReadPosition;
    }
}
