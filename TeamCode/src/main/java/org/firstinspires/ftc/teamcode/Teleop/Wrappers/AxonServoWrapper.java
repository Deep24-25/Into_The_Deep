package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class AxonServoWrapper {
    private Servo axon;
    private AnalogInput encoder;
    private double lastReadPosition;
    private double sign = 1;
    private double encoderOffset;
    private double inverseEncoderOffset;

    private double ratio = 0;

    public AxonServoWrapper(Servo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset, double ratio) {
        this.axon = axon;
        this.ratio = ratio;
        this.encoder = encoder;
        if (inversePower) {
            sign = -1;
        }
        if (inverseEncoder) {
            inverseEncoderOffset = 360;
        }

        this.encoderOffset = encoderOffset;
    }

    /*public void set(double power) {
        axon.set(power * sign);
    }
*/
  /*  public double get() {
        return axon.get() * sign;
    }
*/

    public void set(double pos) {
        axon.setPosition((((pos/ratio) - encoderOffset) / 360.0));
    }

    public double readPos() {

        lastReadPosition = (Math.abs(inverseEncoderOffset - (((encoder.getVoltage() / 3.3 * 360) + encoderOffset)))) % 360;

        return lastReadPosition * ratio;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    public double getRawPos() {
        return (encoder.getVoltage() / 3.3);
    }

    public void setRawPos(double pos) {
        axon.setPosition(pos);
    }

    public double getLastReadPos() {
        return lastReadPosition;
    }

    public void setEncoderOffset(double offset) {
        this.encoderOffset = offset;

    }
}
