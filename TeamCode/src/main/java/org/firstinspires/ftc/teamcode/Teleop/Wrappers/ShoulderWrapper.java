package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Core.HWMap;

public class ShoulderWrapper {
    private final Motor shoulderMotor;
    private double lastReadAngle;
    private static final int ENCODER_OFFSET = 360;
    private double SLIDES_CPR;

    //ratio 24:41
    public ShoulderWrapper(HWMap hwMap) {
        shoulderMotor = hwMap.getPivotMotor();
        SLIDES_CPR = shoulderMotor.getCPR();

    }

    public void set(double power) {
        shoulderMotor.set(power);
    }

    public double readAngle() {
        double currentPositionInTicks = shoulderMotor.getCurrentPosition();
        double ratio = 24.0 / 41.0;

        lastReadAngle = ((currentPositionInTicks * ratio) / (SLIDES_CPR)) * Math.PI;

        return lastReadAngle;
    }

    public double getLastReadAngle() {
        return lastReadAngle;
    }
}
