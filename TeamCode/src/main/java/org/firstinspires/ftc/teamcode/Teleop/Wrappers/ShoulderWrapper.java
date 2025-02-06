package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class ShoulderWrapper {
    private final MotorEx shoulderMotor;
    private double lastReadAngle;
    private final double SLIDES_CPR;

    //ratio 24:41
    public ShoulderWrapper(HWMap hwMap, boolean reset) {
        shoulderMotor = hwMap.getPivotMotor();
        SLIDES_CPR = shoulderMotor.getCPR();
        shoulderMotor.resetEncoder();


    }

    public void set(double power) {
        shoulderMotor.set(power);
    }

    public double get() {
        return shoulderMotor.get();
    }

    public double readAngle() {
        double currentPositionInTicks = shoulderMotor.getCurrentPosition();
        double ratio = 24.0 / 41.0;

        lastReadAngle = ((360 * currentPositionInTicks * ratio) / (SLIDES_CPR)) % 360;

        return lastReadAngle;
    }

    public double getLastReadAngle() {
        return lastReadAngle;
    }

    public void resetEncoder() {
        shoulderMotor.resetEncoder();
    }

    public double getCurrent() {
        return shoulderMotor.motorEx.getCurrent(CurrentUnit.AMPS);
    }
}
