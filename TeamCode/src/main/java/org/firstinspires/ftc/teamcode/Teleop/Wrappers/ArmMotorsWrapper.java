package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class ArmMotorsWrapper {
    private final MotorEx armMoterOne;
    private final MotorEx armMoterTwo;
    private final MotorEx armMoterThree;
    private double lastReadPositionInCM;
    private static double SLIDES_CPR;
    //spool 24:36


    public ArmMotorsWrapper(HWMap hwMap, boolean reset) {
        armMoterOne = hwMap.getArmMotorOne();
        armMoterTwo = hwMap.getArmMotorTwo();
        armMoterThree = hwMap.getArmMotorThree();
        if (reset) {
            armMoterOne.resetEncoder();
        }
        SLIDES_CPR = armMoterOne.getCPR();
    }

    /**
     * Description: This method sets power to the linear slides motors
     * Parameters: power (-1:1)
     */
    public void set(double power) {
        armMoterOne.set(power);
        armMoterTwo.set(power);
        armMoterThree.set(power);
    }

    /**
     * Description: This method reads and returns the position in CM of the linear slides (The most current value).
     * Parameters: None
     */
    public double readPositionInCM() {
        double currentPositionInTicks = armMoterOne.getCurrentPosition();
        double diameterOfSpool = 4.7; //1.85 inches
        double ratio = (28.0 / 38.0) * (30.0 / 90.0);
        lastReadPositionInCM = ((currentPositionInTicks * ratio) / (SLIDES_CPR)) * Math.PI * diameterOfSpool;
        return lastReadPositionInCM;
    }

    /**
     * Description: This method returns the value that was last read from the slides (Not to most current value).
     * Parameters: None
     */
    public double getLastReadPositionInCM() {
        return lastReadPositionInCM;
    }

    public double getArmMotor2Angle() {
        return armMoterTwo.getCurrentPosition();
    }

    public double getArmMotor3Angle() {
        return armMoterThree.getCurrentPosition();
    }

    public void resetEncoder() {
        armMoterOne.resetEncoder();
    }

    public double currentVelocity() {
        return armMoterOne.getCorrectedVelocity();
    }

    public double getAM1Current() {
        return armMoterOne.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public double getAM2Current() {
        return armMoterTwo.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public double getAM1Velocity() {
        return armMoterOne.motorEx.getVelocity();
    }

    public double getAM3Current() {
        return armMoterThree.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public double get() {
        return armMoterOne.get();
    }
}
