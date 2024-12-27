package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Core.HWMap;

public class ArmMotorsWrapper {
    private final Motor armMoterOne;
    private final Motor armMoterTwo;
    private final Motor armMoterThree;
    private double lastReadPositionInCM;
    private static double SLIDES_CPR;
    //spool 24:36


    public ArmMotorsWrapper(HWMap hwMap) {
        armMoterOne = hwMap.getArmMotorOne();
        armMoterTwo = hwMap.getArmMotorTwo();
        armMoterThree = hwMap.getArmMotorThree();
        armMoterOne.resetEncoder();
        armMoterTwo.resetEncoder();
        armMoterThree.resetEncoder();
        SLIDES_CPR = armMoterOne.getCPR();
    }

    /**
     * Description: This method sets power to the linear slides motors
     * Parameters: power (-1:1)
     */
    public void set(double power) {
//        armMoterOne.set(power);
//        armMoterTwo.set(power);
//        armMoterThree.set(power);
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
        double ratio = (24.0 / 38.0) * (30.0/90.0);
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
    public void resetEncoder(){
        armMoterOne.resetEncoder();
    }
    public double currentVelocity(){
        return armMoterOne.getCorrectedVelocity();
    }

    public double get() {
        return armMoterOne.get();
    }
}
