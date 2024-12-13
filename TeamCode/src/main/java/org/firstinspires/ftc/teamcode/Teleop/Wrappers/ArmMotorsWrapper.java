package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Core.HWMap;
public class ArmMotorsWrapper {
    private final Motor armMoterOne;
    private final Motor armMoterTwo;
    private final Motor armMoterThree;
    private double lastReadPositionInCM;
    private static double SLIDES_CPR;

    public ArmMotorsWrapper(HWMap hwMap){
        armMoterOne = hwMap.getArmMotorOne();
        armMoterTwo = hwMap.getArmMotorTwo();
        armMoterThree = hwMap.getArmMotorThree();
        armMoterOne.setInverted(true);
        SLIDES_CPR = armMoterOne.getCPR();
    }
    /**
     * Description: This method sets power to the linear slides motors
     * Parameters: power (-1:1)
     */
    public void set(double power){
        armMoterOne.set(power);
        armMoterTwo.set(power);
        armMoterThree.set(power);
    }
    /**
     * Description: This method reads and returns the position in CM of the linear slides (The most current value).
     * Parameters: None
     */
    public double readPositionInCM(){
        double currentPositionInTicks = armMoterOne.getCurrentPosition();
        double diameterOfSpool = 1;
        //double ratio = (0 / 0) * (0 / 0);
        lastReadPositionInCM = (currentPositionInTicks / (SLIDES_CPR)) * Math.PI * diameterOfSpool;
        return lastReadPositionInCM;
    }
    public double getCurrentVelocity(){
        double currentVelocity = armMoterOne.getCorrectedVelocity();
        double diameterOfSpool = 0;
     //   double ratio = (0 / 0) * (0 / 0);
        return (currentVelocity / (SLIDES_CPR)) * Math.PI * diameterOfSpool;
    }
    /**
     * Description: This method returns the value that was last read from the slides (Not to most current value).
     * Parameters: None
     */
    public double getLastReadPositionInCM() {
        return lastReadPositionInCM;
    }
}
