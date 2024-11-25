package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;

public class ArmFSM {
    private enum States {
        AT_BASKET_HEIGHT, AT_SUBMERSIBLE_HEIGHT, FULLY_RETRACTED, FULLY_EXTENDED, MOVING_ABOVE_SAFE_HEIGHT, MOVING_BELOW_SAFE_HEIGHT
    }

    //Random Values
    private static final double SAFE_HEIGHT = 3;
    private static final double BASKET_LOW = 6;
    private static final double BASKET_HIGH = 10;
    private static final double SUBMERSIBLE_LOW = 4;
    private static final double SUBMERSIBLE_HIGH = 8;
    private static final double FULLY_RETRACTED = 0;
    private static final int MAX_HEIGHT = 0;
    public static double PHorizontal = 0, IHorizontal = 0, DHorizontal = 0, FHorizontal = 0;
    public static double PVertical = 0, IVertical = 0, DVertical = 0, FVertical = 0;
    public static double PAngle = 0, IAngle = 0, DAngle = 0, FAngle = 0;
    private final double[] horizontalIndex = {FULLY_RETRACTED, MAX_HEIGHT};

    private final ArmMotorsWrapper armMotorsWrapper;
    private final PIDFController pidfController;
    private double targetPosition;
    private double measuredPosition;
    private States currentState = States.FULLY_RETRACTED;
    private int currentIndex;
    private boolean slowMovement;
    private double slidePowerCap = 1.0;
    private final double slideFinalMovementsCap = 1.0;
    private final double slideMovementCap = 1.0;

    private double power = 0;
    private double tolerance = 1.5;

    public ArmFSM(HWMap hwMap, ArmMotorsWrapper armMotorsWrapper) {
        this.armMotorsWrapper = armMotorsWrapper;
        pidfController = new PIDFController(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
        currentIndex = 1;
        targetPosition = 0;
        pidfController.setTolerance(tolerance);
    }

    public void updateState() {
        armMotorsWrapper.readPositionInCM();
        if (atPos(tolerance)) {
            if (isTargetPosAtFullyRetractedHeight())
                currentState = States.FULLY_RETRACTED;
            else if (isTargetPosAtBasketHeight())
                currentState = States.AT_BASKET_HEIGHT;
            else if (isTargetPosAtSubmersibleHeight())
                currentState = States.AT_SUBMERSIBLE_HEIGHT;
            else
                currentState = States.FULLY_EXTENDED;
        } else {
            measuredPosition = armMotorsWrapper.getLastReadPositionInCM();
            if (isTargetPosAboveSafeHeight())
                currentState = States.MOVING_ABOVE_SAFE_HEIGHT;
            else if (isTargetPosBelowSafeHeight())
                currentState = States.MOVING_BELOW_SAFE_HEIGHT;
        }

    }

    public void setHorizontalPID() {
        pidfController.setPIDF(PHorizontal, IHorizontal, DHorizontal, FHorizontal);

    }

    public void setVerticalPID() {
        pidfController.setPIDF(PVertical, IVertical, DVertical, FVertical);
    }

    public void setAngularPID() {
        pidfController.setPIDF(PAngle, IAngle, DAngle, FAngle);
    }

    // set state
    public boolean AT_BASKET_HEIGHT() {
        return currentState == States.AT_BASKET_HEIGHT;
    }

    public boolean AT_SUBMERSIBLE_HEIGHT() {
        return currentState == States.AT_SUBMERSIBLE_HEIGHT;
    }

    public boolean FULLY_RETRACTED() {
        return currentState == States.FULLY_RETRACTED;
    }

    public boolean FULLY_EXTENDED() {
        return currentState == States.FULLY_EXTENDED;
    }

    public boolean MOVING_ABOVE_SAFE_HEIGHT() {
        return currentState == States.MOVING_ABOVE_SAFE_HEIGHT;
    }

    public boolean MOVING_BELOW_SAFE_HEIGHT() {
        return currentState == States.MOVING_BELOW_SAFE_HEIGHT;
    }

    public void updatePIDF() {
        // TODO: Can use sample and specimen mode instead
        if (targetPosition == BASKET_HIGH || targetPosition == BASKET_LOW)
            pidfController.setPIDF(PVertical, IVertical, DVertical, FVertical);
        else if (targetPosition == SUBMERSIBLE_HIGH || targetPosition == SUBMERSIBLE_LOW)
            pidfController.setPIDF(PAngle, IAngle, DAngle, FAngle);
        else
            pidfController.setPIDF(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
        measuredPosition = armMotorsWrapper.getLastReadPositionInCM();
        power = pidfController.calculate(measuredPosition, targetPosition);
        power = Math.min(Math.abs(power), Math.abs(slidePowerCap)) * Math.signum(power);
    }

    public void moveToSelectedIndexPosition() {
        targetPosition = currentIndex;
        pidfController.setPIDF(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
    }

    public void moveToSafeHeight() {
        targetPosition = SAFE_HEIGHT;
        pidfController.setPIDF(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
    }

    public void retractToIntake() {
        targetPosition = FULLY_RETRACTED;
        pidfController.setPIDF(PAngle, IAngle, DAngle, FAngle);
    }

    public void indexIncrement() {
        int tempIndex = currentIndex + 1;
        if (tempIndex < horizontalIndex.length) {
            currentIndex++;
        }
    }

    public void indexDecrement() {
        int tempIndex = currentIndex - 1;
        if (tempIndex >= 0) {
            currentIndex--;
        }
    }

    public boolean isTargetPosAboveSafeHeight() {
        return targetPosition >= SAFE_HEIGHT;
    }

    public boolean isTargetPosBelowSafeHeight() {
        return targetPosition < SAFE_HEIGHT;
    }

    public boolean isTargetPosAtFullyRetractedHeight() {
        return targetPosition == FULLY_RETRACTED;
    }

    public boolean isTargetPosAtBasketHeight() {
        return targetPosition == BASKET_HIGH || targetPosition == BASKET_LOW;
    }

    public boolean isTargetPosAtSubmersibleHeight() {
        return targetPosition == SUBMERSIBLE_HIGH || targetPosition == SUBMERSIBLE_LOW;
    }

    public boolean atTargetPos() {
        return targetPosition == measuredPosition;
    }

    public void setPowerCapFinalMovements() {
        slidePowerCap = slideFinalMovementsCap;
    }

    public void setPowerCapMovement() {
        slidePowerCap = slideMovementCap;
    }

    public boolean atPos(double tolerance) {
        return ((targetPosition + tolerance) >= measuredPosition) && ((targetPosition - tolerance) <= measuredPosition);
    }

    public double getTolerance() {
        return tolerance;
    }

    public boolean targetIsSafeHeight() {
        return targetPosition == SAFE_HEIGHT;
    }

    public static double getFullyRetractedHeight() {
        return FULLY_RETRACTED;
    }

    public void setCurrentIndex(int currentIndex) {
        this.currentIndex = currentIndex;
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    //TODO: COULD NOT FIND WORKING INDEXING CODE SO ADDED THIS DUMMY FUNCTION FOR THE TEST, PLEASE FIX
    public void goToSpecimenPickupHeight() {

    }

    public boolean atSpecimenPickupHeight() {
        return false;
    }
    public void moveToChamberLockHeight(){

    }
}