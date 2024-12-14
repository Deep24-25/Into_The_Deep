package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;

public class ArmFSM {


    private enum States {
        AT_BASKET_HEIGHT, AT_SUBMERSIBLE_HEIGHT, AT_SPECIMEN_PICKUP, AT_CHAMBER_LOCK_HEIGHT, AT_MINI_INTAKE, FULLY_RETRACTED, FULLY_EXTENDED, MOVING_ABOVE_SAFE_HEIGHT, MOVING_BELOW_SAFE_HEIGHT
    }

    //Random Values
    private static final double SAFE_HEIGHT = 5;
    private static final double BASKET_LOW = 65.4;
    private static final double BASKET_HIGH = 109.2;
    private static final double SUBMERSIBLE_LOW = 33;
    private static final double SUBMERSIBLE_HIGH = 66;
    private static final double FULLY_RETRACTED = 0;
    private static final double MINI_INTAKE = 7;
    private static final int MAX_HEIGHT = 80;//102 cm is physical max
    private static final double SPECIMEN_PICKUP = 0;
    private double chamberLockHeight = 60;
    private double[] submersibleHeights = {SUBMERSIBLE_LOW, SUBMERSIBLE_HIGH};
    private double[] basketHeights = {BASKET_LOW, BASKET_HIGH};
    private int submersibleIndex = 1;
    private int basketIndex = 1;


    public static double PHorizontal = 0.2, IHorizontal = 0.001, DHorizontal = 0.002, FHorizontal = 0;
    public static double PVertical = 0.2, IVertical = 0.001, DVertical = 0.002, FVertical = 0;
    public static double PAngle = 0.2, IAngle = 0.001, DAngle = 0.002, FAngle = 0;
    private final double[] intakeIndecies = {FULLY_RETRACTED, 10, 20, 30, 40, 50, 60, 70, MAX_HEIGHT};

    private final ArmMotorsWrapper armMotorsWrapper;
    private final PIDFController pidfController;
    private double targetPosition;
    private double measuredPosition;
    private States currentState;
    private int currentIndex;
    private double slidePowerCap = 1.0;
    private final double slideFinalMovementsCap = 1.0;
    private final double slideMovementCap = 1.0;

    private double power = 0;
    private static final double TOLERANCE = 2.0;

    private Logger logger;

    public ArmFSM(HWMap hwMap, Logger logger) {
        this.armMotorsWrapper = new ArmMotorsWrapper(hwMap);
        pidfController = new PIDFController(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
        currentIndex = 1;
        targetPosition = 0;
        pidfController.setTolerance(TOLERANCE);
        this.logger = logger;
    }

    @VisibleForTesting
    public ArmFSM(ArmMotorsWrapper armMotorsWrapper, PIDFController pidfController) {
        this.armMotorsWrapper = armMotorsWrapper;
        this.pidfController = pidfController;

    }

    public void updateState() {
        updatePIDF();
        pidfController.setTolerance(3);
        armMotorsWrapper.readPositionInCM();
        if (pidfController.atSetPoint()) {
            if (isTargetPosAtFullyRetractedHeight())
                currentState = States.FULLY_RETRACTED;
            else if (isTargetPosAtBasketHeight())
                currentState = States.AT_BASKET_HEIGHT;
            else if (isTargetPosAtSubmersibleHeight())
                currentState = States.AT_SUBMERSIBLE_HEIGHT;
            else if (isTargetPosSpecimenPickUpHeight()) {
                currentState = States.AT_SPECIMEN_PICKUP;
            } else if (isTargetPosChamberLockHeight()) {
                currentState = States.AT_CHAMBER_LOCK_HEIGHT;
            } else if (isTargetPosMiniIntakeHeight()) {
                currentState = States.AT_MINI_INTAKE;
            }
        } else if (isFullyExtended()) {
            currentState = States.FULLY_EXTENDED;
        } else {
            if (isTargetPosAboveSafeHeight())
                currentState = States.MOVING_ABOVE_SAFE_HEIGHT;
            else if (isTargetPosBelowSafeHeight())
                currentState = States.MOVING_BELOW_SAFE_HEIGHT;
        }
    }

    public boolean isFullyExtended() {
        return measuredPosition == MAX_HEIGHT;
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

    // get state
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

    public boolean AT_SPECIMEN_PICKUP_HEIGHT() {
        return currentState == States.AT_SPECIMEN_PICKUP;

    }

    public boolean AT_CHAMBER_LOCK_HEIGHT() {
        return currentState == States.AT_CHAMBER_LOCK_HEIGHT;

    }

    public boolean AT_MINI_INTAKE() {
        return currentState == States.AT_MINI_INTAKE;

    }

    public void updatePIDF() {
        armMotorsWrapper.readPositionInCM();
        if (targetPosition == BASKET_HIGH || targetPosition == BASKET_LOW)
            pidfController.setPIDF(PVertical, IVertical, DVertical, FVertical);
        else if (targetPosition == SUBMERSIBLE_HIGH || targetPosition == SUBMERSIBLE_LOW)
            pidfController.setPIDF(PAngle, IAngle, DAngle, FAngle);
        else
            pidfController.setPIDF(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
        measuredPosition = armMotorsWrapper.getLastReadPositionInCM();
        power = pidfController.calculate(measuredPosition, targetPosition);
        power = Math.min(Math.abs(power), Math.abs(slidePowerCap)) * Math.signum(power);
        armMotorsWrapper.set(power);
    }

    public void moveToSelectedIndexPosition() {
        targetPosition = intakeIndecies[currentIndex];
    }

    public void indexIncrement() {
        int tempIndex = currentIndex + 1;
        if (tempIndex < intakeIndecies.length) {
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

    public boolean isTargetPosSpecimenPickUpHeight() {
        return targetPosition == SPECIMEN_PICKUP;
    }

    public boolean isTargetPosChamberLockHeight() {
        return targetPosition == chamberLockHeight;
    }

    public boolean isTargetPosMiniIntakeHeight() {
        return targetPosition == MINI_INTAKE;
    }

    public void moveToMiniIntake() {
        targetPosition = MINI_INTAKE;
    }

    public void setPowerCapFinalMovements() {
        slidePowerCap = slideFinalMovementsCap;
    }

    public void setPowerCapMovement() {
        slidePowerCap = slideMovementCap;
    }


    public double getTolerance() {
        return TOLERANCE;
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

    public void moveToSubmersibleHeight() {
        targetPosition = submersibleHeights[submersibleIndex];
    }

    public void moveToBasketHeight() {
        targetPosition = basketHeights[basketIndex];
    }

    public void setIndexToSubmersibleLowHeight() {
        chamberLockHeight = 28;
        submersibleIndex = 0;
    }

    public void setIndexToSubmersibleHighHeight() {
        chamberLockHeight = 61;
        submersibleIndex = 1;
    }

    public void moveToSpecimenPickUpHeight() {
        targetPosition = SPECIMEN_PICKUP;
    }

    public void moveToChamberLockHeight() {
        targetPosition = chamberLockHeight;
    }

    public void setIndexToBasketHighHeight() {
        basketIndex = 1;
    }

    public void setIndexToBasketLowHeight() {
        basketIndex = 0;
    }

    public void goToBasketHeight() {
        targetPosition = basketHeights[basketIndex];
    }

    public void retract() {
        targetPosition = FULLY_RETRACTED;
    }

    public void moveToSafeHeight() {
        targetPosition = SAFE_HEIGHT;
    }

    public boolean isTargetPosHighHeight() {
        return targetPosition == submersibleHeights[1];
    }

    public boolean isTargetPosLowHeight() {
        return targetPosition == submersibleHeights[0];
    }

    public void log() {
        logger.log("-------------------------ARM LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Arm State: ", currentState, Logger.LogLevels.PRODUCTION);
        logger.log("Arm Current Height: ", armMotorsWrapper.getLastReadPositionInCM(), Logger.LogLevels.PRODUCTION);
        logger.log("Arm Target Height: ", targetPosition, Logger.LogLevels.PRODUCTION);
        logger.log("Current Index: ", currentIndex, Logger.LogLevels.PRODUCTION);
        logger.log("AtSetPoint(): ", pidfController.atSetPoint(), Logger.LogLevels.PRODUCTION);
        logger.log("-------------------------ARM LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);

    }


}