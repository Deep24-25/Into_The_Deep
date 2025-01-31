package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;


import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;

import java.util.concurrent.TimeUnit;

@Config
public class ArmFSM {


    private enum States {
        AT_BASKET_HEIGHT, AT_SUBMERSIBLE_HEIGHT, AT_SPECIMEN_PICKUP, AT_CHAMBER_LOCK_HEIGHT, AT_MINI_INTAKE, FULLY_RETRACTED, FULLY_EXTENDED, MOVING_ABOVE_SAFE_HEIGHT, MOVING_BELOW_SAFE_HEIGHT, EXTENDED, MOVED_TO_AUTO_SPEC_INTAKE, EXTENDING_TO_INTAKE_SPECiMEN, EXTENDED_TO_INTAKE_SPECiMEN
    }

    private static final double SAFE_HEIGHT = 1;
    public static double BASKET_LOW = 40;
    public static double BASKET_HIGH = 68;
    public static double SUBMERSIBLE_HIGH = 34;

    private static final double FULLY_RETRACTED = 4;
    private static final double MINI_INTAKE = 7;
    private static final double MAX_HEIGHT = 37;//102 cm is physical max
    private static final double SPECIMEN_PICKUP = 2;
    private static final double AUTO_SPEC_INTAKE = 23;

    public static double chamberLockHeight = SUBMERSIBLE_HIGH + 18;
    private final double[] basketHeights = {BASKET_LOW, BASKET_HIGH};
    private int basketIndex = 1;

    public static double MAX_FEEDRATE = 0.5; // cm/sec

    public static double PHorizontal = 0.12, IHorizontal = 0.1, DHorizontal = 0.004, FHorizontal = 0;
    public static double PVertical = 0.12, IVertical = 0.1, DVertical = 0.004, FVertical = 0.003;
    public static double P_E_Horizontal = 0.12, I_E_Horizontal = 0.1, D_E_Horizontal = 0.004, F_E_Horizontal = 0;
    public static double PLinearizing = 0.12, ILinearizing = 0.1, DLinearizing = 0.004, FLinearizing = 0;

    private final ArmMotorsWrapper armMotorsWrapper;
    private final PIDFController pidfController;
    private final ShoulderFSM shoulderFSM;
    private double targetPosition;
    private double measuredPosition;
    private States currentState;
    private double slidePowerCap = 0.6;
    public static double extendingToIntakeSpecimenHeight = 15.0;
    private static double TOLERANCE = 2.0;

    private Logger logger;
    private final Timing.Timer timer;
    private double rightY = 0;
    private double currentFeedrate = 0;

    private boolean shouldPID = true;

    public ArmFSM(HWMap hwMap, Logger logger, ShoulderFSM shoulderFSM, ElbowFSM elbowFSM) {
        this.armMotorsWrapper = new ArmMotorsWrapper(hwMap);
        pidfController = new PIDFController(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
        targetPosition = FULLY_RETRACTED;
        pidfController.setTolerance(TOLERANCE);
        this.logger = logger;
        this.shoulderFSM = shoulderFSM;
        timer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS);
    }

    @VisibleForTesting
    public ArmFSM(ArmMotorsWrapper armMotorsWrapper, PIDFController pidfController, Timing.Timer timer, ShoulderFSM shoulderFSM) {
        this.armMotorsWrapper = armMotorsWrapper;
        this.pidfController = pidfController;
        this.timer = timer;
        this.shoulderFSM = shoulderFSM;
    }


    public void updateState(double rightY) {
        updatePIDF();
        this.rightY = rightY;
        armMotorsWrapper.readPositionInCM();
        timer.start();
        if (shoulderFSM.AT_BASKET_DEPOSIT() || shoulderFSM.AT_DEPOSIT_CHAMBERS() || shoulderFSM.GOING_TO_BASKET() || shoulderFSM.GOING_TO_CHAMBER()) {
            setVerticalPID();
            setTolerance(TOLERANCE);
        } else if (shoulderFSM.AT_INTAKE() || shoulderFSM.GOING_TO_INTAKE()) {
            if (isTargetPosAtFullyRetractedHeight() || isTargetPosAtAutoSpecimenIntake()) {
                setHorizontalPID();
                setTolerance(TOLERANCE);
            } else {
                setFeedPID();
                setTolerance(TOLERANCE);

            }
        }

        if (pidfController.atSetPoint()  && !isTargetPosAtAutoSpecimenIntake()) {
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
            } else if (isTargetPosAtExtendingToIntakeSpecimenHeight()) {
                currentState = States.EXTENDED_TO_INTAKE_SPECiMEN;
            } else {
                currentState = States.EXTENDED;
            }
        } else if (isFullyExtended()) {
            currentState = States.FULLY_EXTENDED;
        } else if (isTargetPosAtAutoSpecimenIntake()) {
            if(pidfController.atSetPoint() || armMotorsWrapper.getLastReadPositionInCM() >= AUTO_SPEC_INTAKE) {
                currentState = States.MOVED_TO_AUTO_SPEC_INTAKE;
            }
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

    public void setFeedPID() {
        pidfController.setPIDF(P_E_Horizontal, I_E_Horizontal, D_E_Horizontal, F_E_Horizontal);
    }

    public void setLinearizingPID() {
        pidfController.setPIDF(PLinearizing, ILinearizing, DLinearizing, FLinearizing);
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

    public boolean AT_SPECIMEN_PICKUP_HEIGHT() {
        return currentState == States.AT_SPECIMEN_PICKUP;
    }
    public boolean EXTENDED_TO_INTAKE_SPECiMEN() {
        return currentState == States.EXTENDED_TO_INTAKE_SPECiMEN;

    }

    public boolean AT_CHAMBER_LOCK_HEIGHT() {

        return currentState == States.AT_CHAMBER_LOCK_HEIGHT;
    }

    public boolean MOVED_TO_AUTO_SPEC_INTAKE() {
        return currentState == States.MOVED_TO_AUTO_SPEC_INTAKE;
    }


    public boolean AT_MINI_INTAKE() {
        return currentState == States.AT_MINI_INTAKE;
    }

    public void moveToExtendingToIntakeSpecimen(){
        targetPosition = extendingToIntakeSpecimenHeight;
    }

    public void updatePIDF() {
        armMotorsWrapper.readPositionInCM();
        if (shouldPID) {
            // hwMap.brakingOff();
            measuredPosition = armMotorsWrapper.getLastReadPositionInCM();
            double power = pidfController.calculate(measuredPosition, targetPosition);
            power = Math.min(Math.abs(power), Math.abs(slidePowerCap)) * Math.signum(power);
            armMotorsWrapper.set(power);
        }


    }
    public boolean isTargetPosAtAutoSpecimenIntake() {
        return targetPosition == AUTO_SPEC_INTAKE;
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

    public boolean isTargetPosAtExtendingToIntakeSpecimenHeight() {
        return targetPosition == extendingToIntakeSpecimenHeight;
    }

    public boolean isTargetPosAtBasketHeight() {
        return targetPosition == BASKET_HIGH || targetPosition == BASKET_LOW;
    }

    public boolean isTargetPosAtSubmersibleHeight() {
        return targetPosition == SUBMERSIBLE_HIGH;
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


    public void moveToSubmersibleHeight() {
        targetPosition = SUBMERSIBLE_HIGH;
    }


    public void setIndexToBasketLowHeight() {
        basketIndex = 0;
    }

    public void setIndexToBasketHighHeight() {
        basketIndex = 1;
    }


    public void moveToChamberLockHeight() {
        targetPosition = chamberLockHeight;
    }


    public void goToBasketHeight() {
        targetPosition = basketHeights[basketIndex];
    }

    public void retract() {
        targetPosition = FULLY_RETRACTED;
    }

    public void setTolerance(double tolerance) {
        TOLERANCE = tolerance;
    }

    public void feed() {
        //35368.421 cpr of motor per one rotation
        targetPosition = armMotorsWrapper.getLastReadPositionInCM();
        shouldPID = false;
        currentFeedrate = MAX_FEEDRATE * rightY;

        if (targetPosition > (MAX_HEIGHT - 2)) {
            // hwMap.brakingOn();
            if (rightY < 0)
                currentFeedrate = Math.max(Math.min(currentFeedrate, 0), -1);
            else
                shouldPID = true;
            targetPosition = MAX_HEIGHT;
        } else {
            //Protects the arm from over-extending and over-retracting
            if (targetPosition <= (MAX_HEIGHT - 2) && targetPosition >= 0) {
                //   hwMap.brakingOff();
                currentFeedrate = Math.max(Math.min(currentFeedrate, 1), -1);
            } else {
                //   hwMap.brakingOff();
                currentFeedrate = Math.max(currentFeedrate, 0);
            }

            /*//Slows the arm down when close to min and max.
            if (currentFeedrate < 0 && targetPosition < 10) {
                currentFeedrate = -Math.min(Math.abs(currentFeedrate), (0.02 * targetPosition));
            } else if (currentFeedrate > 0 && targetPosition > (MAX_HEIGHT - 10)) {
                currentFeedrate = Math.min(currentFeedrate, (0.02 * (MAX_HEIGHT - 10)));
            }*/

        }
        armMotorsWrapper.set(currentFeedrate);

    }

    public void resetArm(boolean dPadUpIsDown, boolean dpadUpIsReleased) {
        if (dPadUpIsDown) {
            shouldPID = false;
            armMotorsWrapper.set(-0.3);
        }
        if (dpadUpIsReleased) {
            armMotorsWrapper.resetEncoder();
            shouldPID = true;
            targetPosition = FULLY_RETRACTED;
        }

    }

    public void log() {
        logger.log("-------------------------ARM LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Arm State: ", currentState, Logger.LogLevels.PRODUCTION);
        logger.log("Arm Current Height: ", armMotorsWrapper.getLastReadPositionInCM(), Logger.LogLevels.PRODUCTION);
        logger.log("Arm Target Height: ", targetPosition, Logger.LogLevels.PRODUCTION);
        logger.log("AtSetPoint(): ", pidfController.atSetPoint(), Logger.LogLevels.DEBUG);
        logger.log("power cap", slidePowerCap, Logger.LogLevels.DEBUG);
        logger.log("Current power", armMotorsWrapper.get(), Logger.LogLevels.DEBUG);
        logger.log("rightY", rightY, Logger.LogLevels.DEBUG);

        logger.log("-------------------------ARM LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);

    }

    public void capSetPower() {
        double currentPos = armMotorsWrapper.getLastReadPositionInCM();
        if (Math.abs(currentPos - targetPosition) <= 20) {
            slidePowerCap = 0.02 * Math.abs(currentPos - targetPosition);
        } else
            slidePowerCap = 0.4;
    }

    public void uncapSetPower() {
        slidePowerCap = 0.6;
    }

    public double getCurrentHeight() {
        return armMotorsWrapper.getLastReadPositionInCM();
    }

    public void setShouldPID(boolean setShouldPID) {
        this.shouldPID = setShouldPID;
    }

    public void setAutoSpecIntakePos() {
        targetPosition = AUTO_SPEC_INTAKE;
    }

    public double getCurrentFeedrate() {
        return currentFeedrate;
    }

    public static double getMaxFeedrate() {
        return MAX_FEEDRATE;
    }

    @VisibleForTesting
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

}