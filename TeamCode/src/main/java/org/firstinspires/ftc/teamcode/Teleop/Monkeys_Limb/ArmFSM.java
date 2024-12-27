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
        AT_BASKET_HEIGHT, AT_SUBMERSIBLE_HEIGHT, AT_SPECIMEN_PICKUP, AT_CHAMBER_LOCK_HEIGHT, AT_MINI_INTAKE, FULLY_RETRACTED, AT_ZERO, FULLY_EXTENDED, MOVING_ABOVE_SAFE_HEIGHT, MOVING_BELOW_SAFE_HEIGHT, LINEARIZED, EXTENDED
    }

    //Random Values
   /* private static final double SAFE_HEIGHT = 0;
    private static final double BASKET_LOW = 65.4;
    private static final double BASKET_HIGH = 109.2;
    private static final double SUBMERSIBLE_LOW = 33;
    private static final double SUBMERSIBLE_HIGH = 66;*/

    //WHEN PIVOTING THE ARM OFFSETS BY 3 cm

    private static final double SAFE_HEIGHT = 1;
    private static final double BASKET_LOW = 40;
    private static final double BASKET_HIGH = 78;
    private static final double SUBMERSIBLE_LOW = 17;
    private static final double SUBMERSIBLE_HIGH = 18;

    private static double FULLY_RETRACTED = 4;
    private static final double MINI_INTAKE = 7;
    private static final int MAX_HEIGHT = 60;//102 cm is physical max
    private static final double SPECIMEN_PICKUP = 2;

    private double SAMPLE_PICKUP_LINEARIZATION_OFFSET = 0; // 2.1734 cm
    private double chamberLockHeight = 60;
    private double[] submersibleHeights = {SUBMERSIBLE_LOW, SUBMERSIBLE_HIGH};
    private double[] basketHeights = {BASKET_LOW, BASKET_HIGH};
    private int submersibleIndex = 1;
    private int basketIndex = 1;
    private double prevTime = 0, currentTime = 0;

    public static double MAX_FEEDRATE = 0.8; // cm/sec

    private double feedPos = 0.0;
    public static double PHorizontal = 0.12, IHorizontal = 0.1, DHorizontal = 0.004, FHorizontal = 0;
    public static double PVertical = 0.12, IVertical = 0.1, DVertical = 0.004, FVertical = 0.003;
    public static double P_E_Horizontal = 0.12, I_E_Horizontal = 0.1, D_E_Horizontal = 0.004, F_E_Horizontal = 0;
    public static double PLinearizing = 0.12, ILinearizing = 0.1, DLinearizing = 0.004, FLinearizing = 0;

    private final double[] intakeIndecies = {FULLY_RETRACTED, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};

    private final ArmMotorsWrapper armMotorsWrapper;
    private final PIDFController pidfController;
    private final ShoulderFSM shoulderFSM;
    private final ElbowFSM elbowFSM;
    private LimbFSM limbFSM;
    private double targetPosition;
    private double measuredPosition;
    private States currentState;
    private int currentIndex;
    private double slidePowerCap = 1.0;
    private final double slideFinalMovementsCap = 1.0;
    private final double slideMovementCap = 1.0;
    private double prevPosition = 0;

    private double power = 0;
    private static double TOLERANCE = 4.0;

    private Logger logger;
    private Timing.Timer timer;
    private double rightY = 0;
    private double currentFeedrate = 0;

    private boolean shouldPID = true;
    private double lastLastReadPos = 0;

    public ArmFSM(HWMap hwMap, Logger logger, ShoulderFSM shoulderFSM, ElbowFSM elbowFSM, LimbFSM limbFSM) {
        this.armMotorsWrapper = new ArmMotorsWrapper(hwMap);
        pidfController = new PIDFController(PHorizontal, IHorizontal, DHorizontal, FHorizontal);
        currentIndex = 1;
        targetPosition = FULLY_RETRACTED;
        pidfController.setTolerance(TOLERANCE);
        this.logger = logger;
        this.shoulderFSM = shoulderFSM;
        this.elbowFSM = elbowFSM;
        this.limbFSM = limbFSM;
        timer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS);
    }

    @VisibleForTesting
    public ArmFSM(ArmMotorsWrapper armMotorsWrapper, PIDFController pidfController, ShoulderFSM shoulderFSM, ElbowFSM elbowFSM, LimbFSM limbFSM) {
        this.armMotorsWrapper = armMotorsWrapper;
        this.pidfController = pidfController;
        this.shoulderFSM = shoulderFSM;
        this.elbowFSM = elbowFSM;
        this.limbFSM = limbFSM;

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
            if (isTargetPosAtFullyRetractedHeight()) {
                setHorizontalPID();
                setTolerance(TOLERANCE);
            } else if (limbFSM.LINEARIZING_INTAKE()) {
                setLinearizingPID();
                linearizeIntakePos();
                targetPosition = feedPos - 0;
            } else {
                setFeedPID();
                setTolerance(TOLERANCE);

            }
        }

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
            } else {
                currentState = States.EXTENDED;
            }
        } else if (isFullyExtended()) {
            currentState = States.FULLY_EXTENDED;
        } else {
            if (isTargetPosAboveSafeHeight())
                currentState = States.MOVING_ABOVE_SAFE_HEIGHT;
            else if (isTargetPosBelowSafeHeight())
                currentState = States.MOVING_BELOW_SAFE_HEIGHT;
        }
        lastLastReadPos = elbowFSM.getElbowCurrentAngle();
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

    public boolean AT_ZERO() {
        return currentState == States.AT_ZERO;
    }

    public boolean AT_MINI_INTAKE() {
        return currentState == States.AT_MINI_INTAKE;
    }

    public boolean LINEARIZED() {
        return currentState == States.LINEARIZED;
    }

    public void updatePIDF() {
        armMotorsWrapper.readPositionInCM();
        if (shouldPID) {
            measuredPosition = armMotorsWrapper.getLastReadPositionInCM();
            power = pidfController.calculate(measuredPosition, targetPosition);
            power = Math.min(Math.abs(power), Math.abs(slidePowerCap)) * Math.signum(power);
            armMotorsWrapper.set(power);
        }


    }


    public void indexIncrement() {
        int tempIndex = currentIndex + 1;
        if (tempIndex < intakeIndecies.length - 1) {
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

    public boolean isTargetPosAtZero() {
        return targetPosition == 0;
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


    public void setIndexToSubmersibleLowHeight() {
        chamberLockHeight = 28;
        submersibleIndex = 0;
    }

    public void setIndexToSubmersibleHighHeight() {
        chamberLockHeight = 61;
        submersibleIndex = 1;
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

    public void linearizeIntakePos() {
        SAMPLE_PICKUP_LINEARIZATION_OFFSET = Math.abs(((15.5 * Math.cos(Math.toRadians(180 - elbowFSM.getElbowCurrentAngle())))) - ((15.5 * Math.cos(Math.toRadians(Math.toRadians(180 - elbowFSM.getIntakeReadyAngle()))))));
    }


    public void retract() {

        targetPosition = FULLY_RETRACTED;
    }


    public void goToZero() {
        targetPosition = 0;
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

    public void setTolerance(double tolerance) {
        TOLERANCE = tolerance;
    }

    public void feed() {
        //35368.421 cpr of motor per one rotation
        shouldPID = false;
        currentTime = timer.elapsedTime();
        currentFeedrate = MAX_FEEDRATE * rightY;

        targetPosition = armMotorsWrapper.getLastReadPositionInCM();

        if (targetPosition <= 60 && targetPosition >= 0) {
            currentFeedrate = Math.max(Math.min(currentFeedrate, 1), -1);
        } else if (targetPosition >= 60) {
            currentFeedrate = Math.max(Math.min(currentFeedrate, 0), -1);
        } else {
            currentFeedrate = Math.max(currentFeedrate, 0);
        }

        armMotorsWrapper.set(currentFeedrate);

    }

    public void resetArm(boolean dPadUpIsDown, boolean dpadUpIsReleased) {
        if (dPadUpIsDown) {
            shouldPID = false;
            armMotorsWrapper.set(-0.6);
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
        logger.log("AtSetPoint(): ", pidfController.atSetPoint(), Logger.LogLevels.PRODUCTION);
        logger.log("Feed Pos:", feedPos, Logger.LogLevels.PRODUCTION);
        logger.log("Current feedrate: ", currentFeedrate, Logger.LogLevels.PRODUCTION);
        logger.log("Linearization offset", SAMPLE_PICKUP_LINEARIZATION_OFFSET, Logger.LogLevels.PRODUCTION);
        logger.log("power cap", slidePowerCap, Logger.LogLevels.PRODUCTION);
        logger.log("Current power", armMotorsWrapper.get(), Logger.LogLevels.PRODUCTION);
        logger.log("-------------------------ARM LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);

    }

    public void resetEncoder() {
        armMotorsWrapper.resetEncoder();
    }

    public void setLimbFSM(LimbFSM limbFSM) {
        this.limbFSM = limbFSM;
    }

    public double getCurrentVelocity() {
        return armMotorsWrapper.currentVelocity();
    }

    public void capSetPower(boolean cap) {
        if (cap)
            slidePowerCap = 0.4;
        else slidePowerCap = 1.0;
    }

    public double getCurrentHeight() {
        return armMotorsWrapper.getLastReadPositionInCM();
    }

    public double getFullyRetracted() {
        return FULLY_RETRACTED;
    }

    public double getMaxHeight() {
        return MAX_HEIGHT;
    }

    public void setShouldPID(boolean setShouldPID) {
        this.shouldPID = setShouldPID;
    }

}