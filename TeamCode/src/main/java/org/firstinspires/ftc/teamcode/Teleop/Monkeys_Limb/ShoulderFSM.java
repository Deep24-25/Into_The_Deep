package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;

import java.util.concurrent.TimeUnit;

@Config
public class ShoulderFSM {

    public enum States {
        GOING_TO_CHAMBER, AT_DEPOSIT_CHAMBERS, GOING_TO_INTAKE, AT_INTAKE, GOING_TO_BASKET, AT_BASKET_DEPOSIT, GOING_TO_SPECIMEN_INTAKE, AT_SPECIMEN_INTAKE
    }


    public static double P_E = 0.07;
    public static double I_E = 0.04;
    public static double D_E = 0.07;
    public static double F_E = 0.02;


    public static double P_R = 0.07;
    public static double I_R = 0.04;
    public static double D_R = 0.07;
    public static double F_R = 0.02;

    private static final double SAMPLE_INTAKE_ANGLE = 0;

    private static final double CHAMBER_ANGLE_LOW = 15;
    private static final double CHAMBER_ANGLE_HIGH = 43;
    private static final double BASKET_ANGLE_LOW = 100;
    private static final double BASKET_ANGLE_HIGH = 100;

    private static final double SPECIMEN_INTAKE_ANGLE = 50;

    private final ShoulderWrapper shoulderWrapper;
    private final PIDFController pidfController;
    private double targetAngle = SAMPLE_INTAKE_ANGLE;
    private double[] chamberAngles = {CHAMBER_ANGLE_LOW, CHAMBER_ANGLE_HIGH};
    private double[] basketAngles = {BASKET_ANGLE_LOW, BASKET_ANGLE_HIGH};

    private int chamberIndex = 1;
    private int basketIndex = 1;
    private double measuredAngle;
    private States currentState;

    private double lastPIDAngle = 0;
    private double power;
    private static double TOLERANCE = 7.5;
    private Timing.Timer resetTimer;

    private Logger logger;
    private boolean timerStarted = false;
    private int resetted = 0;

    private boolean shouldPID = true;
    private boolean pressedAlr = false;

    public ShoulderFSM(HWMap hwMap, Logger logger) {
        this.pidfController = new PIDFController(P_E, I_E, D_E, F_E);
        shoulderWrapper = new ShoulderWrapper(hwMap);
        this.logger = logger;
        resetTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    }

    @VisibleForTesting
    public ShoulderFSM(ShoulderWrapper shoulderWrapper, PIDFController pidfController) {
        this.shoulderWrapper = shoulderWrapper;
        this.pidfController = pidfController;

    }

    public void updateState() {
        pidfController.setTolerance(TOLERANCE);
        updatePID();

        if (pidfController.atSetPoint()) {
            if (isShoulderTargetPosDepositChamberAngle()) {
                currentState = States.AT_DEPOSIT_CHAMBERS;
            } else if (isShoulderTargetPosDepositBasketAngle()) {
                currentState = States.AT_BASKET_DEPOSIT;
            } else if (isShoulderTargetPosSpecimenIntakeAngle()) {
                currentState = States.AT_SPECIMEN_INTAKE;
            } else if (isShoulderTargetPosSampleIntakeAngle()) {
                currentState = States.AT_INTAKE;
            }
        } else {
            if (isShoulderTargetPosDepositChamberAngle()) {
                currentState = States.GOING_TO_CHAMBER;
            } else if (isShoulderTargetPosDepositBasketAngle()) {
                currentState = States.GOING_TO_BASKET;
            } else if (isShoulderTargetPosSpecimenIntakeAngle()) {
                currentState = States.GOING_TO_SPECIMEN_INTAKE;
            } else if (isShoulderTargetPosSampleIntakeAngle()) {
                currentState = States.GOING_TO_INTAKE;
            }

        }

    }

    private boolean isShoulderTargetPosSpecimenIntakeAngle() {
        return targetAngle == SPECIMEN_INTAKE_ANGLE;
    }

    public boolean isShoulderTargetPosDepositChamberAngle() {
        return targetAngle == chamberAngles[0] || targetAngle == chamberAngles[1];
    }

    public boolean isChamberAngleLow() {
        return targetAngle == chamberAngles[0];
    }

    public boolean isChamberAngleHigh() {
        return targetAngle == chamberAngles[1];
    }

    public void indexToLowChamberAngle() {
        chamberIndex = 0;
    }

    public void indexToHighChamberAngle() {
        chamberIndex = 1;
    }

    public void setChamberTargetAngle() {
        targetAngle = chamberAngles[chamberIndex];
    }

    //Basket methods
    public boolean isShoulderTargetPosDepositBasketAngle() {
        return targetAngle == basketAngles[0] || targetAngle == basketAngles[1];
    }

    public boolean isBasketAngleLow() {
        return targetAngle == basketAngles[0];
    }

    public boolean isBasketAngleHigh() {
        return targetAngle == basketAngles[1];
    }

    public void indexToLowChamber() {
        basketIndex = 0;
    }

    public void indexToHighChamber() {
        basketIndex = 1;
    }

    public void setBasketTargetAngle() {
        setExtendPIDF();
        targetAngle = basketAngles[basketIndex];
    }

    public void moveToIntakeAngle() {
        targetAngle = SAMPLE_INTAKE_ANGLE;
    }

    public boolean isShoulderTargetPosSampleIntakeAngle() {
        return targetAngle == SAMPLE_INTAKE_ANGLE;
    }


    public boolean GOING_TO_CHAMBER() {
        return currentState == States.GOING_TO_CHAMBER;
    }

    public boolean AT_DEPOSIT_CHAMBERS() {
        return currentState == States.AT_DEPOSIT_CHAMBERS;
    }

    public boolean GOING_TO_INTAKE() {
        return currentState == States.GOING_TO_INTAKE;
    }

    public boolean AT_INTAKE() {
        return currentState == States.AT_INTAKE;
    }

    public boolean GOING_TO_BASKET() {
        return currentState == States.GOING_TO_BASKET;
    }

    public boolean AT_BASKET_DEPOSIT() {
        return currentState == States.AT_BASKET_DEPOSIT;
    }

    public boolean AT_SPECIMEN_INTAKE() {
        return currentState == States.AT_SPECIMEN_INTAKE;
    }

    public boolean GOING_TO_SPECIMEN_INTAKE() {
        return currentState == States.GOING_TO_SPECIMEN_INTAKE;
    }

    public void updatePID() { // This method is used to update position every loop.
        shoulderWrapper.readAngle();

        if (shouldPID) {

            if (targetAngle >= 100)
                targetAngle = 100;

            if (lastPIDAngle != targetAngle) {
                pidfController.reset();
            }
            lastPIDAngle = targetAngle;

            measuredAngle = shoulderWrapper.readAngle();

            //This is the error between measured and target position.
            double delta = angleDelta(measuredAngle, targetAngle);
            double sign = angleDeltaSign(measuredAngle, targetAngle);
            // The error * sign (which is direction)
            double error = delta * sign;

            // We use zero because we already calculate for error
            power = pidfController.calculate(0, error);
            shoulderWrapper.set(power);
        }

    }


    public void moveToSpecimenIntakeAngle() {
        targetAngle = SPECIMEN_INTAKE_ANGLE;
    }


    protected double angleDelta(double measuredAngle, double targetAngle) {
        return Math.min(normalizeDegrees(measuredAngle - targetAngle), 360 - normalizeDegrees(measuredAngle - targetAngle));
    }

    protected double angleDeltaSign(double measuredAngle, double targetAngle) {
        return -(Math.signum(normalizeDegrees(targetAngle - measuredAngle) - (360 - normalizeDegrees(targetAngle - measuredAngle))));
    }

    protected static double normalizeDegrees(double angle) {
        return (angle + 360) % 360;
    }

    public double getTolerance() {
        return TOLERANCE;
    }

    public void setExtendPIDF() {
        pidfController.setPIDF(P_E, I_E, D_E, F_E);
    }

    public void setRetractPIDF() {
        pidfController.setPIDF(P_R, I_R, D_R, F_R);
    }

    public double getShoulderCurrentAngle() {
        return shoulderWrapper.getLastReadAngle();
    }

    public void resetEncoder() {
        targetAngle = -5;
        if (!timerStarted) {
            resetTimer.start();
            timerStarted = false;
        }

        pidfController.setTolerance(3);
        if (resetTimer.done()) {
            resetted = 1;
            shoulderWrapper.resetEncoder();
            timerStarted = false;
        }

        pidfController.setTolerance(TOLERANCE);
        targetAngle = 0;
    }

    public boolean isShoulderPosNegative() {
        return shoulderWrapper.getLastReadAngle() < -0.5;
    }

    public void restShoulder(boolean dPadDownPressed, boolean dPadDownReleased) {
        if (dPadDownPressed) {
            shouldPID = false;
            shoulderWrapper.set(-0.1);
        }
        if (dPadDownReleased) {
            shoulderWrapper.resetEncoder();
            shouldPID = true;
        }

    }

    public void log() {
        logger.log("-------------------------SHOULDER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder power", shoulderWrapper.get(), Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder State:", currentState, Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder Current Angle: ", shoulderWrapper.getLastReadAngle(), Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder target Angle: ", targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("AtSetPoint(): ", pidfController.atSetPoint(), Logger.LogLevels.PRODUCTION);
        logger.log("resetted: ", resetted, Logger.LogLevels.PRODUCTION);

        logger.log("-------------------------SHOULDER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);

    }


}
