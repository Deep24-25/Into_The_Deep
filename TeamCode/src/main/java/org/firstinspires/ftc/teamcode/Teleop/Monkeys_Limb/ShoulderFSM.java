package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;

public class ShoulderFSM {

    public enum States {
        GOING_TO_CHAMBER, AT_DEPOSIT_CHAMBERS, GOING_TO_INTAKE, AT_INTAKE, GOING_TO_BASKET, AT_BASKET_DEPOSIT, GOING_TO_SPECIMEN_INTAKE, AT_SPECIMEN_INTAKE
    }


    public static double P = 0.01;
    public static double I = 0.001;
    public static double D = 0.002;
    public static double F = 0;


    private static final double ratio = 24.0 / 40.0;

    private static final double SAMPLE_INTAKE_ANGLE = 0;

    private static final double CHAMBER_ANGLE_LOW = 45;
    private static final double CHAMBER_ANGLE_HIGH = 90;
    private static final double BASKET_ANGLE = 100;
    private static final double SPECIMEN_INTAKE_ANGLE = 120;

    private final ShoulderWrapper shoulderWrapper;
    private final PIDFController pidfController;
    private double targetAngle = SAMPLE_INTAKE_ANGLE;
    private double[] chamberAngles = {CHAMBER_ANGLE_LOW, CHAMBER_ANGLE_HIGH};
    private int chamberIndex = 1;
    private double measuredAngle;
    private States currentState;

    private double lastPIDAngle = 0;
    private double power;
    private static final double TOLERANCE = 7.5;

    private Logger logger;

    public ShoulderFSM(HWMap hwMap, Logger logger) {
        this.pidfController = new PIDFController(P, I, D, F);
        shoulderWrapper = new ShoulderWrapper(hwMap);
        this.logger = logger;
    }

    @VisibleForTesting
    public ShoulderFSM(ShoulderWrapper shoulderWrapper, PIDFController pidfController) {
        this.shoulderWrapper = shoulderWrapper;
        this.pidfController = pidfController;

    }

    public void updateState() {
        pidfController.setPIDF(P, I, D, F);
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
        return targetAngle == CHAMBER_ANGLE_HIGH | targetAngle == CHAMBER_ANGLE_LOW;
    }

    public boolean isShoulderTargetPosSampleIntakeAngle() {
        return targetAngle == SAMPLE_INTAKE_ANGLE;
    }

    public boolean isShoulderTargetPosDepositBasketAngle() {
        return targetAngle == BASKET_ANGLE;
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

    public void moveToIntakeAngle() {
        targetAngle = SAMPLE_INTAKE_ANGLE;
    }

    public void moveToBasketAngle() {
        targetAngle = BASKET_ANGLE;
    }

    public void indexToLowChamberAngle() {
        chamberIndex = 0;
    }

    public void indexToHighChamberAngle() {
        chamberIndex = 1;
    }

    public void moveToSpecimenIntakeAngle() {
        targetAngle = SPECIMEN_INTAKE_ANGLE;
    }

    public void moveToChamberAngle() {
        targetAngle = chamberAngles[chamberIndex];
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

    public void log() {
        logger.log("-------------------------SHOULDER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder State:", currentState, Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder Current Angle: ", shoulderWrapper.getLastReadAngle(), Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder target Angle: ", targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("AtSetPoint(): ", pidfController.atSetPoint(), Logger.LogLevels.PRODUCTION);

        logger.log("-------------------------SHOULDER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);

    }

}
