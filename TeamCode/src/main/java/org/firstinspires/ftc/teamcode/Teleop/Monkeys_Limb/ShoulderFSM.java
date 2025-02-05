package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;


@Config
public class ShoulderFSM {

    public enum States {
        GOING_TO_CHAMBER, AT_DEPOSIT_CHAMBERS, GOING_TO_INTAKE, AT_INTAKE, GOING_TO_BASKET, AT_BASKET_DEPOSIT, GOING_TO_SPECIMEN_INTAKE, AT_SPECIMEN_INTAKE
    }


    public static double P_E = 0.07;
    public static double I_E = 0.04;
    public static double D_E = 0.07;
    public static double F_E = 0.02;

    private static final double SAMPLE_INTAKE_ANGLE = 0;

    private static final double CHAMBER_ANGLE = 100; // 100
    private static final double BASKET_ANGLE = 100;

    private static final double SPECIMEN_INTAKE_ANGLE = 0;

    private final ShoulderWrapper shoulderWrapper;
    private final PIDFController pidfController;
    private double targetAngle = SAMPLE_INTAKE_ANGLE;

    private States currentState;

    private double lastPIDAngle = 0;

    private Logger logger;

    private boolean shouldPID = true;
    private LimbFSM limbFSM;

    private int counter = 0;
    private double TOLERANCE = 7.5;

    public ShoulderFSM(HWMap hwMap, Logger logger, LimbFSM limbFSM, boolean reset) {
        this.pidfController = new PIDFController(P_E, I_E, D_E, F_E);
        shoulderWrapper = new ShoulderWrapper(hwMap, reset);
        this.logger = logger;
        this.limbFSM = limbFSM;
    }

    public ShoulderFSM(HWMap hwMap, Logger logger, boolean reset) {
        this.pidfController = new PIDFController(P_E, I_E, D_E, F_E);
        shoulderWrapper = new ShoulderWrapper(hwMap, reset);
        this.logger = logger;
    }

    @VisibleForTesting
    public ShoulderFSM(ShoulderWrapper shoulderWrapper, PIDFController pidfController) {
        this.shoulderWrapper = shoulderWrapper;
        this.pidfController = pidfController;
    }

    public void updateState(boolean isAuto) {
        pidfController.setTolerance(TOLERANCE);
        updatePID();
        if (isShoulderTargetPosDepositChamberAngle() && isAuto && limbFSM.SPECIMEN_MODE()) {
            TOLERANCE = 2.5;
        } else {
            TOLERANCE = 7.5;
        }

        if (pidfController.atSetPoint()) {
            if (isShoulderTargetPosDepositChamberAngle() && limbFSM.SPECIMEN_MODE() && isAuto) {
                counter++;
                if (counter == 50) {
                    currentState = States.AT_DEPOSIT_CHAMBERS;
                    counter = 0;
                }
            }
            if (!(isShoulderTargetPosDepositChamberAngle() && limbFSM.SPECIMEN_MODE() && isAuto)) {
                counter = 0;
            }
            if (isShoulderTargetPosDepositChamberAngle() && limbFSM.SPECIMEN_MODE()) {
                currentState = States.AT_DEPOSIT_CHAMBERS;
            } else if (isShoulderTargetPosDepositBasketAngle() && limbFSM.SAMPLE_MODE()) {
                currentState = States.AT_BASKET_DEPOSIT;
            } else if (isShoulderTargetPosSpecimenIntakeAngle() && limbFSM.SPECIMEN_MODE()) {
                currentState = States.AT_SPECIMEN_INTAKE;
            } else if (isShoulderTargetPosSampleIntakeAngle() && limbFSM.SAMPLE_MODE()) {
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
        return targetAngle == CHAMBER_ANGLE;
    }

    public void setChamberTargetAngle() {
        setExtendPIDF();
        targetAngle = CHAMBER_ANGLE;
    }

    //Basket methods
    public boolean isShoulderTargetPosDepositBasketAngle() {
        return targetAngle == BASKET_ANGLE;
    }

    public void setBasketTargetAngle() {
        setExtendPIDF();
        targetAngle = BASKET_ANGLE;
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


    public void updatePID() { // This method is used to update position every loop.
        shoulderWrapper.readAngle();

        if (shouldPID) {

            if (targetAngle >= 100)
                targetAngle = 100;

            if (lastPIDAngle != targetAngle) {
                pidfController.reset();
            }
            lastPIDAngle = targetAngle;

            double measuredAngle = shoulderWrapper.readAngle();

            //This is the error between measured and target position.
            double delta = angleDelta(measuredAngle, targetAngle);
            double sign = angleDeltaSign(measuredAngle, targetAngle);
            // The error * sign (which is direction)
            double error = delta * sign;

            // We use zero because we already calculate for error
            double power = pidfController.calculate(0, error);
            shoulderWrapper.set(power);
        }

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

    public void setExtendPIDF() {
        pidfController.setPIDF(P_E, I_E, D_E, F_E);
    }


    public void resetShoulder(boolean dPadDownPressed, boolean dPadDownReleased) {
        if (dPadDownPressed) {
            shouldPID = false;
            shoulderWrapper.set(-0.1);
        }
        if (dPadDownReleased) {
            shoulderWrapper.resetEncoder();
            shouldPID = true;
            targetAngle = SAMPLE_INTAKE_ANGLE;
        }

    }

    public void log() {
        logger.log("---------------------SHOULDER LOG----------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder State:", currentState, Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder Current Angle: ", shoulderWrapper.getLastReadAngle(), Logger.LogLevels.PRODUCTION);
        logger.log("Shoulder target Angle: ", targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("AtSetPoint(): ", pidfController.atSetPoint(), Logger.LogLevels.DEBUG);
        logger.log("Shoulder power", shoulderWrapper.get(), Logger.LogLevels.DEBUG);
        logger.log("Counter", counter, Logger.LogLevels.PRODUCTION);
        logger.log("---------------------SHOULDER LOG----------------------", "-", Logger.LogLevels.PRODUCTION);

    }

    public void setLimbFSM(LimbFSM limbFSM) {
        this.limbFSM = limbFSM;
    }

}
