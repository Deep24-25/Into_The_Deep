package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;

public class ShoulderFSM {

    public enum States {
        GOING_TO_CHAMBER, AT_DEPOSIT_CHAMBERS, GOING_TO_INTAKE, AT_INTAKE, GOING_TO_BASKET, AT_BASKET_DEPOSIT, GOING_TO_SPECIMEN_INTAKE, AT_SPECIMEN_INTAKE
    }


    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;



    private static final double ratio = 24/40;

    private static final double SAMPLE_INTAKE_ANGLE = 0;

    private static final double CHAMBER_ANGLE_LOW = 0;
    private static final double CHAMBER_ANGLE_HIGH = 0;
    private static final double BASKET_ANGLE = 0;
    private static final double SPECIMEN_INTAKE_ANGLE = 0;

    private final ShoulderWrapper shoulderWrapper;
    private final PIDFController pidfController;
    private static double targetAngle;
    private double measuredAngle;
    private States currentState = States.GOING_TO_CHAMBER;

    private double lastPIDAngle = 0;
    private double power;
    private double TOLERANCE = 3;

    public ShoulderFSM(HWMap hwMap) {
        this.pidfController = new PIDFController(P, I, D,F);
       shoulderWrapper = new ShoulderWrapper(hwMap);
    }

    @VisibleForTesting
    public ShoulderFSM(ShoulderWrapper shoulderWrapper, PIDFController pidfController) {
        this.shoulderWrapper = shoulderWrapper;
        this.pidfController = pidfController;

    }

    public void updateState() {
        pidfController.setPIDF(P,I,D,F);
        pidfController.setSetPoint(0);
        pidfController.setTolerance(TOLERANCE);
        updatePID();
        if(isShoulderTargetPosDepositChamberAngle()) {
            if (pidfController.atSetPoint()){
                currentState = States.AT_DEPOSIT_CHAMBERS;
            }
            else
                currentState = States.GOING_TO_CHAMBER;
        }
        else if (isShoulderTargetPosSampleIntakeAngle()) {
            if (pidfController.atSetPoint()){
                currentState = States.AT_INTAKE;
            }
            else
                currentState = States.GOING_TO_INTAKE;
        }
        else if (isShoulderTargetPosDepositBasketAngle()){
            if (pidfController.atSetPoint()){
                currentState = States.AT_BASKET_DEPOSIT;
            }
            else
                currentState = States.GOING_TO_BASKET;
        }
        else if(isShoulderCurrentPosSpecimenIntakeAngle()) {
            if(pidfController.atSetPoint()) {
                currentState = States.AT_SPECIMEN_INTAKE;
            }
            else {
                currentState = States.GOING_TO_SPECIMEN_INTAKE;
            }
        }
    }

    private boolean isShoulderCurrentPosSpecimenIntakeAngle() {
        return targetAngle == SPECIMEN_INTAKE_ANGLE;
    }

    public boolean isShoulderTargetPosDepositChamberAngle() {
        return targetAngle == CHAMBER_ANGLE_HIGH || targetAngle == CHAMBER_ANGLE_LOW;
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

    }

    public void moveToIntakeAngle() {
        targetAngle = SAMPLE_INTAKE_ANGLE;
    }

    public void moveToBasketAngle() {
        targetAngle = BASKET_ANGLE;
    }

    public void moveToLowChamberAngle() {
        targetAngle = CHAMBER_ANGLE_LOW;
    }

    public void moveToHighChamberAngle() {
        targetAngle = CHAMBER_ANGLE_HIGH;
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

}
