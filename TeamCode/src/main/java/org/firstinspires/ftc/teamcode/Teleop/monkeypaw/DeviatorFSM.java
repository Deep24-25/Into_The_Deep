package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonCRServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;

@Config
public class DeviatorFSM {
    private enum DeviatorStates {
        RIGHT_DEVIATED,
        LEFT_DEVIATED,
        RELAXED,
        RIGHT_DEVIATING,
        LEFT_DEVIATING,
        RELAXING,
        VERTICALING,
        VERTICALED
    }

    private double targetAngle;
    public static double TOLERANCE = 5;
    private double deviatorCurrentAngle;
    //Robot CONSTANTS:
    public static double P = 0.005;
    public static double I = 0;
    public static double D = 0;


    public static double RIGHT_DEVIATED_POS = 47.5;
    public static double LEFT_DEVIATED_POS = 150;
    public static double RELAXED_POS = 195;
    public static double VERTICAL_POS = 92.5;


    private AxonServoWrapper deviatorServoWrapper;
    /* private PIDController pidController;
     */
    private DeviatorStates state;
    private Logger logger;

    private int currentIndex = 0;

    private final double[] deviations = {RELAXED_POS, 172.5, LEFT_DEVIATED_POS, 127.5, VERTICAL_POS, 70, RIGHT_DEVIATED_POS, 25, 0};

    public DeviatorFSM(HWMap hwMap, Logger logger) {
        deviatorServoWrapper = new AxonServoWrapper(hwMap.getWristDeviServo(), hwMap.getWristDeviEncoder(), false, false, 0); // check if you need to reverse axons
        /*pidController = new PIDController(P, I, D);
         */
        this.logger = logger;
        deviatorCurrentAngle = deviatorServoWrapper.getLastReadPos();
        relax(); // Need this so target angle is set to the relax position setting state would do nothing as that itself does not change target angle
        state = DeviatorStates.RELAXING;

    }

    @VisibleForTesting
    public DeviatorFSM(AxonServoWrapper axonServoWrapper, Logger logger, PIDController pidController) {
        this.deviatorServoWrapper = axonServoWrapper;
        /*this.pidController = pidController;
         */
        this.logger = logger;
    }

    public void updateState() {
        /*pidController.setPID(P, I, D);
        pidController.setSetPoint(0); // PIDs the error to 0
        pidController.setTolerance(PID_TOLERANCE); // sets the buffer
        updatePID();

        */

        updatePos();
        if (isTargetAngleToDeviateRight()) {
            if (atSetPoint()) {
                state = DeviatorStates.RIGHT_DEVIATED;
            } else {
                state = DeviatorStates.RIGHT_DEVIATING;
            }
        } else if (isTargetAngleToDeviateLeft()) {
            if (atSetPoint()) {
                state = DeviatorStates.LEFT_DEVIATED;
            } else {
                state = DeviatorStates.LEFT_DEVIATING;
            }
        } else if (isTargetAngleToRelax()) {
            if (atSetPoint()) {
                state = DeviatorStates.RELAXED;
            } else {
                state = DeviatorStates.RELAXING;
            }
        } else if (isTargetAngleToVertical()) {
            if (atSetPoint()) {
                state = DeviatorStates.VERTICALED;
            } else {
                state = DeviatorStates.VERTICALING;
            }
        }
    }

    public void indexIncrement() {
        int tempIndex = currentIndex + 1;
        if (tempIndex < deviations.length - 1) {
            currentIndex++;
        } else {
            relax();
        }
    }

    public void indexDecrement() {
        int tempIndex = currentIndex - 1;
        if (tempIndex >= 0) {
            currentIndex--;
        }
    }


    public boolean isTargetAngleToRelax() {
        return targetAngle == RELAXED_POS;
    }

    public boolean isTargetAngleToVertical() {
        return targetAngle == VERTICAL_POS;
    }

    public boolean isTargetAngleToDeviateRight() {
        return targetAngle == RIGHT_DEVIATED_POS;
    }

    public boolean isTargetAngleToDeviateLeft() {
        return targetAngle == LEFT_DEVIATED_POS;
    }

    public boolean atSetPoint() {
        return (deviatorServoWrapper.getLastReadPos() <= targetAngle + TOLERANCE) || (deviatorServoWrapper.getLastReadPos() >= targetAngle - TOLERANCE);
    }

    public void updatePos() {
        deviatorServoWrapper.readPos();
        deviatorServoWrapper.set(deviations[currentIndex]);
        logger.log("Current angle", deviatorServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
    }


/*
    public void updatePID() { // This method is used to update position every loop.
        deviatorServoWrapper.readPos();
        double angleDelta = angleDelta(deviatorServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(deviatorServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("Deviator Power",power, Logger.LogLevels.DEBUG);
        deviatorServoWrapper.set(power);
    }*/

    // Finds the smallest distance between 2 angles, input and output in degrees
    private double angleDelta(double angle1, double angle2) {
        return Math.min(normalizeDegrees(angle1 - angle2), 360 - normalizeDegrees(angle1 - angle2));
    }

    // Finds the direction of the smallest distance between 2 angles
    private double angleDeltaSign(double position, double target) {
        return -(Math.signum(normalizeDegrees(target - position) - (360 - normalizeDegrees(target - position))));
    }

    // Takes input angle in degrees, returns that angle in the range of 0-360
    //Prevents the servos from looping around
    private static double normalizeDegrees(double angle) {
        return (angle + 360) % 360;
    }

    public boolean RIGHT_DEVIATED() {
        return state == DeviatorStates.RIGHT_DEVIATED;
    }

    public boolean LEFT_DEVIATING() {
        return state == DeviatorStates.LEFT_DEVIATING;
    }


    public boolean LEFT_DEVIATED() {
        return state == DeviatorStates.LEFT_DEVIATED;
    }

    public boolean RELAXED() {
        return state == DeviatorStates.RELAXED;
    }

    public boolean RELAXING() {
        return state == DeviatorStates.RELAXING;
    }

    public boolean RIGHT_DEVIATING() {
        return state == DeviatorStates.RIGHT_DEVIATING;
    }

    public void deviateRight() {
        targetAngle = RIGHT_DEVIATED_POS;
    }

    public void deviateLeft() {
        targetAngle = LEFT_DEVIATED_POS;
    }

    public void relax() {
        targetAngle = RELAXED_POS;
        currentIndex = 0;
    }

    public void vertical() {
        targetAngle = VERTICAL_POS;
        currentIndex = 4;
    }

    public boolean indexCloserToRelaxation() {
        double angle = deviatorServoWrapper.getLastReadPos();
        if(deviatorServoWrapper.getLastReadPos() < VERTICAL_POS) {
            angle = 360 - deviatorServoWrapper.getLastReadPos();
        }

        return Math.abs(angle - RELAXED_POS) <= 45;
    }


    public void log() {
        logger.log("Deviator State", state, Logger.LogLevels.PRODUCTION);
        logger.log("Deviator Current Position", deviatorServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
        //logger.log("At Max Index", atMaxIndex(), Logger.LogLevels.PRODUCTION);
        logger.log("Deviator Target Pos", targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("Current index", currentIndex, Logger.LogLevels.PRODUCTION);
        logger.log("At Target Pos", deviatorServoWrapper.getLastReadPos() == targetAngle, Logger.LogLevels.PRODUCTION);


    }
}

