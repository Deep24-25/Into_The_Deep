package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
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
        VERTICALED,
        CHAMBER_DEPOSITTING,
        CHAMBER_DEPOSITED
    }

    private double targetAngle;
    public static double TOLERANCE = 10;
    //Robot CONSTANTS:
    public static double P = 0.005;
    public static double I = 0;
    public static double D = 0;


    public static double RIGHT_DEVIATED_POS = 42.5;
    public static double LEFT_DEVIATED_POS = 145;
    public static double RELAXED_POS = 190;
    public static double VERTICAL_POS = 87.5;
    public static double CHAMBER_DEPOSIT_POS = 5;

    private final AxonServoWrapper deviatorServoWrapper;

    private DeviatorStates state;
    private final Logger logger;

    private int currentIndex = 0;

    private final double[] deviations = {RELAXED_POS, 167.5, LEFT_DEVIATED_POS, 122.5, VERTICAL_POS, 65, RIGHT_DEVIATED_POS, 20, 0};

    public DeviatorFSM(HWMap hwMap, Logger logger) {
        deviatorServoWrapper = new AxonServoWrapper(hwMap.getWristDeviServo(), hwMap.getWristDeviEncoder(), false, false, 0); // check if you need to reverse axons
        this.logger = logger;
        relax(); // Need this so target angle is set to the relax position setting state would do nothing as that itself does not change target angle
        state = DeviatorStates.RELAXING;

    }

    public void updateState() {
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
        } else if (isTargetAngleChamberDepositPos()) {
            if (atSetPoint()) {
                state = DeviatorStates.CHAMBER_DEPOSITED;
            } else {
                state = DeviatorStates.CHAMBER_DEPOSITTING;
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


    public boolean isTargetAngleToRelax() {
        return targetAngle == (RELAXED_POS);
    }

    public boolean isTargetAngleToVertical() {
        return targetAngle == (VERTICAL_POS);
    }

    public boolean isTargetAngleToDeviateRight() {
        return targetAngle == (RIGHT_DEVIATED_POS);
    }

    public boolean isTargetAngleToDeviateLeft() {
        return targetAngle == (LEFT_DEVIATED_POS);
    }

    public boolean isTargetAngleChamberDepositPos() {
        return targetAngle == (CHAMBER_DEPOSIT_POS);
    }

    public void goToChamberDepositPos() {
        targetAngle = (CHAMBER_DEPOSIT_POS);
    }

    public boolean atSetPoint() {
        return (deviatorServoWrapper.getLastReadPos() <= targetAngle + TOLERANCE) && (deviatorServoWrapper.getLastReadPos() >= targetAngle - TOLERANCE);
    }

    public void updatePos() {
        deviatorServoWrapper.readPos();
        if (!isTargetAngleChamberDepositPos())
            deviatorServoWrapper.set(deviations[currentIndex]);
        else
            deviatorServoWrapper.set(targetAngle);
        logger.log("Current angle", deviatorServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
    }
    // Takes input angle in degrees, returns that angle in the range of 0-360
    //Prevents the servos from looping around

    public boolean RELAXED() {
        return state == DeviatorStates.RELAXED;
    }

    public boolean RELAXING() {
        return state == DeviatorStates.RELAXING;
    }

    public boolean CHAMBER_DEPOSITTED() {
        return state == DeviatorStates.CHAMBER_DEPOSITED;
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
        if (deviatorServoWrapper.getLastReadPos() < (VERTICAL_POS)) {
            angle = 360 - deviatorServoWrapper.getLastReadPos();
        }

        return Math.abs(angle - (RELAXED_POS)) <= 45;
    }


    public void log() {
        logger.log("------------------ DEVIATOR LOG--------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Deviator State", state, Logger.LogLevels.PRODUCTION);
        logger.log("Deviator Current Position", deviatorServoWrapper.getLastReadPos(), Logger.LogLevels.DEBUG);
        logger.log("Deviator Target Pos", targetAngle, Logger.LogLevels.DEBUG);
        logger.log("Current index", currentIndex, Logger.LogLevels.DEBUG);
        logger.log("At Target Pos", deviatorServoWrapper.getLastReadPos() == targetAngle, Logger.LogLevels.DEBUG);
        logger.log("------------------ DEVIATOR LOG--------------------", "-", Logger.LogLevels.PRODUCTION);


    }
}

