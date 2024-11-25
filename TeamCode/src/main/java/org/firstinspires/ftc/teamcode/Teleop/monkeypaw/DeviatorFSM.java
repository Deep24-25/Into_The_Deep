package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

@Config
public class DeviatorFSM {
    private enum DeviatorStates {
        RIGHT_DEVIATED,
        LEFT_DEVIATED,
        RELAXED,
        RIGHT_DEVIATING,
        LEFT_DEVIATING,
        RELAXING
    }

    private double targetAngle;


    public static  double RIGHT_DEVIATED_POS = 45;
    public static  double LEFT_DEVIATED_POS = 315;
    public static  double RELAXED_POS = 0;


    private DeviatorServoWrapper deviatorServoWrapper;

    private DeviatorStates state;
    private Logger logger;

    public DeviatorFSM(HWMap hwMap, Logger logger) {
        deviatorServoWrapper = new DeviatorServoWrapper(hwMap);
        this.logger = logger;

    }
    @VisibleForTesting
    public DeviatorFSM(DeviatorServoWrapper deviatorServoWrapper, Logger logger) {
        this.deviatorServoWrapper = deviatorServoWrapper;
        this.logger = logger;
    }

    public void updateState() {
        deviatorServoWrapper.readAngle();
        deviatorServoWrapper.setAngle(targetAngle);
        if (isTargetAngleToDeviateRight()) {
            if (isCurrentAngleDeviatedRight()) {
                state = DeviatorStates.RIGHT_DEVIATED;
            } else {
                state = DeviatorStates.RIGHT_DEVIATING;
            }
        }
        else if (isTargetAngleToDeviateLeft()) {
            if (isCurrentAngleDeviatedLeft()) {
                state = DeviatorStates.LEFT_DEVIATED;
            } else {
                state = DeviatorStates.LEFT_DEVIATING;
            }
        }
        else if (isTargetAngleToRelax()) {
            if (isCurrentAngleRelaxed()) {
                state = DeviatorStates.RELAXED;
            } else {
                state = DeviatorStates.RELAXING;
            }
        }
    }

    private boolean isCurrentAngleDeviatedRight() {
        return deviatorServoWrapper.getLastReadAngle() == RIGHT_DEVIATED_POS;
    }

    private boolean isCurrentAngleDeviatedLeft() {
        return deviatorServoWrapper.getLastReadAngle() == LEFT_DEVIATED_POS;
    }

    private boolean isCurrentAngleRelaxed() {
        return deviatorServoWrapper.getLastReadAngle() == RELAXED_POS;
    }

    public boolean isTargetAngleToRelax() {
        return targetAngle == RELAXED_POS;
    }

    public boolean isTargetAngleToDeviateRight() {
        return targetAngle == RIGHT_DEVIATED_POS;
    }

    public boolean isTargetAngleToDeviateLeft() {
        return targetAngle == LEFT_DEVIATED_POS;
    }



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
    }



    public void log() {
        logger.log("Deviator State",state, Logger.LogLevels.PRODUCTION);
        logger.log("Deviator Current Position",deviatorServoWrapper.getLastReadAngle(), Logger.LogLevels.PRODUCTION);
        logger.log("Deviator Target Pos",targetAngle, Logger.LogLevels.PRODUCTION);


    }
}

