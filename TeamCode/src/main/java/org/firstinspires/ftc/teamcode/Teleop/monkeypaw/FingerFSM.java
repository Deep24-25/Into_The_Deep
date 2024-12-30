package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.FingerServoWrapper;

import java.util.concurrent.TimeUnit;

@Config
public class FingerFSM {
    private enum FingerStates {
        GRIPPING,
        GRIPPED,
        RELEASING,
        RELEASED
    }

    private double targetAngle;
    private double fingerCurrentAngle;


    public static double SAMPLE_GRIPPED_POS = 300;
    public static double SPECIMEN_GRIPPED_POS = 5;
    public static double SAMPLE_RELEASED_POS = 0;
    public static double SPECIMEN_RELEASED_POS = 280;

    private FingerServoWrapper fingerServoWrapper;

    private FingerStates state;
    private Logger logger;
    private Timing.Timer specimenGrippingTimer;
    private Timing.Timer otherTimer;

    public static long SPECIMEN_TIME_OFFSET = 2500;

    public static long SAMPLE_TIME_OFFSET = 500;

    public FingerFSM(HWMap hwMap, Logger logger) {
        fingerServoWrapper = new FingerServoWrapper(hwMap);
        this.logger = logger;
        fingerCurrentAngle = fingerServoWrapper.readAngle();
        otherTimer = new Timing.Timer(SAMPLE_TIME_OFFSET, TimeUnit.MILLISECONDS);
        specimenGrippingTimer = new Timing.Timer(SPECIMEN_TIME_OFFSET, TimeUnit.MILLISECONDS);
        state = FingerStates.RELEASING;
    }

    @VisibleForTesting
    public FingerFSM(FingerServoWrapper fingerServoWrapper, Logger logger) {
        this.fingerServoWrapper = fingerServoWrapper;
        this.logger = logger;
        fingerCurrentAngle = fingerServoWrapper.readAngle();
    }

    public void updateState() {
        fingerServoWrapper.readAngle();
        fingerServoWrapper.setAngle(targetAngle);
        if (isTargetAngleToGrip()) {
            if (isTargetAngleSpecimenGrip()) {
                if (!(state == FingerStates.GRIPPED)) {
                    state = FingerStates.GRIPPING;
                }
                if (!specimenGrippingTimer.isTimerOn()) {
                    specimenGrippingTimer.start();
                }
                if (specimenGrippingTimer.done()) {
                    specimenGrippingTimer.pause();
                    state = FingerStates.GRIPPED;
                }
            } else {
                if (!(state == FingerStates.GRIPPED)) {
                    state = FingerStates.GRIPPING;
                }
                if (!otherTimer.isTimerOn()) {
                    otherTimer.start();
                }
                if (otherTimer.done()) {
                    otherTimer.pause();
                    state = FingerStates.GRIPPED;
                }
            }
        } else if (isTargetAngleToRelease()) {
            if (!otherTimer.isTimerOn()) {
                otherTimer.start();
            }
            if (otherTimer.done()) {
                otherTimer.pause();
                state = FingerStates.RELEASED;
            }
            if (!(state == FingerStates.RELEASED)) {
                state = FingerStates.RELEASING;
            }

        }
    }

    public boolean isTargetAngleToRelease() {
        return targetAngle == SAMPLE_RELEASED_POS || targetAngle == SPECIMEN_RELEASED_POS;
    }

    public boolean isTargetAngleToGrip() {
        return targetAngle == SAMPLE_GRIPPED_POS || targetAngle == SPECIMEN_GRIPPED_POS;
    }

    public boolean isTargetAngleSpecimenGrip() {
        return targetAngle == SPECIMEN_GRIPPED_POS;
    }
    public boolean isTargetAngleSampleRelease() {
        return targetAngle == SAMPLE_RELEASED_POS;
    }

    public void gripSample() {
        targetAngle = SAMPLE_GRIPPED_POS;
        fingerServoWrapper.setAngle(targetAngle);
    }

    public void gripSpecimen() {
        targetAngle = SPECIMEN_GRIPPED_POS;
        fingerServoWrapper.setAngle(targetAngle);
    }

    public void releaseSample() {
        fingerServoWrapper.setAngle(90);
        targetAngle = SAMPLE_RELEASED_POS;
        fingerServoWrapper.setAngle(targetAngle);
    }

    public void releaseSpecimen() {
        targetAngle = SPECIMEN_RELEASED_POS;
        fingerServoWrapper.setAngle(targetAngle);
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

    public boolean GRIPPED() {
        return state == FingerStates.GRIPPED;
    }

    public boolean GRIPPING() {
        return state == FingerStates.GRIPPING;
    }

    public boolean RELEASED() {
        return state == FingerStates.RELEASED;
    }

    public boolean RELEASING() {
        return state == FingerStates.RELEASING;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getFingerCurrentAngle() {
        return fingerCurrentAngle;
    }

    public double getGrippedPos() {
        return SAMPLE_GRIPPED_POS;
    }

    public double getReleasedPos() {
        return SAMPLE_RELEASED_POS;
    }


    public void log() {
        logger.log("------------------------- FINGER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Finger State", state, Logger.LogLevels.PRODUCTION);
        logger.log("Finger Target Pos", targetAngle, Logger.LogLevels.DEBUG);
        logger.log("Specimen timer:", specimenGrippingTimer.elapsedTime(), Logger.LogLevels.DEBUG);
        logger.log("Sample timer:", otherTimer.elapsedTime(), Logger.LogLevels.DEBUG);

        logger.log("------------------------- FINGER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);


    }

}
