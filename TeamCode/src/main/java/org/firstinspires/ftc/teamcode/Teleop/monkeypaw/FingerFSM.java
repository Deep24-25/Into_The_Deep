package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;


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


    public static double SAMPLE_GRIPPED_POS = 1; //0.2
    public static double SPECIMEN_GRIPPED_POS = 0.01; //0.19
    public static double SAMPLE_RELEASED_POS = 0.05; //0.67
    public static double SPECIMEN_RELEASED_POS = 0.23; //0.67

    private final FingerServoWrapper fingerServoWrapper;

    private FingerStates state;
    private final Logger logger;
    private final Timing.Timer timer;

    public static long OFFSET = 500;


    public FingerFSM(HWMap hwMap, Logger logger) {
        fingerServoWrapper = new FingerServoWrapper(hwMap);
        this.logger = logger;
        timer = new Timing.Timer(OFFSET, TimeUnit.MILLISECONDS);
        state = FingerStates.RELEASING;
    }

    public void updateState() {
        fingerServoWrapper.readAngle();
        fingerServoWrapper.setAngle(targetAngle);
        if (isTargetAngleToGrip()) {
            if (!(state == FingerStates.GRIPPED)) {
                state = FingerStates.GRIPPING;
            }
            if (!timer.isTimerOn()) {
                timer.start();
            }
            if (timer.done()) {
                timer.pause();
                state = FingerStates.GRIPPED;
            }

        } else if (isTargetAngleToRelease()) {
            if (!(state == FingerStates.RELEASED)) {
                state = FingerStates.RELEASING;
            }
            if (!timer.isTimerOn()) {
                timer.start();
            }
            if (timer.done()) {
                timer.pause();
                state = FingerStates.RELEASED;
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

    public boolean GRIPPED() {
        return state == FingerStates.GRIPPED;
    }

    public boolean RELEASED() {
        return state == FingerStates.RELEASED;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void log() {
        logger.log("------------------------- FINGER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Finger State", state, Logger.LogLevels.PRODUCTION);
        logger.log("Finger Target Pos", targetAngle, Logger.LogLevels.DEBUG);
        logger.log("Specimen timer:", timer.elapsedTime(), Logger.LogLevels.DEBUG);
        logger.log("Sample timer:", timer.elapsedTime(), Logger.LogLevels.DEBUG);

        logger.log("------------------------- FINGER LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);


    }

}
