package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

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


    public static  double SAMPLE_GRIPPED_POS = -140;
    public static double SPECIMEN_GRIPPED_POS = 90;
    public static  double SAMPLE_RELEASED_POS = 140;
    public static double SPECIMEN_RELEASED_POS = -145;

    private FingerServoWrapper fingerServoWrapper;

    private FingerStates state;
    private Logger logger;
    private Timing.Timer timer;

    public FingerFSM(HWMap hwMap, Logger logger) {
        fingerServoWrapper = new FingerServoWrapper(hwMap);
        this.logger = logger;
        fingerCurrentAngle = fingerServoWrapper.readAngle();
        timer =  new Timing.Timer(1000, TimeUnit.MILLISECONDS);
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
            if(!(state == FingerStates.GRIPPED)) {
                state = FingerStates.GRIPPING;
            }
            if(!timer.isTimerOn()) {
                timer.start();
            }
            if(timer.done()) {
                timer.pause();
                state = FingerStates.GRIPPED;
            }

        } else if (isTargetAngleToRelease()) {
            if(!timer.isTimerOn()) {
                timer.start();
            }
            if(timer.done()) {
                timer.pause();
                state = FingerStates.RELEASED;
            }
            if(!(state == FingerStates.RELEASED)) {
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

   // public void updatePID() { // This method is used to update position every loop.
       /* fingerServoWrapper.readAngle();
        double angleDelta = angleDelta(fingerServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(fingerServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("Finger Power",power, Logger.LogLevels.PRODUCTION);
        fingerServoWrapper.set(power);*/

    //}



    public void gripSample() {
        targetAngle = SAMPLE_GRIPPED_POS;
        fingerServoWrapper.setAngle(targetAngle);
    }
    public void gripSpecimen() {
        targetAngle = SPECIMEN_GRIPPED_POS;
        fingerServoWrapper.setAngle(targetAngle);
    }

    public void releaseSample() {
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
        logger.log("Finger State",state, Logger.LogLevels.PRODUCTION);
        logger.log("Finger Current Position",fingerServoWrapper.getPos(), Logger.LogLevels.PRODUCTION);
        logger.log("Finger Current Angle",fingerServoWrapper.readAngle(), Logger.LogLevels.PRODUCTION);
        logger.log("Finger Target Pos",targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("At Target Pos", fingerServoWrapper.readAngle() == targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("Finger Servo Timer", timer.elapsedTime(), Logger.LogLevels.PRODUCTION);

    }

}
