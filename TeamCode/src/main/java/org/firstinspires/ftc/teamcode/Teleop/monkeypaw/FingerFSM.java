package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
@Config
public class FingerFSM {
    private enum FingerStates {
        GRIPPING,
        GRIPPED,
        RELEASING,
        RELEASED
    }

    private double targetAngle;
    public static  double PID_TOLERANCE = 5;
    private double fingerCurrentAngle;
    //Robot CONSTANTS:
    public static  double P = 0.01;
    public static  double I = 0;
    public static  double D = 0;


    public static  double GRIPPED_POS = 100;
    public static  double RELEASED_POS = 50;

    private AxonServoWrapper fingerServoWrapper;
    private PIDController pidController;

    private FingerStates state;
    private Logger logger;

    public FingerFSM(HWMap hwMap, Logger logger) {
        fingerServoWrapper = new AxonServoWrapper(hwMap.getFingerServo(),hwMap.getFingerEncoder(),false, false); // check if you need to reverse axons
        pidController = new PIDController(P, I, D);
        this.logger = logger;
        fingerCurrentAngle = fingerServoWrapper.getLastReadPos();

    }
    @VisibleForTesting
    public FingerFSM(AxonServoWrapper axonServoWrapper, Logger logger, PIDController pidController) {
        fingerServoWrapper = axonServoWrapper;
        this.pidController = pidController;
        this.logger = logger;
        fingerCurrentAngle = fingerServoWrapper.getLastReadPos();
    }

    public void updateState() {
        pidController.setPID(P, I, D);
        pidController.setSetPoint(0); // PIDs the error to 0
        pidController.setTolerance(PID_TOLERANCE); // sets the buffer
        updatePID();

        if (isTargetAngleToGrip()) {
            if (fingerServoWrapper.getLastReadPos() >= (targetAngle - PID_TOLERANCE)) {
                state = FingerStates.GRIPPED;
            } else {
                state = FingerStates.GRIPPING;
            }
        } else if (isTargetAngleToRelease()) {
            if (pidController.atSetPoint()) {
                state = FingerStates.RELEASED;
            } else {
                state = FingerStates.RELEASING;
            }
        }
    }

    public boolean isTargetAngleToRelease() {
        return targetAngle == RELEASED_POS;
    }

    public boolean isTargetAngleToGrip() {
        return targetAngle == GRIPPED_POS;
    }

    public void updatePID() { // This method is used to update position every loop.
        fingerServoWrapper.readPos();
        double angleDelta = angleDelta(fingerServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(fingerServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("Finger Power",power, Logger.LogLevels.PRODUCTION);
        fingerServoWrapper.set(power);

    }



    public void grip() {
        targetAngle = GRIPPED_POS;
    }

    public void release() {
        targetAngle = RELEASED_POS;
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
        return GRIPPED_POS;
    }
    public double getReleasedPos() {
        return RELEASED_POS;
    }



    public void log() {
        logger.log("Finger State",state, Logger.LogLevels.PRODUCTION);
        logger.log("Finger Current Position",fingerServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
        logger.log("Finger Target Pos",targetAngle, Logger.LogLevels.PRODUCTION);

    }
}
