package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

public class ElbowFSM {

    public enum ElbowStates{
        FLEXING_TO_SAMPLE_INTAKE,
        FLEXED_TO_SAMPLE_INTAKE,
        FLEXING_TO_SPECIMEN_INTAKE,
        FLEXED_TO_SPECIMEN_INTAKE,
        RELAXED,
        RELAXING,
        FLEXING_TO_DEPOSIT,
        FLEXED_TO_DEPOSIT
    }
    private double targetAngle;
    private static final double PID_TOLERANCE = 0;
    private double fingerCurrentAngle;
    //Robot CONSTANTS:
    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;


    private static final double GRIPPED_POS = 0;
    private static final double RELEASED_POS = 0;

    private AxonServoWrapper fingerServoWrapper;
    private PIDController pidController;

    private FingerFSM.FingerStates state;
    private Logger logger;

    public FingerFSM(HWMap hwMap) {
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
            if (fingerServoWrapper.getLastReadPos() > (targetAngle)) {
                state = FingerFSM.FingerStates.GRIPPED;
            } else {
                state = FingerFSM.FingerStates.GRIPPING;
            }
        } else if (isTargetAngleToRelease()) {
            if (pidController.atSetPoint()) {
                state = FingerFSM.FingerStates.RELEASED;
            } else {
                state = FingerFSM.FingerStates.RELEASING;
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
        logger.log("Finger Power",power, Logger.LogLevels.DEBUG);
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
        return state == FingerFSM.FingerStates.GRIPPED;
    }

    public boolean GRIPPING() {
        return state == FingerFSM.FingerStates.GRIPPING;
    }

    public boolean RELEASED() {
        return state == FingerFSM.FingerStates.RELEASED;
    }

    public boolean RELEASING() {
        return state == FingerFSM.FingerStates.RELEASING;
    }



}
