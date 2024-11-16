package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

public class WristFSM {
    private enum WristStates {
        FLEXING,
        FLEXED,
        RELAXING,
        RELAXED
    }

    private double targetAngle;
    private static final double PID_TOLERANCE = 0;
    private double wristCurrentAngle;
    //Robot CONSTANTS:
    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;


    private static final double FLEXED_POS = 0;
    private static final double RELAXED_POS = 0;

    private AxonServoWrapper wristServoWrapper;
    private PIDController pidController;

    private WristStates state;
    private Logger logger;

    public WristFSM(HWMap hwMap) {
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(),hwMap.getWristDeviEncoder(),false, false); // check if you need to reverse axons
        pidController = new PIDController(P, I, D);
        this.logger = logger;
        wristCurrentAngle = wristServoWrapper.getLastReadPos();
    }
    @VisibleForTesting
    public WristFSM(AxonServoWrapper axonServoWrapper, Logger logger, PIDController pidController) {
        wristServoWrapper = axonServoWrapper;
        this.pidController = pidController;
        this.logger = logger;
        wristCurrentAngle = wristServoWrapper.getLastReadPos();
    }

    public void updateState() {
        pidController.setPID(P, I, D);
        pidController.setSetPoint(0); // PIDs the error to 0
        pidController.setTolerance(PID_TOLERANCE); // sets the buffer
        updatePID();

        if (isTargetAngleToFlex()) {
            if (pidController.atSetPoint()) {
                state = WristStates.FLEXED;
            } else {
                state = WristStates.FLEXING;
            }
        } else if (isTargetAngleToRelax()) {
            if (pidController.atSetPoint()) {
                state = WristStates.RELAXED;
            } else {
                state = WristStates.RELAXING;
            }
        }
    }

    public boolean isTargetAngleToRelax() {
        return targetAngle == RELAXED_POS;
    }

    public boolean isTargetAngleToFlex() {
        return targetAngle == FLEXED_POS;
    }

    public void updatePID() { // This method is used to update position every loop.
        wristServoWrapper.readPos();
        double angleDelta = angleDelta(wristServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(wristServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("Finger Power",power, Logger.LogLevels.DEBUG);
        wristServoWrapper.set(power);

    }

    public void flex() {
        targetAngle = FLEXED_POS;
    }

    public void relax() {
        targetAngle = RELAXED_POS;
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

    public boolean FLEXED() {
        return state == WristStates.FLEXED;
    }

    public boolean FLEXING() {
        return state == WristStates.FLEXING;
    }

    public boolean RELAXED() {
        return state == WristStates.RELAXED;
    }

    public boolean RELAXING() {
        return state == WristStates.RELAXING;
    }

}
