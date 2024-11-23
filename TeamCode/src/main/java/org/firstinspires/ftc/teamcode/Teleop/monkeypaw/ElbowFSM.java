package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

@Config
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
    public static  double PID_TOLERANCE = 2;
    private double elbowCurrentAngle;
    //Robot CONSTANTS:
    public static  double P = 1;
    public static  double I = 0;
    public static  double D = 0;
    public static  double F = 0;


    public static  double SAMPLE_FLEXED_POS = 90;
    public static  double SPECIMEN_FLEXED_POS = 30;
    public static  double DEPOSIT_FLEXED_POS = 180;
    public static  double RELAXED_POS = 0;

    private AxonServoWrapper elbowServoWrapper;
    private PIDFController pidfController;

    private ElbowStates state;
    private Logger logger;

    public ElbowFSM(HWMap hwMap, Logger logger) {
        elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(),hwMap.getElbowEncoder(),false, false); // check if you need to reverse axons
        pidfController = new PIDFController(P, I, D, F);
        this.logger = logger;
        elbowCurrentAngle = elbowServoWrapper.getLastReadPos();
    }
    @VisibleForTesting
    public ElbowFSM(AxonServoWrapper axonServoWrapper, Logger logger, PIDFController pidfController) {
        elbowServoWrapper = axonServoWrapper;
        this.pidfController = pidfController;
        this.logger = logger;
        elbowCurrentAngle = elbowServoWrapper.getLastReadPos();
    }

    public void updateState() {
        pidfController.setPIDF(P, I, D, F);
        pidfController.setSetPoint(0); // PIDs the error to 0
        pidfController.setTolerance(PID_TOLERANCE); // sets the buffer
        updatePID();

        if (isTargetAngleToSampleFlexedPos()) {
            if (pidfController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE;
            }
        }
        else if (isTargetAngleToSpecimenFlexedPos()) {
            if (pidfController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SPECIMEN_INTAKE;
            } else {
                state = ElbowStates.FLEXING_TO_SPECIMEN_INTAKE;
            }
        }
        else if (isTargetAngleToDepositFlexedPos()) {
            if (pidfController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_DEPOSIT;
            }
        }
        else if (isTargetAngleToRelax()) {
            if (pidfController.atSetPoint()) {
                state = ElbowStates.RELAXED;
            } else {
                state = ElbowStates.RELAXING;
            }
        }
    }

    public boolean isTargetAngleToRelax() {
        return targetAngle == RELAXED_POS;
    }

    public boolean isTargetAngleToSampleFlexedPos() {
        return targetAngle == SAMPLE_FLEXED_POS;
    }

    public boolean isTargetAngleToSpecimenFlexedPos() {
        return targetAngle == SPECIMEN_FLEXED_POS;
    }

    public boolean isTargetAngleToDepositFlexedPos() {
        return targetAngle == DEPOSIT_FLEXED_POS;
    }


    public void updatePID() { // This method is used to update position every loop.
        elbowServoWrapper.readPos();
        double angleDelta = angleDelta(elbowServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(elbowServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference
        double power = pidfController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("Elbow Power",power, Logger.LogLevels.DEBUG);
        elbowServoWrapper.set(power);

    }



    public void flexToSamplePos() {
        targetAngle = SAMPLE_FLEXED_POS;
    }

    public void flexToSpecimenPos() {
        targetAngle = SPECIMEN_FLEXED_POS;
    }
    public void flexToDepositPos() {
        targetAngle = DEPOSIT_FLEXED_POS;
    }
    public void relax() {

        //targetAngle = RELAXED_POS;
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

    public boolean FLEXED_TO_SAMPLE_INTAKE() {
        return state == ElbowStates.FLEXED_TO_SAMPLE_INTAKE;
    }

    public boolean FLEXING_TO_SAMPLE_INTAKE() {
        return state == ElbowStates.FLEXING_TO_SAMPLE_INTAKE;
    }

    public boolean FLEXING_TO_SPECIMEN_INTAKE() {
        return state == ElbowStates.FLEXING_TO_SPECIMEN_INTAKE;
    }

    public boolean FLEXED_TO_SPECIMEN_INTAKE() {
        return state == ElbowStates.FLEXED_TO_SPECIMEN_INTAKE;
    }

    public boolean FLEXED_TO_DEPOSIT() {
        return state == ElbowStates.FLEXED_TO_DEPOSIT;
    }

    public boolean FLEXING_TO_DEPOSIT() {
        return state == ElbowStates.FLEXING_TO_DEPOSIT;
    }

    public boolean RELAXING() {
        return state == ElbowStates.RELAXING;
    }

    public boolean RELAXED() {
        return state == ElbowStates.RELAXED;
    }

    public void log() {
        logger.log("Elbow State",state, Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Current Position",elbowServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Target Pos",targetAngle, Logger.LogLevels.PRODUCTION);

    }




}
