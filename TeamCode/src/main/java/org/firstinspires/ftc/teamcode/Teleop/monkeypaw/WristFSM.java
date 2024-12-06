package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
@Config
public class WristFSM {
    private enum WristStates {
        FLEXING_TO_SAMPLE_INTAKE_READY_POS,
        FLEXED_TO_SAMPLE_INTAKE_READY_POS,
        FLEXING_TO_SAMPLE_INTAKE_CAPTURE_POS,
        FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS,
        FLEXING_TO_SAMPLE_INTAKE_CONTROL_POS,
        FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS,
        FLEXING_TO_SAMPLE_INTAKE_RETRACT_POS,
        FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS,
        FLEXING,
        FLEXED,
        RELAXING,
        RELAXED
    }

    private double globalTargetAngle;
    public static  double PID_TOLERANCE = 5;
    private double wristCurrentAngle;
    //Robot CONSTANTS:
    public static double P = 0.0075;
    public static  double I = 0;
    public static  double D = 0;

/*
    //test bench
    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0;*/

    public static  double RELAXED_POS = 90;
    public static  double SAMPLE_FLEXED_POS = 270;
    public static  double SAMPLE_INTAKE_READY_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_CAPTURE_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_CONTROL_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_RETRACT_POS = RELAXED_POS;
    public static double SPECIMEN_FLEX_POS = 20;


    private AxonServoWrapper wristServoWrapper;
    private PIDController pidController;

    private WristStates state;
    private Logger logger;

    private boolean relaxCalled = false;
    private boolean sampleControl = false;
    private boolean sampleIntakeReady = false;
    private boolean sampleCapture = false;

    ElbowFSM elbowFSM;



    public WristFSM(HWMap hwMap, Logger logger, ElbowFSM elbowFSM) {
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(),hwMap.getWristFlexEncoder(),true, false, 0); // check if you need to reverse axons
        pidController = new PIDController(P, I, D);
        this.logger = logger;
        wristCurrentAngle = wristServoWrapper.getLastReadPos();
        this.elbowFSM = elbowFSM;
        globalTargetAngle = RELAXED_POS;

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
        if (isTargetAngleToRelax() && relaxCalled) {
            if (pidController.atSetPoint()) {
                state = WristStates.RELAXED;
            } else {
                state = WristStates.RELAXING;
            }
        }
        else if (isTargetAngleToSampleIntakeReadyFlexedPos() && sampleIntakeReady) {
            if (pidController.atSetPoint()) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_READY_POS;
            }
        }

        else if (isTargetAngleToSampleIntakeCapturePos() && sampleCapture) {
            if (pidController.atSetPoint()) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_CAPTURE_POS;
            }
        }

        else if (isTargetAngleToSampleIntakeControlPos() && sampleControl) {
            if (pidController.atSetPoint()) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_CONTROL_POS;
            }
        }

        else if (isTargetAngleToSampleIntakeRetractPos()) {
            if (pidController.atSetPoint()) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_RETRACT_POS;
            }
        }
        else if (isTargetAngleToFlex()) {
            if (pidController.atSetPoint()) {
                state = WristStates.FLEXED;
            } else {
                state = WristStates.FLEXING;
            }
        }
    }

    public boolean isTargetAngleToRelax() {
        return globalTargetAngle == RELAXED_POS;
    }

    public boolean isTargetAngleToFlex() {
        return globalTargetAngle == SAMPLE_FLEXED_POS;
    }

    public boolean isTargetAngleToSampleIntakeReadyFlexedPos() {
        return globalTargetAngle == SAMPLE_INTAKE_READY_POS;

    }

    public boolean isTargetAngleToSampleIntakeCapturePos() {
        return globalTargetAngle == SAMPLE_INTAKE_CAPTURE_POS;
    }

    public boolean isTargetAngleToSampleIntakeControlPos() {
        return globalTargetAngle == SAMPLE_INTAKE_CONTROL_POS;
    }

    public boolean isTargetAngleToSampleIntakeRetractPos() {

        return globalTargetAngle == SAMPLE_INTAKE_RETRACT_POS;
    }

    public void updatePID() { // This method is used to update position every loop.

        wristServoWrapper.readPos();

        double encoderTargetAngle = convertGlobalAngleToEncoder(globalTargetAngle, elbowFSM.getElbowCurrentAngle());

        if(encoderTargetAngle < 85) {
            encoderTargetAngle = 85;
        }
        if(encoderTargetAngle > 271) {
            encoderTargetAngle = 271;
        }


        double angleDelta = angleDelta(wristServoWrapper.getLastReadPos(), encoderTargetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(wristServoWrapper.getLastReadPos(), encoderTargetAngle); // sets the direction of servo based on minimum difference
        double desiredSign = desiredSign(encoderTargetAngle);

        if(!isActualSignEqualToDesiredSign(sign, desiredSign)) {
            sign = -sign;
            angleDelta = negateError(angleDelta);
        }

        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("encoder target angle", encoderTargetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("--------------Wrist Current Position-----------",wristServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
        logger.log("PID Power", power, Logger.LogLevels.PRODUCTION);
        logger.log("Actual Servo Power", wristServoWrapper.get(), Logger.LogLevels.PRODUCTION);
        wristServoWrapper.set(power);

    }


    public void flex() {
        globalTargetAngle = SAMPLE_FLEXED_POS;
    }

    public void flexToSampleIntakeReadyPos() {
        globalTargetAngle = SAMPLE_INTAKE_READY_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = true;
        sampleCapture = false;
    }

    public void flexToSampleIntakeControlPos() {
        globalTargetAngle = SAMPLE_INTAKE_CONTROL_POS;
        sampleControl = true;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
    }

    public void flexToSampleIntakeCapturePos() {
        globalTargetAngle = SAMPLE_INTAKE_CAPTURE_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = true;
    }

    public void flexToSampleIntakeRetractPos() {
        globalTargetAngle = SAMPLE_INTAKE_RETRACT_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = false;
    }

    public void relax() {
        globalTargetAngle = RELAXED_POS;
        relaxCalled = true;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = false;
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

    private boolean isActualSignEqualToDesiredSign(double actualSign, double desiredSign) {
        return actualSign == desiredSign;
    }

    private double desiredSign(double encoderTargetAngle) {
        if(encoderTargetAngle > wristServoWrapper.getLastReadPos()) {
            return 1;
        }
        else if(encoderTargetAngle < wristServoWrapper.getLastReadPos()) {
            return -1;
        }
        return 0;
    }

    private double negateError(double currentError) {
        return 360 - Math.abs(currentError);
    }

    private double convertGlobalAngleToEncoder(double globalWristAngle, double elbowCurrentAngle) {
        return (globalWristAngle - elbowCurrentAngle) + 180;
       /* if(elbowAngle > 180) {
            return wristAngle + (360 -elbowAngle);
        }
        else if(elbowAngle < 180) {
            return wristAngle - elbowAngle;
        }*/
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


    public boolean FLEXED_TO_SAMPLE_INTAKE_READY_POS() {
        return state == WristStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
    }

    public boolean FLEXING_TO_SAMPLE_INTAKE_READY_POS() {
        return state == WristStates.FLEXING_TO_SAMPLE_INTAKE_READY_POS;
    }

    public boolean FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS() {
        return state == WristStates.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS;
    }

    public boolean FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS() {
        return state == WristStates.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS;
    }

    public boolean FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS() {
        return state == WristStates.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS;
    }


    public void log() {
        logger.log("Wrist State",state, Logger.LogLevels.PRODUCTION);

        logger.log("Wrist Target Pos", globalTargetAngle, Logger.LogLevels.PRODUCTION);

        logger.log("Raw Angle", wristServoWrapper.getRawPos(), Logger.LogLevels.PRODUCTION);
    }

}
