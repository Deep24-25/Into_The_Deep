package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;

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
        RELAXED,
        FLEXED_TO_SPECIMEN_READY_POS,
        FLEXING_TO_SPECIMEN__READY_POS,
        FLEXING_TO_HIGH_CHAMBER_DEPOSIT,
        FLEXED_TO_HIGH_CHAMBER_DEPOSIT,
        FLEXING_TO_LOW_CHAMBER_DEPOSIT,
        FLEXED_TO_LOW_CHAMBER_DEPOSIT,
        RELAXING_FROM_CHAMBER_DEPOSIT,
        RELAXED_FROM_CHAMBER_DEPOSIT,
        FLEXING_TO_HIGH_BASKET_DEPOSIT,
        FLEXED_TO_HIGH_BASKET_DEPOSIT,
        FLEXING_TO_LOW_BASKET_DEPOSIT,
        FLEXED_TO_LOW_BASKET_DEPOSIT,
        RELAXING_FROM_BASKET_DEPOSIT,
        RELAXED_FROM_BASKET_DEPOSIT,
    }

    private double globalTargetAngle;
    public static double PID_TOLERANCE = 5;
    private double wristCurrentAngle;
    //Robot CONSTANTS:
    public static double P = 0.0075;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.02;

/*
    //test bench
    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0;*/

    public static double RELAXED_POS = 180;
    public static double SAMPLE_FLEXED_POS = 270;
    public static double SAMPLE_INTAKE_READY_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_CAPTURE_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_CONTROL_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_RETRACT_POS = RELAXED_POS;
    public static double SPECIMEN_FLEX_POS = 110.224725;

    public static double HIGH_CHAMBER_DEPOSIT_FLEXED_POS = 222;
    public static double LOW_CHAMBER_DEPOSIT_FLEXED_POS = 200;

    public static double HIGH_BASKET_DEPOSIT_FLEXED_POS = 270.1;
    public static double LOW_BASKET_DEPOSIT_FLEXED_POS = 270.1;

    public static double BASKET_RELAXED_POS = 180;


    private AxonServoWrapper wristServoWrapper;
    private PIDController pidController;

    private WristStates state;
    private Logger logger;
    private double encoderTargetAngle;

    private boolean relaxCalled = false;
    private boolean sampleControl = false;
    private boolean sampleIntakeReady = false;
    private boolean sampleCapture = false;
    private boolean sampleRetract = false;

    ElbowFSM elbowFSM;

    public static double ENCODER_OFFSET = 15;
    private static final double TOLERANCE = 10;


    public WristFSM(HWMap hwMap, Logger logger, ElbowFSM elbowFSM) {
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(), hwMap.getWristFlexEncoder(), true, true, ENCODER_OFFSET); // check if you need to reverse axons
        this.logger = logger;
        wristCurrentAngle = wristServoWrapper.getLastReadPos();
        this.elbowFSM = elbowFSM;
        globalTargetAngle = RELAXED_POS;
        state = WristStates.RELAXING;
    }

    @VisibleForTesting
    public WristFSM(AxonServoWrapper axonServoWrapper, Logger logger, PIDController pidController) {
        wristServoWrapper = axonServoWrapper;
        this.pidController = pidController;
        this.logger = logger;
        wristCurrentAngle = wristServoWrapper.getLastReadPos();
    }

    public void updateState() {
        wristCurrentAngle = wristServoWrapper.readPos();
        if (isTargetAngleToRelax() && relaxCalled) {
            if (atPos(TOLERANCE)) {
                state = WristStates.RELAXED;
            } else {
                state = WristStates.RELAXING;
            }
        } else if (isTargetAngleToSampleIntakeReadyFlexedPos() && sampleIntakeReady) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_READY_POS;
            }
        } else if (isTargetAngleToSampleIntakeCapturePos() && sampleCapture) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_CAPTURE_POS;
            }
        } else if (isTargetAngleToSampleIntakeControlPos() && sampleControl) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_CONTROL_POS;
            }
        } else if (isTargetAngleToSampleIntakeRetractPos() && sampleRetract) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS;
            } else {
                state = WristStates.FLEXING_TO_SAMPLE_INTAKE_RETRACT_POS;
            }
        } else if (isTargetAngleToFlex()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED;
            } else {
                state = WristStates.FLEXING;
            }
        } else if (isTargetAngleToSpecimenIntakeFlexPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_SPECIMEN_READY_POS;
            } else {
                state = WristStates.FLEXING_TO_SPECIMEN__READY_POS;
            }
        } else if (isTargetAngleToHighChamberDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_HIGH_CHAMBER_DEPOSIT;
            }
        } else if (isTargetAngleToLowChamberDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_LOW_CHAMBER_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_LOW_CHAMBER_DEPOSIT;
            }
        } else if (isTargetAngleToHighBasketDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_HIGH_BASKET_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_HIGH_BASKET_DEPOSIT;
            }
        } else if (isTargetAngleToLowBasketDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_LOW_BASKET_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_LOW_BASKET_DEPOSIT;
            }
        } else if (isTargetAngleToBasketRelax()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.RELAXED_FROM_BASKET_DEPOSIT;
            } else {
                state = WristStates.RELAXING_FROM_BASKET_DEPOSIT;
            }
        }

    }

    public boolean isTargetAngleToBasketRelax() {
        return globalTargetAngle == BASKET_RELAXED_POS;
    }

    public boolean isTargetAngleToHighBasketDepositFlexedPos() {
        return globalTargetAngle == HIGH_BASKET_DEPOSIT_FLEXED_POS;
    }

    public boolean isTargetAngleToLowBasketDepositFlexedPos() {
        return globalTargetAngle == LOW_BASKET_DEPOSIT_FLEXED_POS;
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


    public boolean isTargetAngleToSpecimenIntakeFlexPos() {
        return globalTargetAngle == SPECIMEN_FLEX_POS;
    }


    public boolean isTargetAngleToHighChamberDepositFlexedPos() {
        return globalTargetAngle == HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
    }

    public boolean isTargetAngleToLowChamberDepositFlexedPos() {
        return globalTargetAngle == LOW_CHAMBER_DEPOSIT_FLEXED_POS;
    }


    public void updatePID() { // This method is used to update position every loop.

        wristServoWrapper.readPos();
        encoderTargetAngle = convertGlobalAngleToEncoder(globalTargetAngle, elbowFSM.getTargetAngle());
        wristServoWrapper.set(encoderTargetAngle);
//
//        if(encoderTargetAngle < 62) {
//            encoderTargetAngle = 62;
//        }
//        if(encoderTargetAngle > 310) {
//            encoderTargetAngle = 310;
//        }
//
//
//        double angleDelta = angleDelta(wristServoWrapper.getLastReadPos(), encoderTargetAngle); // finds the minimum difference between current angle and target angle
//        double sign = angleDeltaSign(wristServoWrapper.getLastReadPos(), encoderTargetAngle); // sets the direction of servo based on minimum difference
//        double desiredSign = desiredSign(encoderTargetAngle);
//
//        if(!isActualSignEqualToDesiredSign(sign, desiredSign)) {
//            sign = -sign;
//            angleDelta = negateError(angleDelta);
//        }

//        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("--------------Wrist-----------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Current Angle", wristServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
        logger.log("Real Target angle", encoderTargetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("Global Target Angle", globalTargetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("--------------Wrist-----------", "-", Logger.LogLevels.PRODUCTION);

        //       wristServoWrapper.set((power + (F*(Math.cos(Math.toRadians(wristServoWrapper.getLastReadPos()))))));

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
        sampleRetract = false;
        F = 0.001;
        PID_TOLERANCE = 8;

    }

    public void flexToSampleIntakeControlPos() {
        globalTargetAngle = SAMPLE_INTAKE_CONTROL_POS;
        sampleControl = true;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;
        F = 0.001;
        PID_TOLERANCE = 8;

    }

    public void flexToSampleIntakeCapturePos() {
        globalTargetAngle = SAMPLE_INTAKE_CAPTURE_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = true;
        sampleRetract = false;
        F = 0.001;
        PID_TOLERANCE = 15;

    }

    public void flexToSampleIntakeRetractPos() {
        globalTargetAngle = SAMPLE_INTAKE_RETRACT_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = true;
        F = 0.005;
        PID_TOLERANCE = 12;

    }

    public void relax() {
        globalTargetAngle = RELAXED_POS;
        relaxCalled = true;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

        F = 0.005;
        PID_TOLERANCE = 12;
    }


    public void flexToSpecimenIntakePos() {
        globalTargetAngle = SPECIMEN_FLEX_POS;
        sampleControl = false;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

    }

    public void flexToHighBasketPos() {
        globalTargetAngle = HIGH_BASKET_DEPOSIT_FLEXED_POS;
        sampleControl = false;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

    }

    public void flexToLowBasketPos() {
        globalTargetAngle = LOW_BASKET_DEPOSIT_FLEXED_POS;
        sampleControl = false;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

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
        if (encoderTargetAngle > wristServoWrapper.getLastReadPos()) {
            return 1;
        } else if (encoderTargetAngle < wristServoWrapper.getLastReadPos()) {
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

    public boolean atPos(double tolerance) {
        return (wristCurrentAngle + tolerance <= encoderTargetAngle) || (wristCurrentAngle - tolerance >= encoderTargetAngle);
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

    public boolean FLEXED_TO_SPECIMEN_READY_POS() {
        return state == WristStates.FLEXED_TO_SPECIMEN_READY_POS;

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

    public boolean FLEXED_TO_HIGH_BASKET_DEPOSIT() {
        return state == WristStates.FLEXED_TO_HIGH_BASKET_DEPOSIT;
    }


    public boolean RELAXED_FROM_BASKET_DEPOSIT() {
        return state == WristStates.RELAXED_FROM_BASKET_DEPOSIT;
    }


    public void log() {
        logger.log("Wrist State", state, Logger.LogLevels.PRODUCTION);

        logger.log("Wrist Target Pos", globalTargetAngle, Logger.LogLevels.PRODUCTION);

        logger.log("Raw Angle", wristServoWrapper.getRawPos(), Logger.LogLevels.PRODUCTION);
    }

}
