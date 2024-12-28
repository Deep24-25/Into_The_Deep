package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;


import com.acmerobotics.dashboard.config.Config;

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
        FLEXED_TO_SPECIMEN_INTAKE_POS,
        FLEXING_TO_SPECIMEN_INTAKE_POS,
        FLEXING_TO_HIGH_CHAMBER_DEPOSIT,
        FLEXED_TO_HIGH_CHAMBER_DEPOSIT,
        SPECIMEN_INTAKE_RETRACTING,
        SPECIMEN_INTAKE_RETRACTED,
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
    public static double RELAXED_POS = 180;
    public static double SAMPLE_FLEXED_POS = 280;
    public static double SAMPLE_INTAKE_READY_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_CAPTURE_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_CONTROL_POS = SAMPLE_FLEXED_POS;
    public static double SAMPLE_INTAKE_RETRACT_POS = RELAXED_POS;
    public static double SPECIMEN_INTAKE_POS = 160;
    public static double SPECIMEN_INTAKE_RETRACT_POS = SPECIMEN_INTAKE_POS - 30;

    public static double HIGH_CHAMBER_DEPOSIT_FLEXED_POS = 75;
    //public static double LOW_CHAMBER_DEPOSIT_READY_FLEXED_POS = 90;


    public static double HIGH_BASKET_DEPOSIT_FLEXED_POS = 120;
    public static double LOW_BASKET_DEPOSIT_FLEXED_POS = 120;

    public static double BASKET_RELAXED_POS = 90;


    private final AxonServoWrapper wristServoWrapper;

    private WristStates state;
    private final Logger logger;
    private double encoderTargetAngle;

    private boolean relaxCalled = false;
    private boolean sampleControl = false;
    private boolean sampleIntakeReady = false;
    private boolean sampleCapture = false;
    private boolean sampleRetract = false;

    private final ElbowFSM elbowFSM;

    public static double ENCODER_OFFSET = -15;
    private static final double TOLERANCE = 15;


    public WristFSM(HWMap hwMap, Logger logger, ElbowFSM elbowFSM) {
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(), hwMap.getWristFlexEncoder(), true, true, ENCODER_OFFSET); // check if you need to reverse axons
        this.logger = logger;
        wristCurrentAngle = wristServoWrapper.getLastReadPos();
        this.elbowFSM = elbowFSM;
        globalTargetAngle = RELAXED_POS;
        state = WristStates.RELAXING;
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
                state = WristStates.FLEXED_TO_SPECIMEN_INTAKE_POS;
            } else {
                state = WristStates.FLEXING_TO_SPECIMEN_INTAKE_POS;
            }
        } else if (isTargetAngleToSpecimenIntakeRetractPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.SPECIMEN_INTAKE_RETRACTING;
            } else {
                state = WristStates.SPECIMEN_INTAKE_RETRACTED;
            }
        } else if (isTargetAngleToHighChamberDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_HIGH_CHAMBER_DEPOSIT;
            }
        } else if (isTargetAngleToLowBasketDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_LOW_BASKET_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_LOW_BASKET_DEPOSIT;
            }
        } else if (isTargetAngleToHighChamberDepositFlexedPos()) {
            if (atPos(TOLERANCE)) {
                state = WristStates.FLEXED_TO_HIGH_BASKET_DEPOSIT;
            } else {
                state = WristStates.FLEXING_TO_HIGH_BASKET_DEPOSIT;
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
        return globalTargetAngle == SPECIMEN_INTAKE_POS;
    }


    public boolean isTargetAngleToHighChamberDepositFlexedPos() {
        return globalTargetAngle == HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
    }


    public boolean isTargetAngleToSpecimenIntakeRetractPos() {
        return globalTargetAngle == SPECIMEN_INTAKE_RETRACT_POS;
    }

    public void updatePID() { // This method is used to update position every loop.
        wristServoWrapper.readPos();
        if (sampleControl || sampleCapture) {
            encoderTargetAngle = convertGlobalAngleToEncoder(globalTargetAngle, elbowFSM.getElbowCurrentAngle());
        } else {
            encoderTargetAngle = convertGlobalAngleToEncoder(globalTargetAngle, elbowFSM.getTargetAngle());

        }
        wristServoWrapper.set(encoderTargetAngle);
    }

    public void flexToSampleIntakeReadyPos() {
        globalTargetAngle = SAMPLE_INTAKE_READY_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = true;
        sampleCapture = false;
        sampleRetract = false;
        PID_TOLERANCE = 8;

    }

    public void flexToSampleIntakeControlPos() {
        globalTargetAngle = SAMPLE_INTAKE_CONTROL_POS;
        sampleControl = true;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;
        PID_TOLERANCE = 8;

    }

    public void flexToSampleIntakeCapturePos() {
        globalTargetAngle = SAMPLE_INTAKE_CAPTURE_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = true;
        sampleRetract = false;
        PID_TOLERANCE = 15;

    }

    public void flexToSampleIntakeRetractPos() {
        globalTargetAngle = SAMPLE_INTAKE_RETRACT_POS;
        relaxCalled = false;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = true;
        PID_TOLERANCE = 12;

    }

    public void relax() {
        globalTargetAngle = RELAXED_POS;
        relaxCalled = true;
        sampleControl = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

        PID_TOLERANCE = 12;
    }


    public void flexToSpecimenIntakePos() {
        globalTargetAngle = SPECIMEN_INTAKE_POS;
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

    public void flexToSpecimenRetractIntake() {
        globalTargetAngle = SPECIMEN_INTAKE_RETRACT_POS;
        sampleControl = false;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

    }

    public void flexToSpecimenDepositReadyPos() {
        globalTargetAngle = HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
        sampleControl = false;
        relaxCalled = false;
        sampleIntakeReady = false;
        sampleCapture = false;
        sampleRetract = false;

    }

    private double convertGlobalAngleToEncoder(double globalWristAngle, double elbowCurrentAngle) {
        return (globalWristAngle - elbowCurrentAngle - ENCODER_OFFSET) + 180;
    }

    public boolean atPos(double tolerance) {
        return (encoderTargetAngle + tolerance >= wristCurrentAngle) || (encoderTargetAngle - tolerance <= wristCurrentAngle);
    }

    public boolean RELAXED() {
        return state == WristStates.RELAXED;
    }

    public boolean FLEXED_TO_SPECIMEN_READY_POS() {
        return state == WristStates.FLEXED_TO_SPECIMEN_INTAKE_POS;

    }


    public boolean FLEXED_TO_SAMPLE_INTAKE_READY_POS() {
        return state == WristStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
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

    public boolean FLEXED_TO_HIGH_CHAMBER_DEPOSIT() {
        return state == WristStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
    }

    public boolean SPECIMEN_INTAKE_RETRACTED(){
        return  state == WristStates.SPECIMEN_INTAKE_RETRACTED;
    }


    public void setSampleCapture(boolean sampleCapture) {
        this.sampleCapture = sampleCapture;
    }

    public void log() {
        logger.log("--------------Wrist Log---------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Wrist State", state, Logger.LogLevels.PRODUCTION);
        logger.log("Current Angle", wristServoWrapper.getLastReadPos(), Logger.LogLevels.DEBUG);
        logger.log("Real Target angle", encoderTargetAngle, Logger.LogLevels.DEBUG);
        logger.log("--------------Wrist Log---------------", "-", Logger.LogLevels.PRODUCTION);
    }

}
