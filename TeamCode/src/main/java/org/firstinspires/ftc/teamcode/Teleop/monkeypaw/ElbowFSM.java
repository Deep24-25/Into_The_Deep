package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

@Config
public class ElbowFSM {

    public enum ElbowStates{
        FLEXING_TO_SAMPLE_INTAKE_READY_POS,
        FLEXED_TO_SAMPLE_INTAKE_READY_POS,
        FLEXING_TO_SAMPLE_INTAKE_CAPTURE_POS,
        FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS,
        FLEXING_TO_SAMPLE_INTAKE_CONTROL_POS,
        FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS,
        FLEXING_TO_SAMPLE_INTAKE_RETRACT_POS,
        FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS,
        FLEXING_TO_SPECIMEN_INTAKE,
        FLEXED_TO_SPECIMEN_INTAKE,
        RELAXING_TO_SPECIMEN_INTAKE_RELAX_POS,
        RELAXED_TO_SPECIMEN_INTAKE_RELAX_POS,
        RELAXED,
        RELAXING,
        FLEXING_TO_BASKET_DEPOSIT,
        FLEXED_TO_BASKET_DEPOSIT,
        FLEXING_TO_HIGH_CHAMBER_DEPOSIT,
        FLEXED_TO_HIGH_CHAMBER_DEPOSIT,
        FLEXING_TO_LOW_CHAMBER_DEPOSIT,
        FLEXED_TO_LOW_CHAMBER_DEPOSIT,
        RELAXING_FROM_CHAMBER_DEPOSIT,
        RELAXED_FROM_CHAMBER_DEPOSIT,
        RELAXING_FROM_BASKET_DEPOSIT,
        RELAXED_FROM_BASKET_DEPOSIT
    }
    private double targetAngle;
    public static  double PID_TOLERANCE = 5;
    private double elbowCurrentAngle;
    //Robot CONSTANTS:
    public static double P = 0.007;
    public static double I = 0;
    public static double D = 0;
    public static double F = -0.2;

    //PID Relax constants:
    public static double Prelax = 0.007;
    public static double Irelax = 0;
    public static double Drelax = 0;
    public static double Frelax = -0.2;


    //PID SampleIntakeReady constants:
    public static double PSampleIntakeReady = 0.007;
    public static double ISampleIntakeReady = 0;
    public static double DSampleIntakeReady = 0;
    public static double FSampleIntakeReady = -0.1;


    //PID SampleIntakeCapture constants:
    public static double PSampleIntakeCapturePos = 0.007;
    public static double ISampleIntakeCapturePos = 0;
    public static double DSampleIntakeCapturePos = 0;
    public static double FSampleIntakeCapturePos = -0.1;



    //PID SampleIntakeControl constants:
    public static double PSampleIntakeControlPos = 0.007;
    public static double ISampleIntakeControlPos = 0;
    public static double DSampleIntakeControlPos = 0;
    public static double FSampleIntakeControlPos = -0.1;


    //PID SampleIntakeRetract constants:
    public static double PSampleIntakeRetractPos = 0.007;
    public static double ISampleIntakeRetractPos = 0;
    public static double DSampleIntakeRetractPos = 0;
    public static double FSampleIntakeRetractPos = -0.2;

/*
    //test bench
    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;*/


    public static  double RELAXED_POS = 66;
    public static  double SAMPLE_INTAKE_READY_POS = 149.521675;
    public static double SAMPLE_INTAKE_CAPTURE_POS = 170;
    public static double SAMPLE_INTAKE_CONTROL_POS = SAMPLE_INTAKE_READY_POS;
    public static double SAMPLE_INTAKE_RETRACT_POS = RELAXED_POS;


    public static  double SPECIMEN_INTAKE_FLEXED_POS = 33;
    public static  double SPECIMEN_INTAKE_RELAX_POS = 30;
    public static  double BASKET_DEPOSIT_FLEXED_POS = 180;
    public static  double HIGH_CHAMBER_DEPOSIT_FLEXED_POS = 222;
    public static  double LOW_CHAMBER_DEPOSIT_FLEXED_POS = 200;

    public static  double BASKET_RELAX_POS = 0;
    public static double CHAMBER_RELAX_POS = 0;

    private AxonServoWrapper elbowServoWrapper;
    private PIDController pidController;

    private ElbowStates state;
    private Logger logger;

    private boolean relaxCalled = false;
    private boolean sampleControl = false;

    public static double ENCODER_OFFSET = 7.0;


    public ElbowFSM(HWMap hwMap, Logger logger) {
        elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(),hwMap.getElbowEncoder(),false, false, ENCODER_OFFSET); // check if you need to reverse axons
        pidController = new PIDController(P, I, D);
        this.logger = logger;
        elbowCurrentAngle = elbowServoWrapper.getLastReadPos();
        targetAngle = RELAXED_POS;
    }
    @VisibleForTesting
    public ElbowFSM(AxonServoWrapper axonServoWrapper, Logger logger, PIDController pidController) {
        elbowServoWrapper = axonServoWrapper;
        this.pidController = pidController;
        this.logger = logger;
        elbowCurrentAngle = elbowServoWrapper.getLastReadPos();
    }

    public void updateState() {
        pidController.setSetPoint(0); // PIDs the error to 0
        pidController.setTolerance(PID_TOLERANCE); // sets the buffer
        updatePID();

         if (isTargetAngleToRelax() && relaxCalled) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.RELAXED;
            } else {
                state = ElbowStates.RELAXING;
            }
        }
        else if (isTargetAngleToSampleIntakeReadyFlexedPos() && !sampleControl) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_READY_POS;
            }
        }

        else if (isTargetAngleToSampleIntakeCapturePos()) {
            if (elbowServoWrapper.getLastReadPos() >= (targetAngle-15)) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_CAPTURE_POS;
            }
        }

        else if (isTargetAngleToSampleIntakeControlPos() && sampleControl) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_CONTROL_POS;
            }
        }

        else if (isTargetAngleToSampleIntakeRetractPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_RETRACT_POS;
            }
        }
        else if (isTargetAngleToSpecimenFlexedPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SPECIMEN_INTAKE;
            } else {
                state = ElbowStates.FLEXING_TO_SPECIMEN_INTAKE;
            }
        }
        else if (isTargetAngleToSpecimenIntakeRelaxedPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.RELAXED_TO_SPECIMEN_INTAKE_RELAX_POS;
            } else {
                state = ElbowStates.RELAXING_TO_SPECIMEN_INTAKE_RELAX_POS;
            }
        }
        else if (isTargetAngleToBasketDepositFlexedPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_BASKET_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_BASKET_DEPOSIT;
            }
        }
        else if (isTargetAngleToHighChamberDepositFlexedPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_HIGH_CHAMBER_DEPOSIT;
            }
        }

        else if (isTargetAngleToLowChamberDepositFlexedPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.FLEXED_TO_LOW_CHAMBER_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_LOW_CHAMBER_DEPOSIT;
            }
        }

        else if (isTargetAngleToChamberRelaxPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.RELAXED_FROM_CHAMBER_DEPOSIT;
            } else {
                state = ElbowStates.RELAXING_FROM_CHAMBER_DEPOSIT;
            }
        }

        else if (isTargetAngleToBasketRelaxPos()) {
            if (pidController.atSetPoint()) {
                state = ElbowStates.RELAXED_FROM_BASKET_DEPOSIT;
            } else {
                state = ElbowStates.RELAXING_FROM_BASKET_DEPOSIT;
            }
        }

    }

    public boolean isTargetAngleToRelax() {
        return targetAngle == RELAXED_POS;
    }

    public boolean isTargetAngleToSampleIntakeReadyFlexedPos() {
        return targetAngle == SAMPLE_INTAKE_READY_POS;
    }

    public boolean isTargetAngleToSampleIntakeCapturePos() {
        return targetAngle == SAMPLE_INTAKE_CAPTURE_POS;
    }

    public boolean isTargetAngleToSampleIntakeControlPos() {
        return targetAngle == SAMPLE_INTAKE_CONTROL_POS;
    }

    public boolean isTargetAngleToSampleIntakeRetractPos() {
        return targetAngle == SAMPLE_INTAKE_RETRACT_POS;
    }


    public boolean isTargetAngleToSpecimenFlexedPos() {
        return targetAngle == SPECIMEN_INTAKE_FLEXED_POS;
    }

    public boolean isTargetAngleToSpecimenIntakeRelaxedPos() {
        return targetAngle == SPECIMEN_INTAKE_RELAX_POS;
    }

    public boolean isTargetAngleToBasketDepositFlexedPos() {
        return targetAngle == BASKET_DEPOSIT_FLEXED_POS;
    }
    public boolean isTargetAngleToHighChamberDepositFlexedPos() {
        return targetAngle == HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
    }

    public boolean isTargetAngleToLowChamberDepositFlexedPos() {
        return targetAngle == LOW_CHAMBER_DEPOSIT_FLEXED_POS;
    }

    public boolean isTargetAngleToBasketRelaxPos() {
        return targetAngle == BASKET_RELAX_POS;
    }

    public boolean isTargetAngleToChamberRelaxPos() {
        return targetAngle == CHAMBER_RELAX_POS;
    }


    public void updatePID() { // This method is used to update position every loop.

        if(targetAngle > 308) {
            targetAngle = 308;
        }
        if(targetAngle < 32) {
            targetAngle = 32;
        }


        elbowServoWrapper.readPos();
        double angleDelta = angleDelta(elbowServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(elbowServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference


        if(!isActualSignEqualToDesiredSign(sign)) {
            sign = -sign;
            angleDelta = negateError(angleDelta);
        }
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("PID Power", power, Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Power",(power + (F*(Math.toRadians(Math.cos(elbowServoWrapper.getLastReadPos()))))), Logger.LogLevels.PRODUCTION);
        logger.log("Cosine", Math.cos(Math.toRadians(elbowServoWrapper.getLastReadPos())), Logger.LogLevels.PRODUCTION);
        logger.log("Actual Servo Power", elbowServoWrapper.get(), Logger.LogLevels.PRODUCTION);
        elbowServoWrapper.set((power + (F*(Math.cos(Math.toRadians(elbowServoWrapper.getLastReadPos()))))));

    }



    public void flexToSampleIntakeReadyPos() {
        targetAngle = SAMPLE_INTAKE_READY_POS;
        sampleControl = false;
        pidController.setPID(PSampleIntakeReady,ISampleIntakeReady,DSampleIntakeReady);
        F = FSampleIntakeReady;
    }

    public void flexToSampleIntakeControlPos() {
        targetAngle = SAMPLE_INTAKE_CONTROL_POS;
        sampleControl = true;
        pidController.setPID(PSampleIntakeControlPos,ISampleIntakeControlPos,DSampleIntakeControlPos);
        F = FSampleIntakeControlPos;
    }

    public void flexToSampleIntakeCapturePos() {
        targetAngle = SAMPLE_INTAKE_CAPTURE_POS;
        pidController.setPID(PSampleIntakeCapturePos,ISampleIntakeCapturePos,DSampleIntakeCapturePos);
        F = FSampleIntakeCapturePos;
    }

    public void flexToSampleIntakeRetractPos() {
            targetAngle = SAMPLE_INTAKE_RETRACT_POS;
            relaxCalled = false;
            pidController.setPID(PSampleIntakeRetractPos, ISampleIntakeRetractPos, DSampleIntakeRetractPos);
            F = FSampleIntakeRetractPos;
    }


    public void flexToSpecimenIntakePos() {
        targetAngle = SPECIMEN_INTAKE_FLEXED_POS;
    }


    public void relaxToSpecimenIntakeRelaxedPos() {
        targetAngle = SPECIMEN_INTAKE_RELAX_POS;
    }

    public void flexToBasketDepositFlexedPos() {
        targetAngle = BASKET_DEPOSIT_FLEXED_POS;
    }

    public void relaxToBasketDepositRelaxedPos() {
        targetAngle = BASKET_RELAX_POS;
    }

    public void flexToHighChamberDepositFlexedPos() {
        targetAngle = HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
    }
    public void flexToLowChamberDepositFlexedPos() {
        targetAngle = LOW_CHAMBER_DEPOSIT_FLEXED_POS;
    }

    public void relaxToChamberDepositRelaxedPos() {
        targetAngle = CHAMBER_RELAX_POS;
    }

    public void relax() {
        targetAngle = RELAXED_POS;
        relaxCalled = true;
        pidController.setPID(Prelax,Irelax,Drelax);
        F = Frelax;
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


    private boolean isActualSignEqualToDesiredSign(double actualSign) {
        return actualSign == desiredSign();
    }

    private double desiredSign() {
        if(targetAngle > elbowServoWrapper.getLastReadPos()) {
            return 1;
        }
        else if(targetAngle < elbowServoWrapper.getLastReadPos()) {
            return -1;
        }
        return 0;
    }

    private double negateError(double currentError) {
        return 360 - Math.abs(currentError);
    }



    public boolean FLEXED_TO_SAMPLE_INTAKE_READY_POS() {
        return state == ElbowStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
    }

    public boolean FLEXING_TO_SAMPLE_INTAKE_READY_POS() {
        return state == ElbowStates.FLEXING_TO_SAMPLE_INTAKE_READY_POS;
    }

    public boolean FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS() {
        return state == ElbowStates.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS;
    }

    public boolean FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS() {
        return state == ElbowStates.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS;
    }

    public boolean FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS() {
        return state == ElbowStates.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS;
    }

    public boolean FLEXING_TO_SPECIMEN_INTAKE() {
        return state == ElbowStates.FLEXING_TO_SPECIMEN_INTAKE;
    }

    public boolean FLEXED_TO_SPECIMEN_INTAKE() {
        return state == ElbowStates.FLEXED_TO_SPECIMEN_INTAKE;
    }

    public boolean RELAXED_TO_SPECIMEN_INTAKE_RELAX_POS() {
        return state == ElbowStates.RELAXED_TO_SPECIMEN_INTAKE_RELAX_POS;
    }
    public boolean RELAXING() {
        return state == ElbowStates.RELAXING;
    }

    public boolean RELAXED() {
        return state == ElbowStates.RELAXED;
    }

    public boolean FLEXING_TO_BASKET_DEPOSIT(){
        return state == ElbowStates.FLEXING_TO_BASKET_DEPOSIT;
    }
    public boolean FLEXED_TO_BASKET_DEPOSIT(){
        return state == ElbowStates.FLEXING_TO_BASKET_DEPOSIT;
    }

    public boolean FLEXING_TO_HIGH_CHAMBER_DEPOSIT(){
        return state == ElbowStates.FLEXING_TO_HIGH_CHAMBER_DEPOSIT;
    }

    public boolean FLEXED_TO_HIGH_CHAMBER_DEPOSIT(){
        return state == ElbowStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
    }

    public boolean FLEXING_TO_LOW_CHAMBER_DEPOSIT(){
        return state == ElbowStates.FLEXING_TO_LOW_CHAMBER_DEPOSIT;
    }

    public boolean FLEXED_TO_LOW_CHAMBER_DEPOSIT(){
        return state == ElbowStates.FLEXED_TO_LOW_CHAMBER_DEPOSIT;
    }

    public boolean RELAXING_FROM_CHAMBER_DEPOSIT(){
        return state == ElbowStates.RELAXING_FROM_CHAMBER_DEPOSIT;
    }

    public boolean RELAXED_FROM_CHAMBER_DEPOSIT(){
        return state == ElbowStates.RELAXED_FROM_CHAMBER_DEPOSIT;
    }

    public boolean RELAXING_FROM_BASKET_DEPOSIT(){
        return state == ElbowStates.RELAXING_FROM_BASKET_DEPOSIT;
    }

    public boolean RELAXED_FROM_BASKET_DEPOSIT(){
        return state == ElbowStates.RELAXED_FROM_BASKET_DEPOSIT;
    }

    public double getElbowCurrentAngle() {
        return elbowServoWrapper.getLastReadPos();
    }

    public double getTargetAngle() {
        return targetAngle;
    }


    public void log() {
        logger.log("Elbow State",state, Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Current Position",elbowServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Target Pos",targetAngle, Logger.LogLevels.PRODUCTION);

    }



}
