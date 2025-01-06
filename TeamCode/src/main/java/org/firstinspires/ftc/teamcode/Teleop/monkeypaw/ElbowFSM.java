package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ArmFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ShoulderFSM;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;

@Config
public class ElbowFSM {

    public enum ElbowStates {
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
    public static double TOLERANCE = 100;

    public static double RELAXED_POS = 65;
    public static double SAMPLE_INTAKE_READY_POS = 120; //140.47-118.736
    public static double HOVERING_LOWER_LIMIT = 128;
    public static double HOVERING_UPPER_LIMIT = 135;
    public static double HOVERING_ANGLE = HOVERING_LOWER_LIMIT;
    public static double SAMPLE_INTAKE_CAPTURE_POS = 155;
    public static double SAMPLE_INTAKE_CONTROL_POS = 120;
    public static double SAMPLE_INTAKE_RETRACT_POS = RELAXED_POS;


    public static double SPECIMEN_INTAKE_FLEXED_POS = 66;
    public static double SPECIMEN_INTAKE_RELAX_POS = 70;
    public static double BASKET_DEPOSIT_FLEXED_POS = 110;
    public static double HIGH_CHAMBER_DEPOSIT_FLEXED_POS = 90;
    public static double LOW_CHAMBER_DEPOSIT_FLEXED_POS = 135;

    public static double BASKET_RELAX_POS = 90;
    public static double CHAMBER_RELAX_POS = 0;


    private final AxonServoWrapper elbowServoWrapper;
    private ArmFSM armFSM;
    private ElbowStates state;
    private final Logger logger;


    private boolean relaxCalled = false;
    private boolean sampleControl = false;

    public static double ENCODER_OFFSET = -38;

    public static double CAPTURE_OFFSET = 57;

    public static double HOVER_TUNER = 30;

    public ShoulderFSM shoulderFSM;

    public ElbowFSM(HWMap hwMap, Logger logger, ShoulderFSM shoulderFSM) {
        elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(), hwMap.getElbowEncoder(), false, false, ENCODER_OFFSET); // check if you need to reverse axons
        //     pidController = new PIDController(P, I, D);
        this.logger = logger;
        targetAngle = RELAXED_POS;
        this.shoulderFSM = shoulderFSM;
    }

    @VisibleForTesting
    public ElbowFSM(AxonServoWrapper axonServoWrapper, Logger logger) {
        elbowServoWrapper = axonServoWrapper;
        this.logger = logger;
    }

    public void updateState() {
        updatePos();
        if (isTargetAngleToRelax() && relaxCalled) {
            if (atSetPoint()) {
                state = ElbowStates.RELAXED;
            } else {
                state = ElbowStates.RELAXING;
            }
        } else if (isTargetAngleToSampleIntakeReadyFlexedPos() && !sampleControl) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_READY_POS;
            }
        } else if (isTargetAngleToSampleIntakeCapturePos()) {
            if (elbowServoWrapper.getLastReadPos() >= (targetAngle - CAPTURE_OFFSET)) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_CAPTURE_POS;
            }
        } else if (isTargetAngleToSampleIntakeControlPos() && sampleControl) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_CONTROL_POS;
            }
        } else if (isTargetAngleToSampleIntakeRetractPos()) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS;
            } else {
                state = ElbowStates.FLEXING_TO_SAMPLE_INTAKE_RETRACT_POS;
            }
        } else if (isTargetAngleToBasketDepositFlexedPos()) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_BASKET_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_BASKET_DEPOSIT;
            }
        } else if (isTargetAngleToSpecimenFlexedPos()) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_SPECIMEN_INTAKE;
            } else {
                state = ElbowStates.FLEXING_TO_SPECIMEN_INTAKE;
            }
        } else if (isTargetAngleToSpecimenIntakeRelaxedPos()) {
            if (atSetPoint()) {
                state = ElbowStates.RELAXED_TO_SPECIMEN_INTAKE_RELAX_POS;
            } else {
                state = ElbowStates.RELAXING_TO_SPECIMEN_INTAKE_RELAX_POS;
            }
        } else if (isTargetAngleToHighChamberDepositFlexedPos()) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_HIGH_CHAMBER_DEPOSIT;
            }
        } else if (isTargetAngleToLowChamberDepositFlexedPos()) {
            if (atSetPoint()) {
                state = ElbowStates.FLEXED_TO_LOW_CHAMBER_DEPOSIT;
            } else {
                state = ElbowStates.FLEXING_TO_LOW_CHAMBER_DEPOSIT;
            }
        } else if (isTargetAngleToChamberRelaxPos()) {
            if (atSetPoint()) {
                state = ElbowStates.RELAXED_FROM_CHAMBER_DEPOSIT;
            } else {
                state = ElbowStates.RELAXING_FROM_CHAMBER_DEPOSIT;
            }
        } else if (isTargetAngleToBasketRelaxPos()) {
            if (atSetPoint()) {
                state = ElbowStates.RELAXED_FROM_BASKET_DEPOSIT;
            } else {
                state = ElbowStates.RELAXING_FROM_BASKET_DEPOSIT;
            }
        }

    }

    public boolean atSetPoint() {
        return (elbowServoWrapper.getLastReadPos() <= targetAngle + TOLERANCE) && (elbowServoWrapper.getLastReadPos() >= targetAngle - TOLERANCE);
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

    public void updatePos() {
        elbowServoWrapper.readPos();
        elbowServoWrapper.set(targetAngle);
    }

    public void flexToSampleHoveringPos() {
        double slope = ((HOVERING_LOWER_LIMIT - HOVERING_UPPER_LIMIT) / HOVER_TUNER);
        HOVERING_ANGLE = (slope * armFSM.getCurrentHeight()) + HOVERING_UPPER_LIMIT;
        targetAngle = HOVERING_ANGLE;
        sampleControl = false;
    }

    public void flexToSampleIntakeReadyPos() {
        targetAngle = SAMPLE_INTAKE_READY_POS;
        sampleControl = false;
    }

    public void flexToSampleIntakeControlPos() {
        targetAngle = SAMPLE_INTAKE_CONTROL_POS;
        sampleControl = true;
    }

    public void flexToSampleIntakeCapturePos() {
        targetAngle = SAMPLE_INTAKE_CAPTURE_POS;
    }

    public void flexToSampleIntakeRetractPos() {
        targetAngle = SAMPLE_INTAKE_RETRACT_POS;
        relaxCalled = false;
    }


    public void flexToSpecimenIntakePos() {
        targetAngle = SPECIMEN_INTAKE_FLEXED_POS;
    }


    public void flexToBasketDepositFlexedPos() {
        targetAngle = BASKET_DEPOSIT_FLEXED_POS;
    }

    public void flexToHighChamberDepositFlexedPos() {
        targetAngle = HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
    }

    public void relax() {
        targetAngle = RELAXED_POS;
        relaxCalled = true;
    }


    public boolean FLEXED_TO_SAMPLE_INTAKE_READY_POS() {
        return state == ElbowStates.FLEXED_TO_SAMPLE_INTAKE_READY_POS;
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


    public boolean FLEXED_TO_SPECIMEN_INTAKE() {
        return state == ElbowStates.FLEXED_TO_SPECIMEN_INTAKE;
    }

    public boolean RELAXED() {
        return state == ElbowStates.RELAXED;
    }


    public boolean FLEXED_TO_BASKET_DEPOSIT() {
        return state == ElbowStates.FLEXED_TO_BASKET_DEPOSIT;
    }


    public boolean FLEXED_TO_HIGH_CHAMBER_DEPOSIT() {
        return state == ElbowStates.FLEXED_TO_HIGH_CHAMBER_DEPOSIT;
    }

    public double getElbowCurrentAngle() {
        return elbowServoWrapper.getLastReadPos();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getIntakeReadyAngle() {
        return SAMPLE_INTAKE_READY_POS;
    }


    public void log() {
        logger.log("------------------------- ELBOW LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Elbow State", state, Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Current Position", elbowServoWrapper.getLastReadPos(), Logger.LogLevels.DEBUG);
        logger.log("Elbow Target Pos", targetAngle, Logger.LogLevels.DEBUG);
        logger.log("------------------------- ELBOW LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);


    }

    public void setArmFSM(ArmFSM armFSM) {
        this.armFSM = armFSM;
    }

    public void setShoulderFSM(ShoulderFSM shoulderFSM) {
        this.shoulderFSM = shoulderFSM;
    }


}
