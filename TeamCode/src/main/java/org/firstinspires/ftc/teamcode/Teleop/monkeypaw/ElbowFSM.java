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
        RELAXED_FROM_BASKET_DEPOSIT,
        SPEC_INTAKE_RETRACTED,
        SPEC_INTAKE_RETRACTING
    }

    private double targetAngle;
    public static double TOLERANCE = 100;

    public static double RELAXED_POS = 100;
    public static double SAMPLE_INTAKE_READY_POS = 150; //140.47-118.736
    public static double HOVERING_LOWER_LIMIT = 173;
    public static double HOVERING_UPPER_LIMIT = 173;
    public static double HOVERING_ANGLE = HOVERING_LOWER_LIMIT;
    public static double SAMPLE_INTAKE_CAPTURE_POS = 188;
    public static double SAMPLE_INTAKE_CONTROL_POS = 160;
    public static double SAMPLE_INTAKE_RETRACT_POS = RELAXED_POS;


    public static double SPECIMEN_INTAKE_FLEXED_POS = 126;
    public static double SPECIMEN_INTAKE_RELAX_POS = 120;
    public static double BASKET_DEPOSIT_FLEXED_POS = 140;
    public static double HIGH_CHAMBER_DEPOSIT_FLEXED_POS_TELE = 101;
    public static double HIGH_CHAMBER_DEPOSIT_FLEXED_POS_AUTO = 101; // 210
    public static double HIGH_CHAMBER_DEPOSIT_FLEXED_POS = HIGH_CHAMBER_DEPOSIT_FLEXED_POS_AUTO;

    public static double LOW_CHAMBER_DEPOSIT_FLEXED_POS = 135;

    public static double BASKET_RELAX_POS = 141;
    public static double CHAMBER_RELAX_POS = 0;


    private final AxonServoWrapper elbowServoWrapper;
    private ArmFSM armFSM;
    private ElbowStates state;
    private final Logger logger;


    private boolean relaxCalled = false;
    private boolean sampleControl = false;

    public static double ENCODER_OFFSET = -10;

    public static double CAPTURE_OFFSET = 57;

    public static double HOVER_TUNER = 30;


    public ShoulderFSM shoulderFSM;

    private static double hoveringOffset = 0;

    private double setCurrentAngle;

    public static final double RELAXED_CURRENT_ANGLE = 80;
    public static double SAMPLE_INTAKE_READY_POS_CURRENT_ANGLE = 130;
    public static double SAMPLE_INTAKE_CONTROL_POS_CURRENT_ANGLE = 130;
    public static final double INTAKE_RETRACTED_CURRENT_ANGLE = 80;
    public static final double BASKET_CURRENT_ANGLE = 110;
    public static final double SPEC_INTAKE_RETRACT_ANGLE = 111;
    private static final double RATIO = 26.0 / 16;
    private boolean isAuto = false;
    private int counter = 0;
    public static int COUNTER_LIMIT = 3;

    public ElbowFSM(HWMap hwMap, Logger logger, ShoulderFSM shoulderFSM) {
        elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(), hwMap.getElbowEncoder(), false, true, ENCODER_OFFSET, 1); // check if you need to reverse axons
        //     pidController = new PIDController(P, I, D);
        this.logger = logger;
        targetAngle = RELAXED_POS;
        this.shoulderFSM = shoulderFSM;
        setCurrentAngle = RELAXED_CURRENT_ANGLE;
    }

    @VisibleForTesting
    public ElbowFSM(AxonServoWrapper axonServoWrapper, Logger logger) {
        elbowServoWrapper = axonServoWrapper;
        this.logger = logger;
    }

    public void updateState() {
        updatePos();
        if (isAuto) {
            HIGH_CHAMBER_DEPOSIT_FLEXED_POS = HIGH_CHAMBER_DEPOSIT_FLEXED_POS_AUTO;
        } else {
            HIGH_CHAMBER_DEPOSIT_FLEXED_POS = HIGH_CHAMBER_DEPOSIT_FLEXED_POS_TELE;
        }
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
        else if (isTargetAngleToSpecIntakeRetractPos()) {
            if (atSetPoint()) {
                state = ElbowStates.SPEC_INTAKE_RETRACTED;
            } else {
                state = ElbowStates.SPEC_INTAKE_RETRACTING;
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

    public boolean isTargetAngleToSpecIntakeRetractPos() {
        return targetAngle == SPEC_INTAKE_RETRACT_ANGLE;
    }

    public void updatePos() {
        elbowServoWrapper.readPos();
        elbowServoWrapper.set(targetAngle);
    }

    public void flexToSpecimenRetractIntake() {
        targetAngle = SPEC_INTAKE_RETRACT_ANGLE;
    }

    public void flexToSampleHoveringPos() {
        double slope = ((HOVERING_LOWER_LIMIT - HOVERING_UPPER_LIMIT) / HOVER_TUNER);
        HOVERING_ANGLE = (slope * armFSM.getCurrentHeight()) + HOVERING_UPPER_LIMIT;
        targetAngle = HOVERING_ANGLE + hoveringOffset;
        sampleControl = false;
    }

    public void flexToSampleIntakeReadyPos() {
        targetAngle = SAMPLE_INTAKE_READY_POS;
        setCurrentAngle = SAMPLE_INTAKE_READY_POS_CURRENT_ANGLE;
        sampleControl = false;
    }

    public void flexToSampleIntakeControlPos() {
        targetAngle = SAMPLE_INTAKE_CONTROL_POS;
        setCurrentAngle = SAMPLE_INTAKE_CONTROL_POS_CURRENT_ANGLE;
        sampleControl = true;
    }

    public void flexToSampleIntakeCapturePos() {
        targetAngle = SAMPLE_INTAKE_CAPTURE_POS;
    }

    public void flexToSampleIntakeRetractPos() {
        targetAngle = SAMPLE_INTAKE_RETRACT_POS;
        setCurrentAngle = INTAKE_RETRACTED_CURRENT_ANGLE;
        relaxCalled = false;
    }


    public void flexToSpecimenIntakePos() {
        targetAngle = SPECIMEN_INTAKE_FLEXED_POS;
    }


    public void flexToBasketDepositFlexedPos() {
        targetAngle = BASKET_DEPOSIT_FLEXED_POS;
        setCurrentAngle = BASKET_CURRENT_ANGLE;
    }

    public void flexToHighChamberDepositFlexedPos() {
        targetAngle = HIGH_CHAMBER_DEPOSIT_FLEXED_POS;
    }

    public void relax() {
        targetAngle = RELAXED_POS;
        setCurrentAngle = RELAXED_CURRENT_ANGLE;
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
        logger.log("Elbow counter", counter, Logger.LogLevels.DEBUG);
        logger.log("Elbow counter limit", COUNTER_LIMIT, Logger.LogLevels.DEBUG);
        logger.log("Elbow Hovering Offset", hoveringOffset, Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Encoder Offset", ENCODER_OFFSET, Logger.LogLevels.PRODUCTION);
        logger.log("------------------------- ELBOW LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);


    }

    public void setArmFSM(ArmFSM armFSM) {
        this.armFSM = armFSM;
    }

    public void increaseHoverOffset() {
        hoveringOffset++;
    }

    public void decreaseHoverOffset() {
        hoveringOffset--;
    }

    public void increaseEncoderOffset() {
        ENCODER_OFFSET += 5;
        elbowServoWrapper.setEncoderOffset(ENCODER_OFFSET);
    }

    public void decreaseEncoderOffset() {
        ENCODER_OFFSET -= 5;
        elbowServoWrapper.setEncoderOffset(ENCODER_OFFSET);
    }

    public boolean specimenPickup() {
        //160, 163, 158, 142, 142, 142

        if (elbowServoWrapper.getLastReadPos() < (SPECIMEN_INTAKE_RELAX_POS - 8)) {
            counter++;
        } else {
            counter = 0;
        }
        return counter >= COUNTER_LIMIT;
    }

    public double getSetCurrentAngle() {
        return setCurrentAngle;
    }

    public boolean elbowHovering() {
        return targetAngle == HOVERING_ANGLE + hoveringOffset;
    }
    public void setIsAuto(boolean isAuto) {
        this.isAuto = isAuto;
    }
    public void resetCounter(){
        counter = 0;
    }

    public boolean SPECIMEN_INTAKE_RETRACTED() {
        return state == ElbowStates.SPEC_INTAKE_RETRACTED;
    }
}
