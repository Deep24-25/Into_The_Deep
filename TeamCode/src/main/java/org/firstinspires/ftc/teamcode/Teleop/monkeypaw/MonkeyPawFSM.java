package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ArmFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;

import java.util.concurrent.TimeUnit;

@Config
public class MonkeyPawFSM {


    public enum States {
        START, READY_TO_START,
        // Intake states
        PREPARING_TO_INTAKE_SAMPLE, PREPARED_TO_INTAKE_SAMPLE, INTAKING_SAMPLE, RELAXING_WITH_SAMPLE, RELAXED_POS_WITH_SAMPLE, RETRACTING_INTAKE, RETRACTED_INTAKE, DEPOSITING_SAMPLE_TO_HP, DEPOSITED_SAMPLE_TO_HP,

        //Sample States
        DEPOSITING_SAMPLE, RELAXING_AFTER_DEPOSIT, RELAXED_AFTER_DEPOSIT,

        //Specimen States
        PREPARING_TO_INTAKE_SPECIMEN, PREPARED_TO_INTAKE_SPECIMEN, INTAKING_SPECIMEN, INTAKED_SPECIMEN, GETTING_READY_TO_DEPOSIT_SPECIMEN, READY_TO_DEPOSIT_SPECIMEN, DEPOSITING_SPECIMEN, DEPOSITED_SPECIMEN
    }

    private final FingerFSM fingerFSM;
    private final DeviatorFSM deviatorFSM;
    private final WristFSM wristFSM;
    private final ElbowFSM elbowFSM;
    private final ArmFSM armFSM;

    private final LimbFSM limbFSM;
    private final Logger logger;
    private States state;
    private final Timing.Timer timer;
    private final Timing.Timer specimenTimer;
    public static long TIMER_LENGTH = 750;

    public static long SPECIMEN_TIMER_LENGTH = 500;
    private boolean dpadUp = false;
    private boolean dpadDown = false;
    private boolean grippedSpecimen = false;

    public MonkeyPawFSM(HWMap hwMap, Logger logger, LimbFSM limbFSM, ElbowFSM elbowFSM, DeviatorFSM deviatorFSM, WristFSM wristFSM, ArmFSM armFSM) {
        this.logger = logger;
        fingerFSM = new FingerFSM(hwMap, logger);
        this.deviatorFSM = deviatorFSM;
        this.elbowFSM = elbowFSM;
        this.wristFSM = wristFSM;
        this.limbFSM = limbFSM;
        this.armFSM = armFSM;
        timer = new Timing.Timer(TIMER_LENGTH, TimeUnit.MILLISECONDS);
        specimenTimer = new Timing.Timer(SPECIMEN_TIMER_LENGTH, TimeUnit.MILLISECONDS);
        state = States.START;
    }

    public void findTargetState(boolean xPressed) {
        if ((limbFSM.PREPARED_TO_INTAKE() || limbFSM.PREPARING_TO_INTAKE() || limbFSM.MOVING_TO_INTAKE_POS()) && (!PREPARED_TO_INTAKE_SAMPLE() && !RELAXING_WITH_SAMPLE() && !RELAXED_POS_WITH_SAMPLE() && !RETRACTING_INTAKE())) {
            state = States.PREPARING_TO_INTAKE_SAMPLE;
        } else if (limbFSM.MOVED_TO_INTAKE_POS() && (PREPARED_TO_INTAKE_SAMPLE()) && !RELAXING_WITH_SAMPLE()) {
            state = States.INTAKING_SAMPLE;
        } else if (RELAXED_POS_WITH_SAMPLE() && (limbFSM.RETRACTING_INTAKE() || limbFSM.RETRACTED_INTAKE()) && !RETRACTED_INTAKE()) {
            state = States.RETRACTING_INTAKE;
        } else if (limbFSM.EXTENDED_TO_BASKET_HEIGHT() && !RELAXING_AFTER_DEPOSIT() && !RELAXED_AFTER_DEPOSIT()) {
            state = States.DEPOSITING_SAMPLE;
        } else if (limbFSM.PREPARING_TO_INTAKE_SPECIMEN() && !PREPARED_TO_INTAKE_SPECIMEN() && !INTAKED_SPECIMEN()) {
            state = States.PREPARING_TO_INTAKE_SPECIMEN;
        } else if (limbFSM.INTAKING_SPECIMEN() && (PREPARED_TO_INTAKE_SPECIMEN() || limbFSM.STARTED()) && !INTAKED_SPECIMEN()) {
            state = States.INTAKING_SPECIMEN;
        } else if (limbFSM.EXTENDING_SPECIMEN() && INTAKED_SPECIMEN() && !READY_TO_DEPOSIT_SPECIMEN()) {
            state = States.GETTING_READY_TO_DEPOSIT_SPECIMEN;
        } else if (limbFSM.DEPOSITED_SPECIMEN() && READY_TO_DEPOSIT_SPECIMEN() && !DEPOSITED_SPECIMEN()) {
            state = States.DEPOSITING_SPECIMEN;
        }

        if (xPressed && (RELAXED_POS_WITH_SAMPLE() || PREPARED_TO_INTAKE_SAMPLE())) {
            state = States.PREPARING_TO_INTAKE_SAMPLE;
        }

    }

    public void updateState(boolean rightTrigger, boolean leftTrigger, boolean yPressed, boolean xPressed, boolean dpadUp, boolean dpadDown) {
        fingerFSM.updateState();
        wristFSM.updateState();
        deviatorFSM.updateState();
        elbowFSM.updateState();
        findTargetState(xPressed);
        this.dpadUp = dpadUp;
        this.dpadDown = dpadDown;
        switch (state) {
            // INTAKE STATES
            case START:
                if (elbowFSM.RELAXED()) {
                    if (wristFSM.RELAXED()) {
                        if (deviatorFSM.RELAXED()) {
                            if (fingerFSM.RELEASED()) {
                                state = States.READY_TO_START;
                            } else {
                                fingerFSM.releaseSample();
                            }
                        } else {
                            deviatorFSM.relax();
                        }
                    } else {
                        wristFSM.relax();
                    }
                } else {
                    elbowFSM.relax();
                }
                break;
            case PREPARING_TO_INTAKE_SAMPLE:
                if (rightTrigger) {
                    if (deviatorFSM.RELAXED() || deviatorFSM.RELAXING()) {
                        deviatorFSM.vertical();
                    } else {
                        deviatorFSM.relax();
                    }
                } else if (leftTrigger) {
                    deviatorFSM.indexIncrement();
                }

                if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_READY_POS() && wristFSM.FLEXED_TO_SAMPLE_INTAKE_READY_POS()) {
                    fingerFSM.releaseSample();
                    if (fingerFSM.RELEASED() && fingerFSM.isTargetAngleSampleRelease()) {
                        state = States.PREPARED_TO_INTAKE_SAMPLE;
                    }
                } else {
                    elbowFSM.flexToSampleIntakeReadyPos();
                    wristFSM.flexToSampleIntakeReadyPos();
                }
                break;
            case PREPARED_TO_INTAKE_SAMPLE:
                if (rightTrigger) {
                    if (deviatorFSM.RELAXED() || deviatorFSM.RELAXING()) {
                        deviatorFSM.vertical();
                    } else {
                        deviatorFSM.relax();
                    }
                } else if (leftTrigger) {
                    deviatorFSM.indexIncrement();
                }
                if (limbFSM.MOVING_TO_INTAKE_POS() || limbFSM.MOVED_TO_INTAKE_POS()) {
                    elbowFSM.flexToSampleHoveringPos();
                    if (dpadDown) {
                        elbowFSM.increaseHoverOffset();
                    } else if (dpadUp) {
                        elbowFSM.decreaseHoverOffset();
                    }
                }
                break;
            case INTAKING_SAMPLE:
                if (rightTrigger) {
                    if (deviatorFSM.indexCloserToRelaxation()) {
                        deviatorFSM.relax();
                    } else {
                        deviatorFSM.vertical();
                    }
                } else if (leftTrigger) {
                    deviatorFSM.indexIncrement();
                }
                wristFSM.setSampleCapture(true);
                elbowFSM.flexToSampleIntakeCapturePos();
                wristFSM.flexToSampleIntakeCapturePos();
                if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS() && wristFSM.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS()) {
                    fingerFSM.gripSample();
                    if (fingerFSM.GRIPPED()) {
                        if (!timer.isTimerOn()) {
                            timer.start();
                        }
                        if (timer.done()) {
                            timer.pause();
                            wristFSM.setSampleCapture(false);
                            state = States.RELAXING_WITH_SAMPLE;
                        }
                    }
                }

                break;
            case RELAXING_WITH_SAMPLE:
                elbowFSM.flexToSampleIntakeControlPos();
                wristFSM.flexToSampleIntakeControlPos();
                if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS() && wristFSM.FLEXED_TO_SAMPLE_INTAKE_CONTROL_POS()) {
                    state = States.RELAXED_POS_WITH_SAMPLE;
                }
                break;
            case RETRACTING_INTAKE:
                wristFSM.flexToSampleIntakeRetractPos();
                elbowFSM.flexToSampleIntakeRetractPos();
                deviatorFSM.relax();
                if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS() && wristFSM.FLEXED_TO_SAMPLE_INTAKE_RETRACT_POS() && deviatorFSM.RELAXED()) {
                    state = States.RETRACTED_INTAKE;
                }
                break;
            case DEPOSITING_SAMPLE_TO_HP:
                fingerFSM.releaseSample();
                if (fingerFSM.RELEASED()) {
                    state = States.DEPOSITED_SAMPLE_TO_HP;
                }
                break;

            //SAMPLE DEPOSIT IN BASKET STATES
            case DEPOSITING_SAMPLE:
                elbowFSM.flexToBasketDepositFlexedPos();
                wristFSM.flexToHighBasketPos();
                if (elbowFSM.FLEXED_TO_BASKET_DEPOSIT() && wristFSM.FLEXED_TO_HIGH_BASKET_DEPOSIT() && yPressed) {
                    fingerFSM.releaseSample();
                    if (fingerFSM.RELEASED()) {
                        state = States.RELAXED_AFTER_DEPOSIT;
                    }
                }
                break;
            //SPECIMEN DEPOSIT STATES
            case PREPARING_TO_INTAKE_SPECIMEN:
                elbowFSM.flexToSpecimenIntakePos();
                wristFSM.flexToSpecimenIntakePos();
                deviatorFSM.relax();
                fingerFSM.releaseSpecimen();
                if (elbowFSM.FLEXED_TO_SPECIMEN_INTAKE() && wristFSM.FLEXED_TO_SPECIMEN_READY_POS() && deviatorFSM.RELAXED() && fingerFSM.RELEASED()) {
                    state = States.PREPARED_TO_INTAKE_SPECIMEN;
                }
                break;
            case INTAKING_SPECIMEN:
                fingerFSM.gripSpecimen();
                if (fingerFSM.GRIPPED() && !grippedSpecimen) {
                    if (!specimenTimer.isTimerOn()) {
                        specimenTimer.start();
                    }
                    if (specimenTimer.done()) {
                        specimenTimer.pause();
                        wristFSM.flexToSpecimenRetractIntake();
                        grippedSpecimen = true;

                    }
                }
                if (wristFSM.SPECIMEN_INTAKE_RETRACTED()) {
                    grippedSpecimen = false;
                    state = States.INTAKED_SPECIMEN;
                }
                break;
            case GETTING_READY_TO_DEPOSIT_SPECIMEN:
                elbowFSM.flexToHighChamberDepositFlexedPos();
                wristFSM.flexToSpecimenDepositReadyPos();
                if (elbowFSM.FLEXED_TO_HIGH_CHAMBER_DEPOSIT() && wristFSM.FLEXED_TO_HIGH_CHAMBER_DEPOSIT())
                    state = States.READY_TO_DEPOSIT_SPECIMEN;
                break;
            case DEPOSITING_SPECIMEN:
                armFSM.moveToSubmersibleHeight();
                armFSM.moveToChamberLockHeight();
                fingerFSM.releaseSpecimen();
                if (fingerFSM.RELEASED() && armFSM.AT_CHAMBER_LOCK_HEIGHT() && armFSM.AT_SUBMERSIBLE_HEIGHT()) {
                    state = States.DEPOSITED_SPECIMEN;
                }
                break;
        }

    }


    public boolean PREPARED_TO_INTAKE_SAMPLE() {
        return state == States.PREPARED_TO_INTAKE_SAMPLE;
    }

    public boolean RETRACTED_INTAKE() {
        return state == States.RETRACTED_INTAKE;
    }

    public boolean INTAKING_SAMPLE() {
        return state == States.INTAKING_SAMPLE;
    }


    public boolean PREPARED_TO_INTAKE_SPECIMEN() {
        return state == States.PREPARED_TO_INTAKE_SPECIMEN;
    }

    public boolean INTAKED_SPECIMEN() {
        return state == States.INTAKED_SPECIMEN;
    }


    public boolean DEPOSITED_SPECIMEN() {
        return state == States.DEPOSITED_SPECIMEN;
    }

    public boolean RELAXING_WITH_SAMPLE() {
        return state == States.RELAXING_WITH_SAMPLE;
    }

    public boolean RELAXED_POS_WITH_SAMPLE() {
        return state == States.RELAXED_POS_WITH_SAMPLE;
    }

    public boolean RELAXING_AFTER_DEPOSIT() {
        return state == States.RELAXING_AFTER_DEPOSIT;
    }

    public boolean RELAXED_AFTER_DEPOSIT() {
        return state == States.RELAXED_AFTER_DEPOSIT;
    }


    public boolean READY_TO_DEPOSIT_SPECIMEN() {
        return state == States.READY_TO_DEPOSIT_SPECIMEN;
    }


    public boolean RETRACTING_INTAKE() {
        return state == States.RETRACTING_INTAKE;
    }


    @VisibleForTesting
    public void setState(States state) {
        this.state = state;
    }

    public void log() {
        logger.log("---------------------MONKEY PAW LOG-------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Monkey Paw State: ", state, Logger.LogLevels.PRODUCTION);
        logger.log("Sample Timer: ", timer.elapsedTime(), Logger.LogLevels.DEBUG);
        logger.log("Specimen Timer: ", specimenTimer.elapsedTime(), Logger.LogLevels.DEBUG);
        logger.log("---------------------MONKEY PAW LOG-------------------", "-", Logger.LogLevels.PRODUCTION);
        elbowFSM.log();
        wristFSM.log();
        deviatorFSM.log();
        fingerFSM.log();
    }

    public void updatePID() {
        wristFSM.updatePID();
    }

}
