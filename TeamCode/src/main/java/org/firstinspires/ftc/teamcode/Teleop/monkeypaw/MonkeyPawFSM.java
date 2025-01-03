package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;

import java.util.concurrent.TimeUnit;

@Config
public class MonkeyPawFSM {


    public enum States {
        START,
        READY_TO_START,
        // Intake states
        PREPARING_TO_INTAKE_SAMPLE,
        PREPARED_TO_INTAKE_SAMPLE,
        INTAKING_SAMPLE,
        RELAXING_WITH_SAMPLE,
        RELAXED_POS_WITH_SAMPLE,
        RETRACTING_INTAKE,
        RETRACTED_INTAKE,
        DEPOSITING_SAMPLE_TO_HP,
        DEPOSITED_SAMPLE_TO_HP,
        MINI_INTAKING,
        MINI_INTAKED,
        RELAXING_MINI_INTAKE,
        RELAXED_MINI_INTAKE,

        //Sample States
        DEPOSITING_SAMPLE,
        RELAXING_AFTER_DEPOSIT,
        RELAXED_AFTER_DEPOSIT,

        //Specimen States
        PREPARING_TO_INTAKE_SPECIMEN,
        PREPARED_TO_INTAKE_SPECIMEN,
        INTAKING_SPECIMEN,
        INTAKED_SPECIMEN,
        RELAXING_FROM_SPECIMEN_INTAKE,
        DEPOSITING_SPECIMEN,
        DEPOSITED_SPECIMEN
    }

    private FingerFSM fingerFSM;
    private DeviatorFSM deviatorFSM;
    private WristFSM wristFSM;
    private ElbowFSM elbowFSM;

    private LimbFSM limbFSM;
    private Logger logger;
    private States state;
    private Timing.Timer timer;
    public static long TIMER_LENGTH = 500;


    public MonkeyPawFSM(HWMap hwMap, Logger logger, LimbFSM limbFSM, ElbowFSM elbowFSM, DeviatorFSM deviatorFSM, WristFSM wristFSM) {
        this.logger = logger;
        fingerFSM = new FingerFSM(hwMap, logger);
        this.deviatorFSM = deviatorFSM;
        this.elbowFSM = elbowFSM;
        this.wristFSM = wristFSM;
        this.limbFSM = limbFSM;
        timer = new Timing.Timer(TIMER_LENGTH, TimeUnit.MILLISECONDS);
        state = States.START;
    }

    @VisibleForTesting
    public MonkeyPawFSM(Logger logger, LimbFSM limbFSM, FingerFSM fingerFSM, DeviatorFSM deviatorFSM, WristFSM wristFSM, ElbowFSM elbowFSM) {
        this.logger = logger;
        this.fingerFSM = fingerFSM;
        this.deviatorFSM = deviatorFSM;
        this.wristFSM = wristFSM;
        this.elbowFSM = elbowFSM;
        this.limbFSM = limbFSM;
    }


    public void updateState(boolean rbPressed2, boolean rightTrigger, boolean leftTrigger, boolean dePadRightPressed, boolean aPressed1, boolean yPressed, boolean xPressed) {
        fingerFSM.updateState();
        wristFSM.updateState();
        deviatorFSM.updateState();
        elbowFSM.updateState();
        findTargetState(rbPressed2, dePadRightPressed, aPressed1, xPressed);
        switch (state) {
            // INTAKE STATES
            case START:
                if (elbowFSM.RELAXED()) {
                    logger.log("Elbow Is Relaxed", elbowFSM.RELAXED(), Logger.LogLevels.PRODUCTION);
                    if (wristFSM.RELAXED()) {

                        logger.log("Wrist Is Relaxed", wristFSM.RELAXED(), Logger.LogLevels.PRODUCTION);
                        if (deviatorFSM.RELAXED()) {

                            logger.log("deviator Is Relaxed", deviatorFSM.RELAXED(), Logger.LogLevels.PRODUCTION);
                            if (fingerFSM.RELEASED()) {
                                logger.log("Finger is Released", fingerFSM.RELEASED(), Logger.LogLevels.PRODUCTION);
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
                    }
                    else {
                        deviatorFSM.relax();
                    }
                } else if (leftTrigger) {
                        deviatorFSM.indexIncrement();
                }

                if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_READY_POS() && wristFSM.FLEXED_TO_SAMPLE_INTAKE_READY_POS()) {
                    if (fingerFSM.RELEASED()) {
                        state = States.PREPARED_TO_INTAKE_SAMPLE;
                    } else {
                        fingerFSM.releaseSample();
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
                    }
                    else {
                        deviatorFSM.relax();
                    }
                } else if (leftTrigger) {
                    deviatorFSM.indexIncrement();
                }
                if(limbFSM.MOVING_TO_INTAKE_POS() || limbFSM.MOVED_TO_INTAKE_POS()) {
                    elbowFSM.flexToSampleHoveringPos();
                }
                break;
            case INTAKING_SAMPLE:
                if (rightTrigger) {
                    if (deviatorFSM.indexCloserToRelaxation()) {
                        deviatorFSM.relax();
                    }
                    else {
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
                if (elbowFSM.FLEXED_TO_SPECIMEN_INTAKE() && wristFSM.FLEXED_TO_SPECIMEN_READY_POS()) {
                    state = States.PREPARED_TO_INTAKE_SPECIMEN;
                }
                break;
            case INTAKING_SPECIMEN:
                fingerFSM.gripSpecimen();
                // elbowFSM.relaxToSpecimenIntakeRelaxedPos();
                if (fingerFSM.GRIPPED()) {
                    state = States.INTAKED_SPECIMEN;
                }
                break;
            case DEPOSITING_SPECIMEN:
                elbowFSM.flexToHighChamberDepositFlexedPos();
                if (elbowFSM.FLEXED_TO_HIGH_CHAMBER_DEPOSIT()) {
                    fingerFSM.releaseSpecimen();
                }
                if (fingerFSM.RELEASED()) {
                    elbowFSM.relax();
                    if (elbowFSM.RELAXED()) {
                        state = States.DEPOSITED_SPECIMEN;
                    }
                }
                break;


            //MINI INTAKING STATES

            case MINI_INTAKING:
                elbowFSM.flexToSampleIntakeReadyPos();
                wristFSM.flex();
                if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_READY_POS() && wristFSM.FLEXED()) {
                    state = States.MINI_INTAKED;
                }
                break;
            case MINI_INTAKED:
                if (rightTrigger) {
                    if (deviatorFSM.RELAXED() || deviatorFSM.RELAXING()) {
                        deviatorFSM.vertical();
                    }
                    else {
                        deviatorFSM.relax();
                    }
                } else if (leftTrigger) {
                        deviatorFSM.indexIncrement();
                }
                break;
            case RELAXING_MINI_INTAKE:
                if (rbPressed2) {
                    elbowFSM.flexToSampleIntakeCapturePos();
                    if (elbowFSM.FLEXED_TO_SAMPLE_INTAKE_CAPTURE_POS()) {
                        fingerFSM.gripSample();
                    }
                }
                if (fingerFSM.GRIPPED()) {
                    elbowFSM.relax();
                    wristFSM.relax();
                }
                if (elbowFSM.RELAXED() && wristFSM.RELAXED()) {
                    state = States.RELAXED_MINI_INTAKE;
                }
                break;
        }

    }

    public void findTargetState(boolean rbPressed2, boolean dpadRightPressed, boolean aPressed1, boolean xPressed) {
        if ((limbFSM.PREPARED_TO_INTAKE() || limbFSM.PREPARING_TO_INTAKE()) && (!PREPARED_TO_INTAKE_SAMPLE() && !RELAXING_WITH_SAMPLE() && !RELAXED_POS_WITH_SAMPLE() && !RETRACTING_INTAKE())) {
            state = States.PREPARING_TO_INTAKE_SAMPLE;
        } else if (limbFSM.MOVED_TO_INTAKE_POS() && (PREPARED_TO_INTAKE_SAMPLE() || INTAKING_SAMPLE())) {
            state = States.INTAKING_SAMPLE;
        } else if (RELAXED_POS_WITH_SAMPLE() && (limbFSM.RETRACTING_INTAKE() || limbFSM.RETRACTED_INTAKE())) {
            state = States.RETRACTING_INTAKE;
        } else if (rbPressed2 && !INTAKING_SAMPLE() && !RELAXING_MINI_INTAKE()) {
            state = States.DEPOSITING_SAMPLE_TO_HP;
        } else if (limbFSM.EXTENDED_TO_BASKET_HEIGHT() && !RELAXING_AFTER_DEPOSIT() && !RELAXED_AFTER_DEPOSIT()) {
            state = States.DEPOSITING_SAMPLE;
        }

        //TODO: Specimens later on.
/*
        // else if(limbFSM.PREPARING_TO_INTAKE_SPECIMEN()) {
        else if (yPressed && !PREPARED_TO_INTAKE_SPECIMEN() && !INTAKED_SPECIMEN()) {
            state = States.PREPARING_TO_INTAKE_SPECIMEN;
        }
        //}
        //else if(limbFSM.INTAKING_SPECIMEN()) {
        else if (yPressed && PREPARED_TO_INTAKE_SPECIMEN() && !INTAKED_SPECIMEN()) {
            state = States.INTAKING_SPECIMEN;
        }
        //}
        //else if(limbFSM.DEPOSITED_SPECIMEN()) {
        else if (yPressed && INTAKED_SPECIMEN()) {
            state = States.DEPOSITING_SPECIMEN;
        }
        //}*/
        if(xPressed && RELAXED_POS_WITH_SAMPLE()) {
            state = States.PREPARING_TO_INTAKE_SAMPLE;
        }
        if(xPressed && RELAXED_POS_WITH_SAMPLE()) {
            state = States.PREPARING_TO_INTAKE_SAMPLE;
        }

    }

    public boolean PREPARING_TO_INTAKE_SAMPLE() {
        return state == States.PREPARING_TO_INTAKE_SAMPLE;
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

    public boolean DEPOSITED_SAMPLE_TO_HP() {
        return state == States.DEPOSITED_SAMPLE_TO_HP;
    }

    public boolean DEPOSITING_SAMPLE_TO_HP() {
        return state == States.DEPOSITING_SAMPLE_TO_HP;
    }

    public boolean PREPARING_TO_INTAKE_SPECIMEN() {
        return state == States.PREPARING_TO_INTAKE_SPECIMEN;
    }

    public boolean PREPARED_TO_INTAKE_SPECIMEN() {
        return state == States.PREPARED_TO_INTAKE_SPECIMEN;
    }

    public boolean INTAKING_SPECIMEN() {
        return state == States.INTAKING_SPECIMEN;
    }

    public boolean INTAKED_SPECIMEN() {
        return state == States.INTAKED_SPECIMEN;
    }

    public boolean DEPOSITING_SAMPLE() {
        return state == States.DEPOSITING_SAMPLE;
    }

    public boolean DEPOSITING_SPECIMEN() {
        return state == States.DEPOSITING_SPECIMEN;
    }

    public boolean DEPOSITED_SPECIMEN() {
        return state == States.DEPOSITED_SPECIMEN;
    }

    public boolean MINI_INTAKING() {
        return state == States.MINI_INTAKING;
    }

    public boolean MINI_INTAKED() {
        return state == States.MINI_INTAKED;
    }

    public boolean RELAXING_MINI_INTAKE() {
        return state == States.RELAXING_MINI_INTAKE;
    }

    public boolean RELAXED_MINI_INTAKE() {
        return state == States.RELAXED_MINI_INTAKE;
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


    public boolean RETRACTING_INTAKE() {
        return state == States.RETRACTING_INTAKE;
    }


    @VisibleForTesting
    public void setState(States state) {
        this.state = state;
    }

    public void log() {
        logger.log("Paw State: ", state, Logger.LogLevels.PRODUCTION);
        elbowFSM.log();
        wristFSM.log();
        deviatorFSM.log();
        fingerFSM.log();
    }

    public void updatePID() {
        //deviatorFSM.updatePID();
        // elbowFSM.updatePID();
        wristFSM.updatePID();
    }

    public double getElbowCurrentAngle() {
        return elbowFSM.getElbowCurrentAngle();
    }

    public double getElbowReadyPosAngle() {
        return getElbowReadyPosAngle();
    }

}
