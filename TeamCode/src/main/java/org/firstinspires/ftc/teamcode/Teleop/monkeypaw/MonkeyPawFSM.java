package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;

@Config
public class MonkeyPawFSM {
    public enum States {

        // Intake states
        PREPARING_TO_INTAKE_SAMPLE,
        PREPARED_TO_INTAKE_SAMPLE,
        INTAKING_SAMPLE,
        RELAXING_WITH_SAMPLE,
        RELAXED_POS_WITH_SAMPLE,
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
    private boolean keepRelaxed;


    public MonkeyPawFSM(HWMap hwMap, Logger logger, LimbFSM limbFSM) {
        this.logger = logger;
        fingerFSM = new FingerFSM(hwMap, logger);
        deviatorFSM = new DeviatorFSM(hwMap, logger);
        wristFSM = new WristFSM(hwMap, logger);
        elbowFSM = new ElbowFSM(hwMap, logger);
        this.limbFSM = limbFSM;
        state = States.PREPARING_TO_INTAKE_SAMPLE;
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


    public void updateState(boolean rbPressed2,boolean xPressed2,boolean bPressed2, boolean aPressed, boolean dPadUpPressed , boolean yPressed, boolean dPadDownPressed, boolean dePadRightPressed) {
        fingerFSM.updateState();
        wristFSM.updateState();
        deviatorFSM.updateState();
        elbowFSM.updateState();
        findTargetState(rbPressed2, dPadUpPressed, yPressed,dPadDownPressed, dePadRightPressed);
        switch (state) {
            // INTAKE STATES
            case PREPARING_TO_INTAKE_SAMPLE:

               if(elbowFSM.RELAXED()) {
                   logger.log("Elbow Is Relaxed",elbowFSM.RELAXED(), Logger.LogLevels.PRODUCTION);
                    if(wristFSM.RELAXED()) {

                        logger.log("Wrist Is Relaxed",wristFSM.RELAXED(), Logger.LogLevels.PRODUCTION);
                        if(deviatorFSM.RELAXED()) {

                            logger.log("deviator Is Relaxed",deviatorFSM.RELAXED(), Logger.LogLevels.PRODUCTION);
                            if(fingerFSM.RELEASED()) {
                                logger.log("Finger is Released",fingerFSM.RELEASED(), Logger.LogLevels.PRODUCTION);
                                state = States.PREPARED_TO_INTAKE_SAMPLE;
                            }
                            else {
                                fingerFSM.releaseSample();
                            }
                        }
                        else {
                            deviatorFSM.relax();
                        }
                    }
                    else {
                        wristFSM.relax();
                    }
                }
                else {
                    elbowFSM.relax();
                }
                break;
            case INTAKING_SAMPLE:
               if(xPressed2) {
                    deviatorFSM.deviateLeft();
                }
                else if(bPressed2) {
                    deviatorFSM.deviateRight();
                }
                else if ((deviatorFSM.RIGHT_DEVIATED() || deviatorFSM.LEFT_DEVIATED()) && (aPressed && !deviatorFSM.RELAXED())) {
                    deviatorFSM.relax();
               }
                if(aPressed) {
                    keepRelaxed = true;
                }
                else if(deviatorFSM.RIGHT_DEVIATED() || deviatorFSM.LEFT_DEVIATED() || (keepRelaxed)) {
                    elbowFSM.flexToSamplePos();
                    wristFSM.flex();
                    if(elbowFSM.FLEXED_TO_SAMPLE_INTAKE() && wristFSM.FLEXED()) {
                        if(rbPressed2) {
                            fingerFSM.gripSample();
                        }
                            if(fingerFSM.GRIPPED()) {
                                state = States.RELAXING_WITH_SAMPLE;
                            }


                    }
                }
                break;
            case RELAXING_WITH_SAMPLE:
                elbowFSM.relax();
                wristFSM.relax();
                deviatorFSM.relax();
                if(elbowFSM.RELAXED() && wristFSM.RELAXED() && deviatorFSM.RELAXED()) {
                    state = States.RELAXED_POS_WITH_SAMPLE;
                }
                break;
            case DEPOSITING_SAMPLE_TO_HP:
                fingerFSM.releaseSample();
                if(fingerFSM.RELEASED()) {
                    state = States.DEPOSITED_SAMPLE_TO_HP;
                }
                break;

                //SAMPLE DEPOSIT IN BASKET STATES
            case DEPOSITING_SAMPLE:
                elbowFSM.flexToDepositPos();
                if(elbowFSM.FLEXED_TO_DEPOSIT()) {
                    fingerFSM.releaseSample();
                    if(fingerFSM.RELEASED()) {
                        state = States.RELAXING_AFTER_DEPOSIT;
                    }
                }
                break;
            case RELAXING_AFTER_DEPOSIT:
                elbowFSM.relax();
                if(elbowFSM.RELAXED()) {
                    state = States.RELAXED_AFTER_DEPOSIT;
                }
                break;

            //SPECIMEN DEPOSIT STATES
            case PREPARING_TO_INTAKE_SPECIMEN:
                elbowFSM.flexToSpecimenIntakePos();
                if(elbowFSM.FLEXED_TO_SPECIMEN_INTAKE()) {
                    state = States.PREPARED_TO_INTAKE_SPECIMEN;
                }
                break;
            case INTAKING_SPECIMEN:
                fingerFSM.gripSpecimen();
                elbowFSM.relaxToSpecimenIntakeRelaxedPos();
                if(fingerFSM.GRIPPED() && elbowFSM.RELAXED_TO_SPECIMEN_INTAKE_RELAX_POS()) {
                    state = States.INTAKED_SPECIMEN;
                }
                break;
            case DEPOSITING_SPECIMEN:
                elbowFSM.flexToDepositPos();
                if(elbowFSM.FLEXED_TO_DEPOSIT()) {
                    fingerFSM.releaseSpecimen();
                }
                if(fingerFSM.RELEASED()) {
                    elbowFSM.relax();
                    if(elbowFSM.RELAXED()) {
                        state = States.DEPOSITED_SPECIMEN;
                    }
                }
                break;


            //MINI INTAKING STATES

            case MINI_INTAKING:
                elbowFSM.flexToSamplePos();
                wristFSM.flex();
                if(elbowFSM.FLEXED_TO_SAMPLE_INTAKE() && wristFSM.FLEXED()) {
                    state = States.MINI_INTAKED;
                }
                break;
            case MINI_INTAKED:
                if(xPressed2) {
                    deviatorFSM.deviateLeft();
                }
                else if (bPressed2) {
                    deviatorFSM.deviateRight();
                }
                else if(aPressed) {
                    deviatorFSM.relax();
                }
                break;
            case RELAXING_MINI_INTAKE:
                if(rbPressed2) {
                    fingerFSM.gripSample();
                }
                if(fingerFSM.GRIPPED()) {
                    elbowFSM.relax();
                    wristFSM.relax();
                }
                if(elbowFSM.RELAXED() && wristFSM.RELAXED()) {
                    state = States.RELAXED_MINI_INTAKE;
                }
                break;
        }
    }

    public void findTargetState(boolean rbPressed2, boolean dpadUpPressed, boolean yPressed, boolean dpadDownPressed, boolean dpadRightPressed) {
        if(limbFSM.PREPARING_TO_INTAKE()) {
            state = States.PREPARING_TO_INTAKE_SAMPLE;
        }
        //else if(limbFSM.MOVED_TO_INTAKE_POS() && PREPARED_TO_INTAKE_SAMPLE()) {
        else if(dpadDownPressed) {
            state = States.INTAKING_SAMPLE;
        }
       // }
        else if(rbPressed2 && !INTAKING_SAMPLE() && !RELAXING_MINI_INTAKE()) {
            state = States.DEPOSITING_SAMPLE_TO_HP;
        }
       // else if(limbFSM.PREPARING_TO_INTAKE_SPECIMEN()) {
        else if(yPressed && !PREPARED_TO_INTAKE_SPECIMEN() && !INTAKED_SPECIMEN()) {
            state = States.PREPARING_TO_INTAKE_SPECIMEN;
        }
        //}
        //else if(limbFSM.INTAKING_SPECIMEN()) {
        else if (yPressed && PREPARED_TO_INTAKE_SPECIMEN() && !INTAKED_SPECIMEN()) {
            state = States.INTAKING_SPECIMEN;
        }
        //}
        //else if(limbFSM.DEPOSITING_SAMPLE()) {
        else if(dpadUpPressed) {
            state = States.DEPOSITING_SAMPLE;
        }
        //}
        //else if(limbFSM.DEPOSITED_SPECIMEN()) {
        else if (yPressed && INTAKED_SPECIMEN()) {
            state = States.DEPOSITING_SPECIMEN;
        }
        //}
        //else if(limbFSM.MOVED_TO_MINI_INTAKE()) {
        else if(dpadRightPressed && !MINI_INTAKED()) {
            state = States.MINI_INTAKING;
        }
        //}
        //else if(limbFSM.RETRACTING_FROM_MINI_INTAKE()) {
        else if (dpadRightPressed && MINI_INTAKED()) {
            state = States.RELAXING_MINI_INTAKE;
        }
        //}
    }
    public boolean PREPARING_TO_INTAKE_SAMPLE() {
        return state == States.PREPARING_TO_INTAKE_SAMPLE;
    }

    public boolean PREPARED_TO_INTAKE_SAMPLE() {
        return state == States.PREPARED_TO_INTAKE_SAMPLE;
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

    @VisibleForTesting
    public void setState(States state) {
        this.state = state;
    }

    public void log() {
        logger.log("Monkey Paw State", state, Logger.LogLevels.PRODUCTION);
        elbowFSM.log();
        wristFSM.log();
        deviatorFSM.log();
        fingerFSM.log();
    }

    public void updatePID() {
        deviatorFSM.updatePID();
        elbowFSM.updatePID();
        wristFSM.updatePID();
    }

}
