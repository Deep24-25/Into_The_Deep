package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;

public class LimbFSM {
    public enum States {
        PREPARING_TO_INTAKE_SPECIMEN, PREPARED_TO_INTAKE_SPECIMEN, INTAKING_SPECIMEN, INTAKED_SPECIMEN, EXTENDING_SPECIMEN, EXTENDED_SPECIMEN, DEPOSITING_SPECIMEN, DEPOSITED_SPECIMEN, PREPARING_TO_DEPOSIT_SPECIMEN, PREPARED_TO_DEPOSIT_SPECIMEN, PREPARING_TO_DEPOSIT_SAMPLE, PREPARED_TO_DEPOSIT_SAMPLE, EXTENDING_TO_BASKET_HEIGHT, EXTENDED_TO_BASKET_HEIGHT, DEPOSITING_SAMPLE, DEPOSITED_SAMPLE, PREPARING_TO_INTAKE, PREPARED_TO_INTAKE, MOVING_TO_INTAKE_POS, MOVED_TO_INTAKE_POS, MOVING_TO_MINI_INTAKE, MOVED_TO_MINI_INTAKE, RETRACTING_FROM_MINI_INTAKE, RETRACTED_FROM_MINI_INTAKE, RETRACTING_INTAKE, RETRACTED_INTAKE
    }

    public enum Mode {
        SAMPLE_MODE, SPECIMEN_MODE
    }


    /*SPECIMEN STATES
    PREPARING_TO_INTAKE_SPECIMEN
    PREPARED_TO_INTAKE_SPECIMEN
    INTAKING_SPECIMEN
    INTAKED_SPECIMEN
    EXTENDING_SPECIMEN
    EXTENDED_SPECIMEN
    DEPOSITING_SPECIMEN
    DEPOSITED_SPECIMEN
     */

    /* SAMPLE STATES
    PREPARING_TO_DEPOSIT_SAMPLE
    PREPARED_TO_DEPOSIT_SAMPLE,
    EXTENDING_TO_BASKET_HEIGHT
    EXTENDED_TO_BASKET_HEIGHT
    DEPOSITING_SAMPLE
    DEPOSITED_SAMPLE
     */

    /* INTAKE STATES
    PREPARING_TO_INTAKE
    PREPARED_TO_INTAKE
    MOVING_TO_INTAKE_POS
    MOVED_TO_INTAKE_POS
    MOVING_TO_MINI_INTAKE
    MOVED_TO_MINI_INTAKE
    RETRACTING_FROM_MINI_INTAKE
    RETRACTED_FROM_MINI_INTAKE
     */


    private HWMap hwMap;
    private ArmFSM armFSM;
    private ShoulderFSM shoulderFSM;
    private volatile MonkeyPawFSM monkeyPawFSM;

    private States states = States.INTAKED_SPECIMEN;
    private Mode mode = Mode.SPECIMEN_MODE;
    private Logger logger;

    private boolean notBeenInMovingToIntake = true;

    public LimbFSM(ShoulderFSM shoulderFSM, ArmFSM armFSM, MonkeyPawFSM monkeyPawFSM, Logger logger) {
        this.logger = logger;
        this.armFSM = armFSM;
        this.shoulderFSM = shoulderFSM;
        this.monkeyPawFSM = monkeyPawFSM;
    }

    public MonkeyPawFSM getMonkeyPawFSM() {
        if (monkeyPawFSM == null) {
            synchronized (this) {
                if (monkeyPawFSM == null) {
                    monkeyPawFSM = new MonkeyPawFSM(hwMap, logger, this);
                }
            }
        }
        return monkeyPawFSM;
    }


    public LimbFSM(ArmFSM armFSM, ShoulderFSM shoulderFSM, MonkeyPawFSM monkeyPawFSM) {
        this.armFSM = armFSM;
        this.shoulderFSM = shoulderFSM;
        this.monkeyPawFSM = monkeyPawFSM;
    }

    public void findTargetState(boolean yPressed, boolean aPressed, boolean xPressed, boolean rightBumperPressed, boolean rightTriggerPressed, boolean leftBumperPressed, boolean leftTriggerPressed) {
       /* if (yPressed && SPECIMEN_MODE()) {
            if ((DEPOSITED_SPECIMEN() || SAMPLE_STATES()) && !PREPARING_TO_INTAKE()) {
                states = States.PREPARING_TO_INTAKE;
            } else if (PREPARED_TO_INTAKE()) {
                states = States.INTAKING_SPECIMEN;
            } else if (INTAKED_SPECIMEN()) {
                states = States.EXTENDING_SPECIMEN;
            } else if (EXTENDED_SPECIMEN()) {
                states = States.DEPOSITING_SPECIMEN;
            }
        } else*/
        if (yPressed && SAMPLE_MODE()) {
            if ((!PREPARED_TO_DEPOSIT_SAMPLE() && !DEPOSITING_SAMPLE() && !EXTENDED_TO_BASKET_HEIGHT() && !EXTENDING_TO_BASKET_HEIGHT()) || DEPOSITED_SAMPLE() || SPECIMEN_STATES()) {
                states = States.PREPARING_TO_DEPOSIT_SAMPLE;
            } else if (PREPARED_TO_DEPOSIT_SAMPLE()) {
                states = States.EXTENDING_TO_BASKET_HEIGHT;
            } else if (EXTENDED_TO_BASKET_HEIGHT()) {
                states = States.DEPOSITING_SAMPLE;
            }
        }
        /*if (xPressed && SPECIMEN_MODE()) {
            if (INTAKING_SPECIMEN() || INTAKED_SPECIMEN()) {
                states = States.PREPARING_TO_INTAKE;
            } else if (EXTENDING_SPECIMEN() || EXTENDED_SPECIMEN())
                states = States.INTAKING_SPECIMEN;
            else if (DEPOSITING_SPECIMEN()) {
                states = States.EXTENDING_SPECIMEN;
            }
        }
        */
        if (aPressed) {
            if ((!MOVING_TO_INTAKE_POS() || DEPOSITED_SAMPLE() || DEPOSITED_SPECIMEN() || RETRACTED_INTAKE()) && !PREPARED_TO_INTAKE() && !MOVED_TO_INTAKE_POS()) {
                states = States.PREPARING_TO_INTAKE;
            } else if (PREPARED_TO_INTAKE() || RETRACTED_INTAKE()) {
                notBeenInMovingToIntake = true;
                states = States.MOVING_TO_INTAKE_POS;
            } else if (MOVING_TO_INTAKE_POS() && !notBeenInMovingToIntake) {
                states = States.MOVED_TO_INTAKE_POS;
            } else if (MOVED_TO_INTAKE_POS()) {
                states = States.RETRACTING_INTAKE;
            }
        }
        if (rightBumperPressed) {
            if (PREPARED_TO_INTAKE()) {
                states = States.MOVING_TO_MINI_INTAKE;
            } else if (MOVED_TO_MINI_INTAKE() && monkeyPawFSM.MINI_INTAKED()) {
                states = States.RETRACTING_FROM_MINI_INTAKE;
            }
        }
        if (leftBumperPressed) {
            if (SPECIMEN_MODE()) {
                mode = Mode.SAMPLE_MODE;
            } else {
                mode = Mode.SPECIMEN_MODE;
            }
        }
    }

    public void updateState(boolean yPressed, boolean aPressed, boolean xPressed, boolean rightBumperPressed, boolean rightTriggerPressed, boolean leftBumperPressed, boolean leftTriggerPressed, boolean test) {
        updateLowLevelFSMStates();
        if (!test)
            findTargetState(yPressed, aPressed, xPressed, rightBumperPressed, rightTriggerPressed, leftBumperPressed, leftTriggerPressed);
        switch (states) {
            case PREPARING_TO_INTAKE_SPECIMEN:
                if (armFSM.FULLY_RETRACTED()) {
                    shoulderFSM.moveToIntakeAngle();
                    if (shoulderFSM.AT_INTAKE() && monkeyPawFSM.PREPARED_TO_INTAKE_SPECIMEN()) {
                        states = States.PREPARED_TO_INTAKE_SPECIMEN;
                    }
                } else {
                    armFSM.retract();
                }
                break;
            case INTAKING_SPECIMEN:
                if (monkeyPawFSM.INTAKED_SPECIMEN()) {
                    armFSM.retract();
                    if (armFSM.FULLY_RETRACTED()) {
                        states = States.INTAKED_SPECIMEN;
                    }
                }
                break;
            case EXTENDING_SPECIMEN:
                shoulderFSM.setChamberTargetAngle();
                if (leftTriggerPressed) {
                    shoulderFSM.indexToHighChamberAngle();
                } else if (rightTriggerPressed) {
                    shoulderFSM.indexToLowChamberAngle();
                }
                if (shoulderFSM.AT_DEPOSIT_CHAMBERS()) {
                    armFSM.moveToSubmersibleHeight();
                    if (shoulderFSM.isChamberAngleLow()) {
                        armFSM.setIndexToSubmersibleLowHeight();
                    } else if (shoulderFSM.isChamberAngleHigh()) {
                        armFSM.setIndexToSubmersibleHighHeight();
                    }
                }
                if (armFSM.AT_SUBMERSIBLE_HEIGHT() && shoulderFSM.AT_DEPOSIT_CHAMBERS()) {
                    states = States.EXTENDED_SPECIMEN;
                }
                break;
            case EXTENDED_SPECIMEN:
                shoulderFSM.setChamberTargetAngle();
                if (leftTriggerPressed) {
                    shoulderFSM.indexToHighChamberAngle();
                } else if (rightTriggerPressed) {
                    shoulderFSM.indexToLowChamberAngle();
                }
                if (shoulderFSM.AT_DEPOSIT_CHAMBERS()) {
                    armFSM.moveToSubmersibleHeight();
                    if (shoulderFSM.isChamberAngleLow()) {
                        armFSM.setIndexToSubmersibleLowHeight();
                    } else if (shoulderFSM.isChamberAngleHigh()) {
                        armFSM.setIndexToSubmersibleHighHeight();
                    }
                }
                if (armFSM.AT_SUBMERSIBLE_HEIGHT() && shoulderFSM.AT_DEPOSIT_CHAMBERS()) {
                    states = States.EXTENDED_SPECIMEN;
                } else {
                    states = States.EXTENDING_SPECIMEN;
                }
                break;
            case DEPOSITING_SPECIMEN:
                armFSM.moveToChamberLockHeight();
                if (armFSM.AT_CHAMBER_LOCK_HEIGHT()) {
                    if (monkeyPawFSM.DEPOSITED_SPECIMEN()) {
                        states = States.DEPOSITED_SPECIMEN;
                    }
                }
                break;
            case PREPARING_TO_DEPOSIT_SAMPLE:
                if (armFSM.FULLY_RETRACTED()) {
                    shoulderFSM.setBasketTargetAngle();
                    if (shoulderFSM.AT_BASKET_DEPOSIT()) {
                        states = States.PREPARED_TO_DEPOSIT_SAMPLE;
                    }
                } else {
                    armFSM.retract();
                }
                break;
            case EXTENDING_TO_BASKET_HEIGHT:
                shoulderFSM.setBasketTargetAngle();
                armFSM.goToBasketHeight();
                if (leftTriggerPressed) {
                    armFSM.setIndexToBasketHighHeight();
                } else if (rightTriggerPressed) {
                    armFSM.setIndexToBasketLowHeight();
                }
                if (armFSM.AT_BASKET_HEIGHT() || shoulderFSM.AT_BASKET_DEPOSIT()) {
                    states = States.EXTENDED_TO_BASKET_HEIGHT;
                }
                break;
            case EXTENDED_TO_BASKET_HEIGHT:
                armFSM.goToBasketHeight();
                if (rightTriggerPressed) {
                    armFSM.setIndexToBasketLowHeight();
                } else if (leftTriggerPressed) {
                    armFSM.setIndexToBasketHighHeight();
                }
                if(!armFSM.AT_BASKET_HEIGHT()){
                    states = States.EXTENDING_TO_BASKET_HEIGHT;
                }
                break;
            case DEPOSITING_SAMPLE:
                if (monkeyPawFSM.RELAXED_AFTER_DEPOSIT()) {
                    states = States.DEPOSITED_SAMPLE;
                }
                break;
            case PREPARING_TO_INTAKE:
                if (shoulderFSM.AT_INTAKE()) {
                    if (!armFSM.FULLY_RETRACTED()) {
                        armFSM.moveToSafeHeight();
                        if (monkeyPawFSM.PREPARED_TO_INTAKE_SAMPLE() || monkeyPawFSM.RETRACTED_INTAKE()) {
                            armFSM.retract();
                            if (armFSM.FULLY_RETRACTED()) {
                                states = States.PREPARED_TO_INTAKE;
                            }
                        }
                    } else {
                        states = States.PREPARED_TO_INTAKE;
                    }
                } else {
                    armFSM.retract();
                    if (armFSM.FULLY_RETRACTED()) {
                        shoulderFSM.moveToIntakeAngle();
                        if (shoulderFSM.AT_INTAKE()) {
                            if (monkeyPawFSM.PREPARED_TO_INTAKE_SAMPLE() || monkeyPawFSM.RETRACTED_INTAKE()) {
                                states = States.PREPARED_TO_INTAKE;
                            }
                        }
                    }
                }
                break;
            case MOVING_TO_INTAKE_POS:
                armFSM.moveToSelectedIndexPosition();
                notBeenInMovingToIntake = false;
                if (leftTriggerPressed) {
                    armFSM.indexIncrement();

                } else if (rightTriggerPressed) {
                    armFSM.indexDecrement();
                }
                break;
            case MOVED_TO_INTAKE_POS:
                if (xPressed) {
                    states = States.MOVING_TO_INTAKE_POS;
                }
                break;
            case RETRACTING_INTAKE:
                if (!armFSM.FULLY_RETRACTED()) {
                    armFSM.moveToSafeHeight();
                    if (monkeyPawFSM.PREPARED_TO_INTAKE_SAMPLE() || monkeyPawFSM.RETRACTED_INTAKE()) {
                        armFSM.retract();
                        if (armFSM.FULLY_RETRACTED()) {
                            states = States.RETRACTED_INTAKE;
                        }
                    }
                } else {
                    if (monkeyPawFSM.PREPARED_TO_INTAKE_SAMPLE() || monkeyPawFSM.RETRACTED_INTAKE()) {
                        states = States.RETRACTED_INTAKE;

                    }
                }
                break;
            case MOVING_TO_MINI_INTAKE:
                armFSM.moveToMiniIntake();
                if (armFSM.AT_MINI_INTAKE()) {
                    states = States.MOVED_TO_MINI_INTAKE;
                }
                break;
            case RETRACTING_FROM_MINI_INTAKE:
                if (monkeyPawFSM.RELAXED_MINI_INTAKE()) {
                    armFSM.retract();
                    if (armFSM.FULLY_RETRACTED()) {
                        states = States.PREPARED_TO_INTAKE;
                    }
                }
                break;
        }

    }

    public void log() {
        logger.log("-------------------------LIMB LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Limb State: ", states, Logger.LogLevels.PRODUCTION);
        logger.log("Robot Mode: ", mode, Logger.LogLevels.PRODUCTION);
        logger.log("-------------------------LIMB LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        armFSM.log();
        shoulderFSM.log();
    }


    public void updateLowLevelFSMStates() {
        armFSM.updateState();
        shoulderFSM.updateState();
    }

    public void checkIndexUpOrDown() {

    }

    public boolean PREPARING_TO_INTAKE_SPECIMEN() {
        return states == States.PREPARING_TO_INTAKE_SPECIMEN;
    }

    public boolean PREPARED_TO_INTAKE_SPECIMEN() {
        return states == States.PREPARED_TO_INTAKE_SPECIMEN;
    }

    public boolean INTAKING_SPECIMEN() {
        return states == States.INTAKING_SPECIMEN;
    }

    public boolean INTAKED_SPECIMEN() {
        return states == States.INTAKED_SPECIMEN;
    }

    public boolean EXTENDING_SPECIMEN() {
        return states == States.EXTENDING_SPECIMEN;
    }

    public boolean EXTENDED_SPECIMEN() {
        return states == States.EXTENDED_SPECIMEN;
    }

    public boolean DEPOSITING_SPECIMEN() {
        return states == States.DEPOSITING_SPECIMEN;
    }

    public boolean DEPOSITED_SPECIMEN() {
        return states == States.DEPOSITED_SPECIMEN;
    }


    public boolean PREPARING_TO_DEPOSIT_SAMPLE() {
        return states == States.PREPARING_TO_DEPOSIT_SAMPLE;
    }

    public boolean PREPARED_TO_DEPOSIT_SAMPLE() {
        return states == States.PREPARED_TO_DEPOSIT_SAMPLE;
    }

    public boolean EXTENDING_TO_BASKET_HEIGHT() {
        return states == States.EXTENDING_TO_BASKET_HEIGHT;
    }

    public boolean EXTENDED_TO_BASKET_HEIGHT() {
        return states == States.EXTENDED_TO_BASKET_HEIGHT;
    }

    public boolean DEPOSITING_SAMPLE() {
        return states == States.DEPOSITING_SAMPLE;
    }

    public boolean DEPOSITED_SAMPLE() {
        return states == States.DEPOSITED_SAMPLE;
    }

    public boolean PREPARING_TO_INTAKE() {
        return states == States.PREPARING_TO_INTAKE;
    }

    public boolean PREPARED_TO_INTAKE() {
        return states == States.PREPARED_TO_INTAKE;
    }

    public boolean MOVING_TO_INTAKE_POS() {
        return states == States.MOVING_TO_INTAKE_POS;
    }

    public boolean MOVED_TO_INTAKE_POS() {
        return states == States.MOVED_TO_INTAKE_POS;
    }

    public boolean MOVING_TO_MINI_INTAKE() {
        return states == States.MOVING_TO_MINI_INTAKE;
    }

    public boolean MOVED_TO_MINI_INTAKE() {
        return states == States.MOVED_TO_MINI_INTAKE;
    }

    public boolean RETRACTING_FROM_MINI_INTAKE() {
        return states == States.RETRACTING_FROM_MINI_INTAKE;
    }

    public boolean RETRACTED_FROM_MINI_INTAKE() {
        return states == States.RETRACTED_FROM_MINI_INTAKE;
    }

    public boolean RETRACTED_INTAKE() {
        return states == States.RETRACTED_INTAKE;
    }

    public boolean RETRACTING_INTAKE() {
        return states == States.RETRACTING_INTAKE;
    }

    public boolean SAMPLE_MODE() {
        return mode == Mode.SAMPLE_MODE;
    }

    public boolean SPECIMEN_MODE() {
        return mode == Mode.SPECIMEN_MODE;
    }

    public boolean SAMPLE_STATES() {
        return PREPARED_TO_DEPOSIT_SAMPLE() || PREPARED_TO_DEPOSIT_SAMPLE() || EXTENDING_TO_BASKET_HEIGHT() || EXTENDED_TO_BASKET_HEIGHT() || DEPOSITING_SAMPLE() || DEPOSITED_SAMPLE();
    }

    public boolean SPECIMEN_STATES() {
        return PREPARING_TO_INTAKE_SPECIMEN() || PREPARED_TO_INTAKE_SPECIMEN() || INTAKING_SPECIMEN() || INTAKED_SPECIMEN() || EXTENDING_SPECIMEN() || EXTENDED_SPECIMEN() || DEPOSITING_SPECIMEN() || DEPOSITED_SPECIMEN();
    }

    public void updatePID() {
        armFSM.updatePIDF();
        shoulderFSM.updatePID();
    }

    @VisibleForTesting
    public void setCurrentState(States state) {
        states = state;
    }

    @VisibleForTesting
    public void setCurrentMode(Mode mode) {
        this.mode = mode;
    }

    public void setMonkeyPawFSM(MonkeyPawFSM monkeyPawFSM) {
        this.monkeyPawFSM = monkeyPawFSM;
    }

    public ShoulderFSM getShoulderFSM() {
        return shoulderFSM;
    }

}
