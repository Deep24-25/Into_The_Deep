package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Config
public class LimbFSM {
    public enum States {
        START, STARTED, PREPARING_TO_INTAKE_SPECIMEN, PREPARED_TO_INTAKE_SPECIMEN, INTAKING_SPECIMEN, INTAKED_SPECIMEN, EXTENDING_SPECIMEN, EXTENDED_SPECIMEN, DEPOSITING_SPECIMEN, DEPOSITED_SPECIMEN, PREPARING_TO_DEPOSIT_SAMPLE, PREPARED_TO_DEPOSIT_SAMPLE, EXTENDING_TO_BASKET_HEIGHT, EXTENDED_TO_BASKET_HEIGHT, DEPOSITING_SAMPLE, DEPOSITED_SAMPLE, PREPARING_TO_INTAKE, PREPARED_TO_INTAKE, MOVING_TO_INTAKE_POS, LINEARIZING_INTAKE, MOVED_TO_INTAKE_POS, RETRACTING_INTAKE, RETRACTED_INTAKE, AUTO_SPEC_INTAKING, AUTO_SPEC_INTAKED, RETRACTING_FOR_AUTO, RETRACTED_FOR_AUTO, EXTENDING_TO_INTAKE_SPECIMEN, EXTENDED_TO_INTAKE_SPECIMEN
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


    private final ArmFSM armFSM;
    private final ShoulderFSM shoulderFSM;
    private volatile MonkeyPawFSM monkeyPawFSM;

    private States states = States.START;
    private Mode mode = Mode.SAMPLE_MODE;
    private Logger logger;
    private HWMap hwMap;
    private double rightY;

    private Timing.Timer autoTimer;
    public static long TIMER_LENGTH = 1500;

    boolean autoTimerDone = false;

    public LimbFSM(HWMap hwMap, ShoulderFSM shoulderFSM, ArmFSM armFSM, MonkeyPawFSM monkeyPawFSM, Logger logger) {
        this.logger = logger;
        this.hwMap = hwMap;
        this.armFSM = armFSM;
        this.shoulderFSM = shoulderFSM;
        this.monkeyPawFSM = monkeyPawFSM;
        autoTimer = new Timing.Timer(TIMER_LENGTH, TimeUnit.MILLISECONDS);

    }

    @VisibleForTesting
    public LimbFSM(ArmFSM armFSM, ShoulderFSM shoulderFSM, MonkeyPawFSM monkeyPawFSM) {
        this.armFSM = armFSM;
        this.shoulderFSM = shoulderFSM;
        this.monkeyPawFSM = monkeyPawFSM;
    }

    public void findTargetState(boolean yPressed, boolean aPressed, boolean xPressed, boolean leftBumperPressed) {
        if (yPressed && SPECIMEN_MODE()) {
            if ((DEPOSITED_SPECIMEN() || SAMPLE_STATES() || STARTED() || PREPARED_TO_INTAKE() || RETRACTED_INTAKE()) && !PREPARING_TO_INTAKE_SPECIMEN()) {
                states = States.PREPARING_TO_INTAKE_SPECIMEN;
            } else if (PREPARED_TO_INTAKE_SPECIMEN()) {
                states = States.INTAKING_SPECIMEN;
            } else if (INTAKED_SPECIMEN()) {
                states = States.EXTENDING_SPECIMEN;
            } else if (EXTENDED_SPECIMEN()) {
                if (armFSM.checkSubHeight())
                    states = States.EXTENDING_SPECIMEN;
                else
                    states = States.DEPOSITING_SPECIMEN;
            }
        } else if (yPressed && SAMPLE_MODE()) {
            if ((!PREPARED_TO_DEPOSIT_SAMPLE() && !DEPOSITING_SAMPLE() && !EXTENDED_TO_BASKET_HEIGHT() && !EXTENDING_TO_BASKET_HEIGHT() && !MOVING_TO_INTAKE_POS()) || DEPOSITED_SAMPLE() || SPECIMEN_STATES()) {
                states = States.PREPARING_TO_DEPOSIT_SAMPLE;
            } else if (PREPARED_TO_DEPOSIT_SAMPLE()) {
                states = States.EXTENDING_TO_BASKET_HEIGHT;
            } else if (EXTENDED_TO_BASKET_HEIGHT()) {
                states = States.DEPOSITING_SAMPLE;
            }
        } else if (DEPOSITED_SPECIMEN() && monkeyPawFSM.DEPOSITED_SPECIMEN()) {
            states = States.PREPARING_TO_INTAKE_SPECIMEN;
        }
        if (xPressed && SPECIMEN_MODE()) {
            if (INTAKING_SPECIMEN() || INTAKED_SPECIMEN()) {
                states = States.PREPARING_TO_INTAKE_SPECIMEN;
            } else if (EXTENDING_SPECIMEN() || EXTENDED_SPECIMEN())
                states = States.INTAKING_SPECIMEN;
            else if (DEPOSITING_SPECIMEN()) {
                states = States.EXTENDING_SPECIMEN;
            }
        }
        if (xPressed && (MOVED_TO_INTAKE_POS() || RETRACTED_INTAKE())) {
            states = States.MOVING_TO_INTAKE_POS;
        }

        if (aPressed) {
            if ((!MOVING_TO_INTAKE_POS() || DEPOSITED_SAMPLE() || DEPOSITED_SPECIMEN() || RETRACTED_INTAKE() || STARTED()) && !PREPARED_TO_INTAKE() && !MOVED_TO_INTAKE_POS()) {
                states = States.PREPARING_TO_INTAKE;
            } else if (PREPARED_TO_INTAKE() || RETRACTED_INTAKE()) {
                states = States.MOVING_TO_INTAKE_POS;
            } else if (MOVING_TO_INTAKE_POS()) {
                states = States.LINEARIZING_INTAKE;
            } else if (MOVED_TO_INTAKE_POS()) {
                states = States.RETRACTING_INTAKE;
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

    public void updateState(boolean dPadRightIsDown, boolean dPadRightWasJustReleased, boolean dpadLeftIsDown, boolean dpadLeftWasJustReleased, boolean yPressed, boolean aPressed, boolean xPressed, boolean rightTriggerPressed, boolean leftBumperPressed, boolean leftTriggerPressed, double rightY, boolean test, boolean auto, boolean dpadDown2, boolean dpadUp2) {
        updateLowLevelFSMStates(auto, dpadDown2, dpadUp2);
        shoulderFSM.resetShoulder(dPadRightIsDown, dPadRightWasJustReleased);
        armFSM.resetArm(dpadLeftIsDown, dpadLeftWasJustReleased);
        this.rightY = rightY;
        if (!test || !auto) {
            findTargetState(yPressed, aPressed, xPressed, leftBumperPressed);
        }
        switch (states) {
            case START:
                if (!shoulderFSM.AT_INTAKE()) {
                    shoulderFSM.moveToIntakeAngle();
                    if (!armFSM.FULLY_RETRACTED()) {
                        armFSM.retract();
                    }
                }
                if (shoulderFSM.AT_INTAKE() && armFSM.FULLY_RETRACTED()) {
                    states = States.STARTED;
                }
                break;
            //INTAKING STATES
            case PREPARING_TO_INTAKE:
                if (shoulderFSM.AT_INTAKE()) {
                    if (!armFSM.FULLY_RETRACTED()) {
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
                    //  armFSM.capSetPower();
                    armFSM.retract();
                    if (armFSM.FULLY_RETRACTED()) {
                        shoulderFSM.moveToIntakeAngle();
                        if (shoulderFSM.AT_INTAKE() || shoulderFSM.AT_SPECIMEN_INTAKE()) {
                            if ((monkeyPawFSM.PREPARED_TO_INTAKE_SAMPLE() || monkeyPawFSM.RETRACTED_INTAKE())) {
                                states = States.PREPARED_TO_INTAKE;
                            }
                        }
                    }
                }
                break;
            case PREPARED_TO_INTAKE:
                armFSM.uncapSetPower();
                break;
            case MOVING_TO_INTAKE_POS:
                armFSM.feed();
                break;
            case LINEARIZING_INTAKE:
                armFSM.setShouldPID(true);
                states = States.MOVED_TO_INTAKE_POS;
                break;
            case RETRACTING_INTAKE:
                if (!armFSM.FULLY_RETRACTED()) {
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
            //SPECIMEN STATES
            case PREPARING_TO_INTAKE_SPECIMEN:
                if (armFSM.FULLY_RETRACTED()) {
                    shoulderFSM.moveToIntakeAngle();
                    if ((shoulderFSM.AT_INTAKE() || shoulderFSM.AT_SPECIMEN_INTAKE()) && monkeyPawFSM.PREPARED_TO_INTAKE_SPECIMEN()) {
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
                /*hwMap.brakingOff();*/
                armFSM.setShouldPID(true);
                /*if (auto) {
                    shoulderFSM.setChamberTargetAngle();
                    armFSM.moveToSubmersibleHeight();
                } else {
                */
                shoulderFSM.setChamberTargetAngle();
                if (shoulderFSM.AT_DEPOSIT_CHAMBERS()) {
                    armFSM.moveToSubmersibleHeight();

                }

                if (shoulderFSM.AT_DEPOSIT_CHAMBERS() && armFSM.AT_SUBMERSIBLE_HEIGHT()) {
                    states = States.EXTENDED_SPECIMEN;
                }
                break;
            case DEPOSITING_SPECIMEN:
                armFSM.chamberLockHeightAlgorithm();
                /*if (auto) {
                    if (!autoTimer.isTimerOn()) {
                        autoTimer.start();
                    }
                    if (autoTimer.done()) {
                        autoTimer.pause();
                        states = States.DEPOSITED_SPECIMEN;
                    }
                }*/
                if (armFSM.AT_CHAMBER_LOCK_HEIGHT()) {
                    armFSM.setSpecimenClipped(false);
                    states = States.DEPOSITED_SPECIMEN;
                }
                break;
            //SAMPLE STATES
            case PREPARING_TO_DEPOSIT_SAMPLE:
                armFSM.retract();
                if (armFSM.FULLY_RETRACTED()) {
                    shoulderFSM.setBasketTargetAngle();
                    if (shoulderFSM.AT_BASKET_DEPOSIT()) {
                        states = States.PREPARED_TO_DEPOSIT_SAMPLE;
                    }
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
                if (!armFSM.AT_BASKET_HEIGHT()) {
                    states = States.EXTENDING_TO_BASKET_HEIGHT;
                }
                break;
            case DEPOSITING_SAMPLE:
                /* hwMap.brakingOn();*/
                if (monkeyPawFSM.RELAXED_AFTER_DEPOSIT()) {
                    states = States.DEPOSITED_SAMPLE;
                }
                break;
            case AUTO_SPEC_INTAKING:

                armFSM.setAutoSpecIntakePos();
                if (armFSM.MOVED_TO_AUTO_SPEC_INTAKE()) {
                    states = States.AUTO_SPEC_INTAKED;
                }
                break;
            case RETRACTING_FOR_AUTO:
                armFSM.retract();
                if (armFSM.FULLY_RETRACTED()) {
                    states = States.RETRACTED_FOR_AUTO;
                }
        }
    }


    public void log() {
        logger.log("-------------------------LIMB LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        logger.log("Limb State: ", states, Logger.LogLevels.PRODUCTION);
        logger.log("Robot Mode: ", mode, Logger.LogLevels.PRODUCTION);
        logger.log("auto timer", autoTimer.elapsedTime(), Logger.LogLevels.PRODUCTION);
        logger.log("-------------------------LIMB LOG---------------------------", "-", Logger.LogLevels.PRODUCTION);
        armFSM.log();
        shoulderFSM.log();
    }


    public void updateLowLevelFSMStates(boolean isAuto, boolean dpadDown, boolean dpadUp) {
        armFSM.updateState(rightY, isAuto, dpadDown, dpadUp);
        shoulderFSM.updateState(isAuto);
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

    public boolean RETRACTED_INTAKE() {
        return states == States.RETRACTED_INTAKE;
    }


    public boolean RETRACTING_INTAKE() {
        return states == States.RETRACTING_INTAKE;
    }

    public boolean EXTENDING_TO_INTAKE_SPECIMEN() {
        return states == States.EXTENDING_TO_INTAKE_SPECIMEN;
    }


    public boolean EXTENDED_TO_INTAKE_SPECIMEN() {
        return states == States.EXTENDED_TO_INTAKE_SPECIMEN;
    }

    public boolean STARTED() {
        return states == States.STARTED;
    }

    public boolean AUTO_SPEC_INTAKED() {
        return states == States.AUTO_SPEC_INTAKED;
    }

    public boolean RETRACTED_FOR_AUTO() {
        return states == States.RETRACTED_FOR_AUTO;
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

    public void updatePID(boolean isAuto) {
        armFSM.updatePIDF();
        shoulderFSM.updatePID();
    }

    public void setMonkeyPawFSM(MonkeyPawFSM monkeyPawFSM) {
        this.monkeyPawFSM = monkeyPawFSM;
    }

    public void setStates(States states) {
        this.states = states;
    }

    public States getStates() {
        return states;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public void setArmPowerCap(double powerCap) {
        armFSM.setMaxPower(powerCap);
    }

    public void setAutoSpecIntake(double height) {
        armFSM.setAutoSpecIntake(height);
    }

    public void setSubDepositHeight(double height) {
        armFSM.setSubmersibleHighAuto(height);
    }


}
