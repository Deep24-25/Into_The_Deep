package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;

public class LimbFSM {
    public enum States{
        PREPARING_TO_INTAKE_SPECIMEN, PREPARED_TO_INTAKE_SPECIMEN, INTAKING_SPECIMEN, INTAKED_SPECIMEN, EXTENDING_TO_SPECIMEN, EXTENDING_SPECIMEN, EXTENDED_SPECIMEN, DEPOSITING_SPECIMEN, DEPOSITED_SPECIMEN, PREPARING_TO_DEPOSIT_SPECIMEN, PREPARED_TO_DEPOSIT_SPECIMEN, PREPARING_TO_DEPOSIT_SAMPLE, PREPARED_TO_DEPOSIT_SAMPLE, EXTENDING_TO_BASKET_HEIGHT, EXTENDED_TO_BASKET_HEIGHT, DEPOSITING_SAMPLE, DEPOSITED_SAMPLE, PREPARING_TO_INTAKE, PREPARED_TO_INTAKE, MOVING_TO_INTAKE_POS, MOVED_TO_INTAKE_POS, MOVING_TO_MINI_INTAKE, MOVED_TO_MINI_INTAKE, RETRACTING_FROM_MINI_INTAKE, RETRACTED_FROM_MINI_INTAKE
    }
    public enum Mode{
        SAMPLE_MODE, SPECIMEN_MODE
    }

    /*SPECIMEN STATES
    PREPARING_TO_INTAKE_SPECIMEN
    PREPARED_TO_INTAKE_SPECIMEN
    INTAKING_SPECIMEN
    INTAKED_SPECIMEN
    EXTENDING_TO_SPECIMEN
    EXTENDING_SPECIMEN
    EXTENDED_SPECIMEN
    DEPOSITING_SPECIMEN
    DEPOSITED_SPECIMEN
    PREPARING_TO_DEPOSIT_SPECIMEN
    PREPARED_TO_DEPOSIT_SPECIMEN
     */

    /* SAMPLE STATES
    PREPARING_TO_DEPOSIT_SAMPLE
    PREPARED_TO_DEPOSIT_SAMPLE, EXTENDING_TO_BASKET_HEIGHT
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
    private volatile ArmFSM armFSM;
    private volatile ArmMotorsWrapper armMotorsWrapper;
    private volatile ShoulderFSM shoulderFSM;
    private volatile ShoulderWrapper shoulderWrapper;

    private States states = States.INTAKED_SPECIMEN;
    private Mode mode = Mode.SAMPLE_MODE;

    public LimbFSM(){

    }

    public void findTargetState(boolean yPressed, boolean aPressed, boolean xPressed, boolean rightBumperPressed, boolean leftBumperPressed){
     if (yPressed && SPECIMEN_MODE()){
         if (PREPARED_TO_INTAKE() || DEPOSITED_SPECIMEN()){
             states = States.PREPARING_TO_INTAKE_SPECIMEN;
         }
         else if (PREPARED_TO_INTAKE_SPECIMEN()){
             states = States.INTAKING_SPECIMEN;
         }
         else if (INTAKED_SPECIMEN()){
             states = States.EXTENDING_TO_SPECIMEN;
         }
         else if (EXTENDED_SPECIMEN()){
             states = States.DEPOSITING_SPECIMEN;
         }
     }
     else if (yPressed && SAMPLE_MODE()){
         if (!PREPARED_TO_DEPOSIT_SAMPLE() && !DEPOSITING_SAMPLE() || DEPOSITED_SAMPLE()){
             states = States.PREPARING_TO_DEPOSIT_SAMPLE;
         }
         else if (PREPARED_TO_DEPOSIT_SAMPLE()){
             states = States.EXTENDING_TO_BASKET_HEIGHT;
         }
         else if (EXTENDED_TO_BASKET_HEIGHT()){
             states = States.DEPOSITING_SAMPLE;
         }
     }
     if (xPressed){
         if (INTAKING_SPECIMEN() || INTAKED_SPECIMEN()){
             states = States.PREPARING_TO_INTAKE_SPECIMEN;
         }
         else if (EXTENDING_TO_SPECIMEN() || EXTENDED_SPECIMEN())
             states = States.INTAKING_SPECIMEN;
         else if (DEPOSITING_SPECIMEN()){
             states = States.EXTENDING_SPECIMEN;
         }
     }
     if (aPressed){
         if (!PREPARED_TO_INTAKE_SPECIMEN() || !MOVING_TO_INTAKE_POS() || DEPOSITED_SAMPLE() || DEPOSITED_SPECIMEN()){
             states = States.PREPARING_TO_INTAKE;
         }
         else if (PREPARED_TO_INTAKE()) {
             states = States.MOVING_TO_INTAKE_POS;
         }
     }
     if (rightBumperPressed){
         if (PREPARED_TO_INTAKE()){
             states = States.MOVING_TO_MINI_INTAKE;
         } else if (MOVED_TO_MINI_INTAKE() && monkeyPaw.MINI_INTAKED()){
             states = States.RETRACTING_FROM_MINI_INTAKE;
         }
     }
     if (leftBumperPressed){
         if (SPECIMEN_MODE()){
             mode = Mode.SAMPLE_MODE;
         }
         else {
             mode = Mode.SPECIMEN_MODE;
         }
     }
     if (DEPOSITED_SPECIMEN() && SPECIMEN_MODE()){
         if (yPressed){
             states = States.PREPARING_TO_INTAKE_SPECIMEN;
         }
         if (aPressed){
             states = States.PREPARING_TO_INTAKE;
         }
     }
    }
    public void updateState(boolean yPressed, boolean aPressed, boolean xPressed, boolean rightBumperPressed, boolean rightTwoBumperPressed, boolean leftBumperPressed, boolean leftTwoBumperPressed) {
        updateLowLevelFSMStates();
        switch (states) {
            case PREPARING_TO_INTAKE_SPECIMEN:
                if (armFSM.FULLY_RETRACTED()){
                    shoulderFSM.moveToSpecimenIntakeAngle();
                    if (shoulderFSM.AT_SPECIMEN_INTAKE() || monkeysPawFSM.PREPARED_TO_INTAKE_SPECIMEN()) {
                        states = States.PREPARED_TO_INTAKE_SPECIMEN;
                    }
                }
                else{
                    armFSM.retract();
                }
                break;
            case INTAKING_SPECIMEN:
                if (monkeysPawFSM.INTAKED_SPECIMEN()) {
                    armFSM.indexIncrement();
                    states = States.INTAKED_SPECIMEN;
            }
                break;
            case EXTENDING_SPECIMEN:
                if (leftTwoBumperPressed) {
                    shoulderFSM.moveToLowChamberAngle();
                    armFSM.moveToSubmersibleLowHeight();
                }
                else if (rightTwoBumperPressed){
                    shoulderFSM.moveToHighChamberAngle();
                    armFSM.moveToSubmersibleHighHeight();
                }
                if (armFSM.AT_SUBMERSIBLE_HEIGHT()) {
                        states = States.EXTENDED_SPECIMEN;
                }
                else {
                    states = States.EXTENDING_SPECIMEN;
                }
                break;
            case DEPOSITING_SPECIMEN:
                armFSM.indexIncrement();
                if (armFSM.AT_SUBMERSIBLE_HEIGHT()){
                    states = States.DEPOSITED_SPECIMEN;
                }
                break;
            case DEPOSITED_SPECIMEN:
                monkeysPawFSM.UNGRIPPED();
                states = States.PREPARED_TO_INTAKE_SPECIMEN;
                break;
            case PREPARING_TO_DEPOSIT_SAMPLE:
                if (armFSM.FULLY_RETRACTED()){
                    shoulderFSM.GOING_TO_BASKET();
                    if (shoulderFSM.AT_BASKET_DEPOSIT()) {
                        states = States.PREPARED_TO_DEPOSIT_SAMPLE;
                    }
                }
                else {
                    armFSM.retract();
                }
                break;
            case EXTENDING_TO_BASKET_HEIGHT:
                armFSM.indexIncrement();
                if (armFSM.AT_BASKET_HEIGHT()){
                    states = States.EXTENDED_TO_BASKET_HEIGHT;
                }
                break;
            case EXTENDED_TO_BASKET_HEIGHT:
                armFSM.getCurrentIndex();
                if (!armFSM.AT_BASKET_HEIGHT()) {
                    states = States.EXTENDING_TO_BASKET_HEIGHT;
                }
                break;
            case DEPOSITING_SAMPLE:
                monkeysPawFSM.RELAXED_AFTER_DEPOSIT;
                states = States.DEPOSITED_SAMPLE;
                break;
            case PREPARING_TO_INTAKE:
                if (shoulderFSM.AT_INTAKE()){
                    if (!armFSM.FULLY_RETRACTED()){
                        armFSM.moveToSafeHeight();
                        if (monkeysPawFSM.PREPARED_TO_INTAKE || monkeysPawFSM.RELAXED_WITH_SAMPLE){
                            armFSM.retract();
                            if (armFSM.FULLY_RETRACTED()) {
                                states = States.PREPARED_TO_INTAKE;
                            }
                        }
                    }
                    else {
                        states = States.PREPARED_TO_INTAKE;
                    }
                }
                else {
                    armFSM.retract();
                    if (armFSM.FULLY_RETRACTED()){
                        shoulderFSM.moveToIntakeAngle();
                        if (shoulderFSM.AT_INTAKE()){
                            if (monkeysPawFSM.PREPARED_TO_INTAKE || monkeysPawFSM.RELAXED_WITH_SAMPLE){
                                states = States.PREPARED_TO_INTAKE;
                            }
                        }
                    }
                }
                break;
            case MOVING_TO_INTAKE_POS:
                if (armFSM.FULLY_EXTENDED()) {
                    armFSM.setPowerCapMovement();
                    if (aPressed) {
                        states = States.MOVED_TO_INTAKE_POS;
                    }
                }
                break;
            case MOVED_TO_INTAKE_POS:
                if (xPressed){
                    states = States.MOVING_TO_INTAKE_POS;
                }
                break;
            case MOVING_TO_MINI_INTAKE:
                armFSM.indexIncrement();
                states = States.MOVED_TO_MINI_INTAKE;
                break;
            case RETRACTING_FROM_MINI_INTAKE:
                if (monkeysPawFSM.RELAXED_MINI_INTAKE()){
                    armFSM.retract();
                    if (armFSM.FULLY_RETRACTED()){
                        states = States.PREPARED_TO_INTAKE;
                    }
                }
                break;
        }

    }


    public void updateLowLevelFSMStates(){
        armFSM.updateState();
        shoulderFSM.updateState();
    }

    public boolean PREPARING_TO_INTAKE_SPECIMEN(){
        return states == States.PREPARING_TO_INTAKE_SPECIMEN;
    }
    public boolean PREPARED_TO_INTAKE_SPECIMEN(){
        return states == States.PREPARED_TO_INTAKE_SPECIMEN;
    }
    public boolean INTAKING_SPECIMEN(){
        return states == States.INTAKING_SPECIMEN;
    }
    public boolean INTAKED_SPECIMEN(){
        return states == States.INTAKED_SPECIMEN;
    }
    public boolean EXTENDING_TO_SPECIMEN(){
        return states == States.EXTENDING_TO_SPECIMEN;
    }
    public boolean EXTENDED_SPECIMEN(){
        return states == States.EXTENDED_SPECIMEN;
    }
    public boolean DEPOSITING_SPECIMEN(){
        return states == States.DEPOSITING_SPECIMEN;
    }
    public boolean DEPOSITED_SPECIMEN(){
        return states == States.DEPOSITED_SPECIMEN;
    }
    public boolean PREPARING_TO_DEPOSIT_SAMPLE(){
        return states == States.PREPARING_TO_DEPOSIT_SAMPLE;
    }
    public boolean PREPARED_TO_DEPOSIT_SAMPLE(){
        return states == States.PREPARED_TO_DEPOSIT_SAMPLE;
    }
    public boolean EXTENDING_TO_BASKET_HEIGHT(){
        return states == States.EXTENDING_TO_BASKET_HEIGHT;
    }
    public boolean EXTENDED_TO_BASKET_HEIGHT(){
        return states == States.EXTENDED_TO_BASKET_HEIGHT;
    }

    public boolean DEPOSITING_SAMPLE(){
        return states == States.DEPOSITING_SAMPLE;
    }
    public boolean DEPOSITED_SAMPLE(){
        return states == States.DEPOSITED_SAMPLE;
    }
    public boolean PREPARING_TO_INTAKE(){
        return states == States.PREPARING_TO_INTAKE;
    }

    public boolean PREPARED_TO_INTAKE(){
        return states == States.PREPARED_TO_INTAKE;
    }
    public boolean MOVING_TO_INTAKE_POS(){
        return states == States.MOVING_TO_INTAKE_POS;
    }
    public boolean MOVED_TO_INTAKE_POS(){
        return states == States.MOVED_TO_INTAKE_POS;
    }
    public boolean MOVING_TO_MINI_INTAKE(){
        return states == States.MOVING_TO_MINI_INTAKE;
    }
    public boolean MOVED_TO_MINI_INTAKE(){
        return states == States.MOVED_TO_MINI_INTAKE;
    }
    public boolean RETRACTING_FROM_MINI_INTAKE(){
        return states == States.RETRACTING_FROM_MINI_INTAKE;
    }
    public boolean RETRACTED_FROM_MINI_INTAKE(){
        return states == States.RETRACTED_FROM_MINI_INTAKE;
    }
    public boolean SAMPLE_MODE(){
        return mode == Mode.SAMPLE_MODE;
    }
    public boolean SPECIMEN_MODE(){
        return mode == Mode.SPECIMEN_MODE;
    }


}
