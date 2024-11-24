package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import androidx.annotation.VisibleForTesting;

public class LimbFSM {
    public enum States{
        PREPARING_TO_INTAKE_SPECIMEN, PREPARED_TO_INTAKE_SPECIMEN, INTAKING_SPECIMEN, INTAKED_SPECIMEN, EXTENDING_TO_SPECIMEN, EXTENDED_SPECIMEN, DEPOSITING_SPECIMEN, DEPOSITED_SPECIMEN, PREPARING_TO_DEPOSIT_SPECIMEN, PREPARED_TO_DEPOSIT_SPECIMEN, PREPARING_TO_DEPOSIT_SAMPLE, PREPARED_TO_DEPOSIT_SAMPLE, EXTENDING_TO_BASKET_HEIGHT, EXTENDED_TO_BASKET_HEIGHT, DEPOSITING_SAMPLE, DEPOSITED_SAMPLE, PREPARING_TO_INTAKE, PREPARED_TO_INTAKE, MOVING_TO_INTAKE_POS, MOVED_TO_INTAKE_POS, MOVING_TO_MINI_INTAKE, MOVED_TO_MINI_INTAKE, RETRACTING_FROM_MINI_INTAKE, RETRACTED_FROM_MINI_INTAKE
    }
    public enum Mode{
        SAMPLE_MODE, SPECIMEN_MODE
    }

    private States currentState = States.INTAKED_SPECIMEN;
    private Mode currentMode = Mode.SAMPLE_MODE;
    public LimbFSM(){

    }
    public void findTargetState(boolean yPressed, boolean aPressed, boolean xPressed, boolean rbPressed, boolean lbPressed,     boolean rightBumperPressed, boolean leftBumperPressed){

    }
    public void updateState(){

    }

    public boolean PREPARING_TO_INTAKE_SPECIMEN(){
        return currentState == States.PREPARING_TO_INTAKE_SPECIMEN;
    }
    public boolean PREPARED_TO_INTAKE_SPECIMEN(){
        return currentState == States.PREPARED_TO_INTAKE_SPECIMEN;
    }
    public boolean INTAKING_SPECIMEN(){
        return currentState == States.INTAKING_SPECIMEN;
    }
    public boolean INTAKED_SPECIMEN(){
        return currentState == States.INTAKED_SPECIMEN;
    }
    public boolean EXTENDING_TO_SPECIMEN(){
        return currentState == States.EXTENDING_TO_SPECIMEN;
    }
    public boolean EXTENDED_SPECIMEN(){
        return currentState == States.EXTENDED_SPECIMEN;
    }
    public boolean DEPOSITING_SPECIMEN(){
        return currentState == States.DEPOSITING_SPECIMEN;
    }
    public boolean DEPOSITED_SPECIMEN(){
        return currentState == States.DEPOSITED_SPECIMEN;
    }
    public boolean PREPARING_TO_DEPOSIT_SPECIMEN(){
        return currentState == States.PREPARING_TO_DEPOSIT_SPECIMEN;
    }
    public boolean PREPARED_TO_DEPOSIT_SPECIMEN(){
        return currentState == States.PREPARED_TO_DEPOSIT_SPECIMEN;
    }
    public boolean PREPARING_TO_DEPOSIT_SAMPLE(){
        return currentState == States.PREPARING_TO_DEPOSIT_SAMPLE;
    }
    public boolean PREPARED_TO_DEPOSIT_SAMPLE(){
        return currentState == States.PREPARED_TO_DEPOSIT_SAMPLE;
    }
    public boolean EXTENDING_TO_BASKET_HEIGHT(){
        return currentState == States.EXTENDING_TO_BASKET_HEIGHT;
    }
    public boolean EXTENDED_TO_BASKET_HEIGHT(){
        return currentState == States.EXTENDED_TO_BASKET_HEIGHT;
    }
    public boolean DEPOSITING_SAMPLE(){
        return currentState == States.DEPOSITING_SAMPLE;
    }
    public boolean DEPOSITED_SAMPLE(){
        return currentState == States.DEPOSITED_SAMPLE;
    }
    public boolean PREPARING_TO_INTAKE(){
        return currentState == States.PREPARING_TO_INTAKE;
    }

    public boolean PREPARED_TO_INTAKE(){
        return currentState == States.PREPARED_TO_INTAKE;
    }
    public boolean MOVING_TO_INTAKE_POS(){
        return currentState == States.MOVING_TO_INTAKE_POS;
    }
    public boolean MOVED_TO_INTAKE_POS(){
        return currentState == States.MOVED_TO_INTAKE_POS;
    }
    public boolean MOVING_TO_MINI_INTAKE(){
        return currentState == States.MOVING_TO_MINI_INTAKE;
    }
    public boolean MOVED_TO_MINI_INTAKE(){
        return currentState == States.MOVED_TO_MINI_INTAKE;
    }
    public boolean RETRACTING_FROM_MINI_INTAKE(){
        return currentState == States.RETRACTING_FROM_MINI_INTAKE;
    }
    public boolean RETRACTED_FROM_MINI_INTAKE(){
        return currentState == States.RETRACTED_FROM_MINI_INTAKE;
    }
    public boolean SAMPLE_MODE(){
        return currentMode == Mode.SAMPLE_MODE;
    }
    public boolean SPECIMEN_MODE(){
        return currentMode == Mode.SPECIMEN_MODE;
    }
    @VisibleForTesting
    public void setCurrentState(States state) {
        currentState = state;
    }
    @VisibleForTesting
    public void setCurrentMode(Mode mode) {
        this.currentMode = mode;
    }

}
