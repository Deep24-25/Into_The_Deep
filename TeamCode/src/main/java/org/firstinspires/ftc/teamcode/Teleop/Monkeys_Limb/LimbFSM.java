package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import org.firstinspires.ftc.teamcode.Core.HWMap;

public class LimbFSM {
    public enum States{
        PREPARING_TO_INTAKE_SPECIMEN, PREPARED_TO_INTAKE_SPECIMEN, INTAKING_SPECIMEN, INTAKED_SPECIMEN, EXTENDING_TO_SPECIMEN, EXTENDED_TO_SPECIMEN, DEPOSITING_SPECIMEN, DEPOSITED_SPECIMEN, PREPARING_TO_DEPOSIT_SPECIMEN, PREPARED_TO_DEPOSIT_SPECIMEN, EXTENDING_TO_BASKET_HEIGHT, EXTENDED_TO_BASKET_HEIGHT, PREPARING_TO_INTAKE, PREPARED_TO_INTAKE, MOVING_TO_INTAKE_POS, MOVED_TO_INTAKE_POS, MOVING_TO_MINI_INTAKE, MOVED_TO_MINI_INTAKE, RETRACTING_FROM_MINI_INTAKE, RETRACTED_FROM_MINI_INTAKE
    }
    public enum Mode{
        SAMPLE_MODE, SPECIMEN_MODE
    }

    private States states = States.INTAKED_SPECIMEN;
    private Mode mode = Mode.SAMPLE_MODE;
    public LimbFSM(){

    }
    public void findTargetState(boolean yPressed, boolean aPressed, boolean rightBumperPressed, boolean leftBumperPressed){

    }
    public void updateState(){

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
    public boolean EXTENDED_TO_SPECIMEN(){
        return states == States.EXTENDED_TO_SPECIMEN;
    }
    public boolean DEPOSITING_SPECIMEN(){
        return states == States.DEPOSITING_SPECIMEN;
    }
    public boolean DEPOSITED_SPECIMEN(){
        return states == States.DEPOSITED_SPECIMEN;
    }
    public boolean PREPARING_TO_DEPOSIT_SPECIMEN(){
        return states == States.PREPARING_TO_DEPOSIT_SPECIMEN;
    }
    public boolean PREPARED_TO_DEPOSIT_SPECIMEN(){
        return states == States.PREPARED_TO_DEPOSIT_SPECIMEN;
    }
    public boolean EXTENDING_TO_BASKET_HEIGHT(){
        return states == States.EXTENDING_TO_BASKET_HEIGHT;
    }
    public boolean EXTENDED_TO_BASKET_HEIGHT(){
        return states == States.EXTENDED_TO_BASKET_HEIGHT;
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
