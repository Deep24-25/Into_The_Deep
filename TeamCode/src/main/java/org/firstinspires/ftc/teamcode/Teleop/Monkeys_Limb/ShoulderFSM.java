package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

public class ShoulderFSM {
    public enum States {
        GOING_TO_CHAMBER, AT_DEPOSIT_CHAMBERS, GOING_TO_INTAKE, AT_INTAKE, GOING_TO_BASKET, AT_BASKET_DEPOSIT
    }

    private States currentState = States.GOING_TO_CHAMBER;

    public ShoulderFSM(){

    }
    public void updateState(){

    }

    public boolean isShoulderTargetPosDepositChamberAngle(){return false;}
    public boolean isShoulderTargetPosIntakeAngle(){return false;}
    public boolean isShoulderTargetPosDepositBasketAngle(){return false;}

    public boolean isShoulderCurrentPosDepositChamberAngle(){return false;}
    public boolean isShoulderCurrentPosIntakeAngle(){return false;}
    public boolean isShoulderCurrentPosDepositBasketAngle(){return false;}

    public boolean GOING_TO_CHAMBER(){
       return currentState == States.GOING_TO_CHAMBER;
    }
    public boolean AT_DEPOSIT_CHAMBERS(){
        return currentState == States.AT_DEPOSIT_CHAMBERS;
    }
    public boolean GOING_TO_INTAKE(){
        return currentState == States.GOING_TO_INTAKE;
    }
    public boolean AT_INTAKE(){
        return currentState == States.AT_INTAKE;
    }
    public boolean GOING_TO_BASKET(){
        return currentState == States.GOING_TO_BASKET;
    }
    public boolean AT_BASKET_DEPOSIT(){
        return currentState == States.AT_BASKET_DEPOSIT;
    }

}
