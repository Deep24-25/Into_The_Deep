
package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

class LimbFSMTest {
    private LimbFSM sut;
    private ArmFSM armFSMMock;
    private ShoulderFSM shoulderFSMMock;
    private MonkeyPawFSM pawFSMMock;


    @BeforeEach
    public void setup() {
        armFSMMock = mock();
        shoulderFSMMock = mock();
        pawFSMMock = mock();
        sut = spy(new LimbFSM(armFSMMock, shoulderFSMMock, pawFSMMock));
    }



/** ------------------------------------findTargetState()-----------------------------------

      **/




    // Specimen State Tests
    @Test
    public void preparingToIntakeSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.PREPARED_TO_INTAKE()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }
/*
    @Test
    public void intakingSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.PREPARED_TO_INTAKE_SPECIMEN());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void extendingToSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.INTAKED_SPECIMEN());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }

    @Test
    public void depositingSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.EXTENDED_SPECIMEN());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }

    @Test
    public void intakingSpecimenToPreparingToIntakeSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.INTAKING_SPECIMEN());

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void intakedSpecimenToPreparingToIntakeSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.INTAKED_SPECIMEN());

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void extendingToSpecimenToIntakingSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.EXTENDING_TO_SPECIMEN());

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void extendedToSpecimenToIntakingSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.EXTENDED_SPECIMEN());

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void depositingSpecimenToExtendingToSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.DEPOSITING_SPECIMEN());

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }

    @Test
    public void depositedSpecimenAndYPressed() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.DEPOSITED_SPECIMEN());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());

    }

    @Test
    public void depositedSpecimenAndAPressed() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.DEPOSITED_SPECIMEN());

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());

    }

    //Sample State Tests
    @Test
    public void preparingToDepositStateAndNotDepositedSample() {
        doReturn(true).when(sut.DEPOSITED_SAMPLE());
        doReturn(true).when(sut.SAMPLE_MODE());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());

    }

    @Test
    public void preparingToDepositStateAndNotPreparedToDepositSampleAndNotDepositingSample() {
        doReturn(false).when(sut.PREPARED_TO_DEPOSIT_SAMPLE());
        doReturn(false).when(sut.DEPOSITING_SAMPLE());
        doReturn(true).when(sut.SAMPLE_MODE());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());

    }

    @Test
    public void extendingToBasketHeight() {
        doReturn(true).when(sut.PREPARED_TO_DEPOSIT_SAMPLE());
        doReturn(true).when(sut.SAMPLE_MODE());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_BASKET_HEIGHT());

    }

    @Test
    public void depositingSample() {
        doReturn(true).when(sut.EXTENDED_TO_BASKET_HEIGHT());
        doReturn(true).when(sut.SAMPLE_MODE());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.DEPOSITING_SAMPLE());
    }

    //Intake States
    @Test
    public void preparingToIntakeAndNotPreparedToIntake() {
        doReturn(false).when(sut.PREPARED_TO_INTAKE());

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndNotMovingToIntakePos() {
        doReturn(false).when(sut.PREPARED_TO_INTAKE());

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndDepositingSample() {
        doReturn(true).when(sut.DEPOSITING_SAMPLE());

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndDepositingSpecimen() {
        doReturn(true).when(sut.DEPOSITING_SPECIMEN());

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void movingToIntakeIfPreparedToIntake() {
        doReturn(true).when(sut.PREPARED_TO_INTAKE());

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.MOVING_TO_INTAKE_POS());
    }

    @Test
    public void movingToMiniIntakeIfPreparedToIntake() {
        doReturn(true).when(sut.PREPARED_TO_INTAKE());

        sut.findTargetState(false, false, false, true, false, false, false);

        assertTrue(sut.MOVING_TO_MINI_INTAKE());
    }


    @Test
    public void retractingFromMiniIntakeIfMovedToMiniIntakeAndMonkeyPawMiniIntaked() {
        doReturn(true).when(sut.MOVED_TO_MINI_INTAKE());
        doReturn(true).when(pawFSMMock.MINI_INTAKED());

        sut.findTargetState(false, false, false, true, false, false, false);

        assertTrue(sut.RETRACTING_FROM_MINI_INTAKE());
    }
    */




/** ------------------------------------updateState()-----------------------------------
     **/



    //Specimen: Preparing To Intake Specimen
    @Test
    public void whenArmIsFullyRetracted() {
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);
        when(shoulderFSMMock.AT_SPECIMEN_INTAKE()).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SPECIMEN()).thenReturn(true);
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock, never()).retract();
        verify(shoulderFSMMock).moveToSpecimenIntakeAngle();

        assertTrue(sut.PREPARED_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void whenArmIsNotFullyRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);
        when(shoulderFSMMock.AT_SPECIMEN_INTAKE()).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SPECIMEN()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retract();

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    //Specimen: Intaking Specimen
    @Test
    public void whenPawHasNotIntaked() {
        sut.setCurrentState(LimbFSM.States.INTAKING_SPECIMEN);
        when(pawFSMMock.INTAKED_SPECIMEN()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock, never()).moveToSpecimenPickUpHeight();
        assertFalse(sut.INTAKED_SPECIMEN());
        assertFalse(armFSMMock.AT_SPECIMEN_PICKUP_HEIGHT());
    }

    @Test
    public void whenPawHasIntaked() {
        sut.setCurrentState(LimbFSM.States.INTAKING_SPECIMEN);
        when(pawFSMMock.INTAKED_SPECIMEN()).thenReturn(true);
        when(armFSMMock.AT_SPECIMEN_PICKUP_HEIGHT()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).moveToSpecimenPickUpHeight();
        assertTrue(sut.INTAKED_SPECIMEN());
    }

    //Specimen: Extending Specimen
    @Test
    public void extendingSpecimenAndArmNotAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.EXTENDING_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(false);
        when(shoulderFSMMock.AT_DEPOSIT_CHAMBERS()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false,false,false,false,true,false,false);

        verify(shoulderFSMMock).moveToLowChamberAngle();
        verify(armFSMMock).moveToSubmersibleLowHeight();

        assertFalse(sut.EXTENDED_SPECIMEN());
    }

    @Test
    public void extendingSpecimenAndArmAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.EXTENDING_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(true);
        when(shoulderFSMMock.AT_DEPOSIT_CHAMBERS()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false,false,false,false,false,false,true);

        verify(shoulderFSMMock).moveToHighChamberAngle();
        verify(armFSMMock).moveToSubmersibleHighHeight();

        assertTrue(sut.EXTENDED_SPECIMEN());
    }

    //Specimen: Extended Specimen
    @Test
    public void extendedSpecimenAndArmNotAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.EXTENDED_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(false);
        when(shoulderFSMMock.AT_DEPOSIT_CHAMBERS()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false,false,false,false,true,false,false);

        verify(shoulderFSMMock).moveToLowChamberAngle();
        verify(armFSMMock).moveToSubmersibleLowHeight();

        assertFalse(sut.EXTENDED_SPECIMEN());
    }

    @Test
    public void extendedSpecimenAndArmAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.EXTENDED_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(true);
        when(shoulderFSMMock.AT_DEPOSIT_CHAMBERS()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false,false,false,false,false,false,true);

        verify(shoulderFSMMock).moveToHighChamberAngle();
        verify(armFSMMock).moveToSubmersibleHighHeight();

        assertTrue(sut.EXTENDED_SPECIMEN());
    }
/*
    //Specimen: Depositing Specimen
    @Test
    public void depositingSpecimenAndArmNotAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(armFSMMock.atTargetPos()).thenReturn(false);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).moveToChamberLockHeight();

        assertTrue(sut.DEPOSITING_SPECIMEN());
    }

    @Test
    public void depositingSpecimenAndArmAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(armFSMMock.atTargetPos()).thenReturn(true);
        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).moveToChamberLockHeight();

        assertTrue(sut.DEPOSITED_SPECIMEN());
    }

    //Specimen: Deposited Specimen
    @Test
    public void depositedSpecimenAndPawIsGripped() {
        sut.setCurrentState(LimbFSM.States.DEPOSITED_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(pawFSMMock.RELAXING_AFTER_DEPOSIT()).thenReturn(false);
        sut.updateState(false,false,false,false,false,false,false);

        assertTrue(sut.DEPOSITED_SPECIMEN());
    }

    @Test
    public void depositedSpecimenAndPawIsUngripped() {
        sut.setCurrentState(LimbFSM.States.DEPOSITED_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(pawFSMMock.RELAXING_AFTER_DEPOSIT()).thenReturn(true);
        sut.updateState(false,false,false,false,false,false,false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    //Sample: Preparing to Deposit Sample
    @Test
    public void armNotRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_DEPOSIT_SAMPLE);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock, never()).isShoulderTargetPosDepositBasketAngle();
        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());
    }

    @Test
    public void armIsRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_DEPOSIT_SAMPLE);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock, never()).retractToIntake();
        verify(shoulderFSMMock).isShoulderTargetPosDepositBasketAngle();
        assertTrue(sut.PREPARED_TO_DEPOSIT_SAMPLE());
    }

    //Sample: Extending to basket height
    @Test
    public void extendingToSampleHeight() {
        sut.setCurrentState(LimbFSM.States.EXTENDING_TO_BASKET_HEIGHT);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.AT_BASKET_HEIGHT()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).moveToSelectedIndexPosition();
        assertTrue(sut.EXTENDED_TO_BASKET_HEIGHT());

    }

    @Test
    public void extendedSampleAndIndexChange() {
        sut.setCurrentState(LimbFSM.States.EXTENDED_TO_BASKET_HEIGHT);
        when(armFSMMock.atTargetPos()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);

        sut.updateState(false,false,false,false,false,false,false);

        verify(sut).checkIndexUpOrDown();

        assertFalse(sut.EXTENDING_TO_BASKET_HEIGHT());
    }
    //Sample: Depositing Sample

    @Test
    public void depositingSampleStateAndRelaxedAfterDeposit() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SAMPLE);
        when(pawFSMMock.RELAXED_AFTER_DEPOSIT()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(pawFSMMock).RELAXED_AFTER_DEPOSIT();

        assertTrue(sut.DEPOSITED_SAMPLE());
    }

    @Test
    public void depositingSampleStateAndNotRelaxedAfterDeposit() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SAMPLE);
        when(pawFSMMock.RELAXED_AFTER_DEPOSIT()).thenReturn(false);

        sut.updateState(false,false,false,false,false,false,false);

        verify(pawFSMMock).RELAXED_AFTER_DEPOSIT();

        assertTrue(sut.DEPOSITING_SAMPLE());
    }

    //Intake: Preparing To Intake
    @Test
    public void preparingToIntakeAndShoulderAtIntakePosAndArmIsRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock, never()).retractToIntake();
        verify(shoulderFSMMock, never()).goToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());


    }

    @Test
    public void preparingToIntakeAndShoulderNotAtIntakePosAndPawPreparedToIntake() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(false);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(false);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock).goToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndShoulderNotAtIntakePosAndPawRelaxedPosWithSample() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(false);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(false);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock).goToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndShoulderAtIntakePosAndArmNotRetractedAndRelaxedPosWithSample() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(false);


        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retractToIntake();
        verify(armFSMMock).moveToSafeHeight();
        verify(shoulderFSMMock, never()).goToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndShoulderAtIntakePosAndArmNotRetractedAndPawPreparedToIntake() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(false);


        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retractToIntake();
        verify(armFSMMock).moveToSafeHeight();
        verify(shoulderFSMMock, never()).goToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }
    //Intake: Moving To Intake

    //TODO: Do not know the logic yet

    //Intake: Moving To Mini Intake
    @Test
    public void movingToMiniIntake() {
        sut.setCurrentState(LimbFSM.States.MOVING_TO_MINI_INTAKE);
        when(armFSMMock.atTargetPos()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).moveToMiniIntake();
        assertTrue(sut.MOVED_TO_MINI_INTAKE());
    }

    //Intake: Retracting From Mini Intake
    @Test
    public void retractingFromMiniIntakeAndRelaxedMiniIntake() {
        sut.setCurrentState(LimbFSM.States.RETRACTING_FROM_MINI_INTAKE);
        when(pawFSMMock.RELAXED_MINI_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock).retractToIntake();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void retractingFromMiniIntakeAndNotRelaxedMiniIntake() {
        sut.setCurrentState(LimbFSM.States.RETRACTING_FROM_MINI_INTAKE);
        when(pawFSMMock.RELAXED_MINI_INTAKE()).thenReturn(false);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false,false,false,false,false,false,false);

        verify(armFSMMock, never()).retractToIntake();
        assertTrue(sut.RETRACTING_FROM_MINI_INTAKE());
    }

*/

}



