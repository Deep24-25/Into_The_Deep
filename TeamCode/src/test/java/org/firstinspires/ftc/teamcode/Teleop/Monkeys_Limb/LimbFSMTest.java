
package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.atLeastOnce;
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


    /**
     * ------------------------------------findTargetState()-----------------------------------
     **/


    // Specimen State Tests
    @Test
    public void preparingToIntakeSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.PREPARED_TO_INTAKE()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void intakingSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.PREPARED_TO_INTAKE_SPECIMEN()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void extendingToSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.INTAKED_SPECIMEN()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }

    @Test
    public void depositingSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.EXTENDED_SPECIMEN()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }

    @Test
    public void intakingSpecimenToPreparingToIntakeSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.INTAKING_SPECIMEN()).thenReturn(true);

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void intakedSpecimenToPreparingToIntakeSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.INTAKED_SPECIMEN()).thenReturn(true);

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void extendingToSpecimenToIntakingSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.INTAKED_SPECIMEN()).thenReturn(false);

        when(sut.EXTENDING_TO_SPECIMEN()).thenReturn(true);

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void extendedToSpecimenToIntakingSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.INTAKED_SPECIMEN()).thenReturn(false);

        when(sut.EXTENDED_SPECIMEN()).thenReturn(true);

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void depositingSpecimenToExtendingToSpecimenState() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.DEPOSITING_SPECIMEN()).thenReturn(true);
        when(sut.INTAKED_SPECIMEN()).thenReturn(false);
        when(sut.EXTENDING_TO_SPECIMEN()).thenReturn(false);

        sut.findTargetState(false, false, true, false, false, false, false);

        assertTrue(sut.EXTENDING_SPECIMEN());
    }

    @Test
    public void depositedSpecimenAndYPressed() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.DEPOSITED_SPECIMEN()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());

    }

    @Test
    public void depositedSpecimenAndAPressed() {
        when(sut.SPECIMEN_MODE()).thenReturn(true);
        when(sut.DEPOSITED_SPECIMEN()).thenReturn(true);

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());

    }

    //Sample State Tests
    @Test
    public void preparingToDepositStateAndNotDepositedSample() {
        when(sut.DEPOSITED_SAMPLE()).thenReturn(true);
        when(sut.SAMPLE_MODE()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());

    }

    @Test
    public void preparingToDepositStateAndNotPreparedToDepositSampleAndNotDepositingSample() {
        when(sut.PREPARED_TO_DEPOSIT_SAMPLE()).thenReturn(false);
        when(sut.DEPOSITING_SAMPLE()).thenReturn(false);
        when(sut.SAMPLE_MODE()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());

    }

    @Test
    public void extendingToBasketHeight() {
        when(sut.PREPARED_TO_DEPOSIT_SAMPLE()).thenReturn(true);
        when(sut.SAMPLE_MODE()).thenReturn(true);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.EXTENDING_TO_BASKET_HEIGHT());

    }

    @Test
    public void depositingSample() {
        when(sut.EXTENDED_TO_BASKET_HEIGHT()).thenReturn(true);
        when(sut.SAMPLE_MODE()).thenReturn(true);
        when(sut.PREPARED_TO_DEPOSIT_SAMPLE()).thenReturn(false);
        when(sut.DEPOSITED_SAMPLE()).thenReturn(true);
        when(sut.DEPOSITED_SAMPLE()).thenReturn(false);

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.DEPOSITING_SAMPLE());
    }

    //Intake States
    @Test
    public void preparingToIntakeAndNotPreparedToIntake() {
        when(sut.PREPARED_TO_INTAKE()).thenReturn(false);

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndNotMovingToIntakePos() {
        when(sut.PREPARED_TO_INTAKE()).thenReturn(false);

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndDepositingSample() {
        when(sut.DEPOSITING_SAMPLE()).thenReturn(true);

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndDepositingSpecimen() {
        when(sut.DEPOSITING_SPECIMEN()).thenReturn(true);

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE());
    }

    @Test
    public void movingToIntakeIfPreparedToIntake() {
        when(sut.PREPARED_TO_INTAKE()).thenReturn(true);
        when(sut.MOVING_TO_INTAKE_POS()).thenReturn(true);

        sut.findTargetState(false, true, false, false, false, false, false);

        assertTrue(sut.MOVING_TO_INTAKE_POS());
    }

    @Test
    public void movingToMiniIntakeIfPreparedToIntake() {
        when(sut.PREPARED_TO_INTAKE()).thenReturn(true);

        sut.findTargetState(false, false, false, true, false, false, false);

        assertTrue(sut.MOVING_TO_MINI_INTAKE());
    }

    @Test
    public void retractingFromMiniIntakeIfMovedToMiniIntakeAndMonkeyPawMiniIntaked() {
        when(sut.MOVED_TO_MINI_INTAKE()).thenReturn(true);
        when(pawFSMMock.MINI_INTAKED()).thenReturn(true);

        sut.findTargetState(false, false, false, true, false, false, false);

        assertTrue(sut.RETRACTING_FROM_MINI_INTAKE());
    }


    /**
     * ------------------------------------updateState()-----------------------------------
     **/


    //Specimen: Preparing To Intake Specimen
    @Test
    public void whenArmIsFullyRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock, never()).retractToIntake();
        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());


    }


    @Test
    public void whenArmIsNotFullyRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    //Specimen: Intaking Specimen
    @Test
    public void whenPawHasNotIntaked() {
        sut.setCurrentState(LimbFSM.States.INTAKING_SPECIMEN);
        when(pawFSMMock.INTAKED_SPECIMEN()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock, never()).specimenPickupHeight();
        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void whenPawHasIntaked() {
        sut.setCurrentState(LimbFSM.States.INTAKING_SPECIMEN);
        when(pawFSMMock.INTAKED_SPECIMEN()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState(false, false, false, false, false, false, false, true);

        //TODO: Line 331 Indexing logic is flawed "indexIncrement()" does not guarantee specimenpickupheight.
        verify(armFSMMock).specimenPickupHeight();
        assertTrue(sut.INTAKED_SPECIMEN());
    }

    //Specimen: Extending Specimen
    @Test
    public void extendingSpecimenAndArmNotAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.EXTENDING_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false, false, false, false, false, false, false, true);
        assertTrue(sut.EXTENDING_SPECIMEN());
    }

    @Test
    public void extendingSpecimenAndArmAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.EXTENDING_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false, false, false, false, false, false, false, true);
        assertTrue(sut.EXTENDED_SPECIMEN());
    }

    //Specimen: Extended Specimen
    @Test
    public void extendedSpecimenAndArmNotAtTargetPos() {
        //TODO: There is no logic for extended specimen
        sut.setCurrentState(LimbFSM.States.EXTENDED_SPECIMEN);
        when(armFSMMock.AT_SUBMERSIBLE_HEIGHT()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false, false, false, false, false, false, false, true);
        assertTrue(sut.EXTENDING_SPECIMEN());
    }

    @Test
    public void extendedSpecimenAndArmAtTargetPos() {
        //TODO: There is no logic for extended specimen
        sut.setCurrentState(LimbFSM.States.EXTENDED_SPECIMEN);
        when(armFSMMock.atTargetPos()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState(false, false, false, false, false, false, false, true);

        assertFalse(sut.EXTENDED_SPECIMEN());
    }
    //Specimen: Depositing Specimen
    // TODO: All of depositing specimen logic is flawed. Indexing up does not ensure chamberLock height and it will not be armFSM.ATSUBHEIGHT(). It will be another height.
    @Test
    public void depositingSpecimenAndArmNotAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(armFSMMock.atTargetPos()).thenReturn(false);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).chamberLockHeight();

        assertTrue(sut.DEPOSITING_SPECIMEN());
    }

    @Test
    public void depositingSpecimenAndArmAtTargetPos() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(armFSMMock.atTargetPos()).thenReturn(true);
        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).chamberLockHeight();

        assertTrue(sut.DEPOSITED_SPECIMEN());
    }

    //Specimen: Deposited Specimen
    //TODO: Logic is also flawed here. There is one method call which returns a boolean, which is not in an if loop. Please look at the diagram.
    @Test
    public void depositedSpecimenAndPawIsGripped() {
        sut.setCurrentState(LimbFSM.States.DEPOSITED_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(pawFSMMock.RELAXING_AFTER_DEPOSIT()).thenReturn(false);
        sut.updateState(false, false, false, false, false, false, false, true);

        assertTrue(sut.DEPOSITED_SPECIMEN());
    }

    @Test
    public void depositedSpecimenAndPawIsUngripped() {
        sut.setCurrentState(LimbFSM.States.DEPOSITED_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(pawFSMMock.RELAXING_AFTER_DEPOSIT()).thenReturn(true);
        sut.updateState(false, false, false, false, false, false, false, true);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    //Sample: Preparing to Deposit Sample
    @Test
    public void armNotRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_DEPOSIT_SAMPLE);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock, never()).isShoulderTargetPosDepositBasketAngle();
        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());
    }

    @Test
    public void armIsRetracted() {
        //TODO: Line 217, the one with shoulderFSM.GOING_TO_BASKET() does not make sense either as it returns a boolean but is not inside an if statement.
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_DEPOSIT_SAMPLE);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock, never()).retractToIntake();
        verify(shoulderFSMMock).isShoulderTargetPosDepositBasketAngle();
        assertTrue(sut.PREPARED_TO_DEPOSIT_SAMPLE());
    }

    //Sample: Extending to basket height
    //TODO: The test I wrote was written according to the flowchart but the implementation looks different and it also includes the flawed indexIncrement() code.
    @Test
    public void extendingToSampleHeight() {
        sut.setCurrentState(LimbFSM.States.EXTENDING_TO_BASKET_HEIGHT);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.AT_BASKET_HEIGHT()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).moveToSelectedIndexPosition();
        assertTrue(sut.EXTENDED_TO_BASKET_HEIGHT());

    }

    @Test
    public void extendedSampleAndIndexChange() {
        sut.setCurrentState(LimbFSM.States.EXTENDED_TO_BASKET_HEIGHT);
        when(armFSMMock.atTargetPos()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(sut).checkIndexUpOrDown();

        assertFalse(sut.EXTENDING_TO_BASKET_HEIGHT());
    }
    //Sample: Depositing Sample
    //TODO: 1 of the 2 tests pass (which is not good) but the logic is flawed as here I also see another boolean function not in an if loop.
    @Test
    public void depositingSampleStateAndRelaxedAfterDeposit() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SAMPLE);
        when(pawFSMMock.RELAXED_AFTER_DEPOSIT()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);


        assertTrue(sut.DEPOSITED_SAMPLE());
    }

    @Test
    public void depositingSampleStateAndNotRelaxedAfterDeposit() {
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SAMPLE);
        when(pawFSMMock.RELAXED_AFTER_DEPOSIT()).thenReturn(false);

        sut.updateState(false, false, false, false, false, false, false, true);

        assertTrue(sut.DEPOSITING_SAMPLE());
    }

    //Intake: Preparing To Intake
    @Test
    public void preparingToIntakeAndShoulderAtIntakePosAndArmIsRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock, never()).retractToIntake();
        verify(shoulderFSMMock, never()).moveToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());


    }

    @Test
    public void preparingToIntakeAndShoulderNotAtIntakePosAndPawPreparedToIntake() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(false).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(false);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock, atLeastOnce()).moveToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndShoulderNotAtIntakePosAndPawRelaxedPosWithSample() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(false).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(false);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock, atLeastOnce()).moveToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndShoulderAtIntakePosAndArmNotRetractedAndRelaxedPosWithSample() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(false);


        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        verify(armFSMMock).moveToSafeHeight();
        verify(shoulderFSMMock, never()).moveToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void preparingToIntakeAndShoulderAtIntakePosAndArmNotRetractedAndPawPreparedToIntake() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE);
        when(shoulderFSMMock.AT_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false).thenReturn(true);
        when(pawFSMMock.PREPARED_TO_INTAKE_SAMPLE()).thenReturn(true);
        when(pawFSMMock.RELAXED_POS_WITH_SAMPLE()).thenReturn(false);


        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        verify(armFSMMock).moveToSafeHeight();
        verify(shoulderFSMMock, never()).moveToIntakeAngle();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }
    //Intake: Moving To Intake

    //TODO: Do not know the logic yet

    //Intake: Moving To Mini Intake
    //TODO: Again flawed indexIncrement logic and as this is from the flowchart the test is different.
    @Test
    public void movingToMiniIntake() {
        sut.setCurrentState(LimbFSM.States.MOVING_TO_MINI_INTAKE);
        when(armFSMMock.atTargetPos()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).moveToMiniIntake();
        assertTrue(sut.MOVED_TO_MINI_INTAKE());
    }

    //Intake: Retracting From Mini Intake
    @Test
    public void retractingFromMiniIntakeAndRelaxedMiniIntake() {
        sut.setCurrentState(LimbFSM.States.RETRACTING_FROM_MINI_INTAKE);
        when(pawFSMMock.RELAXED_MINI_INTAKE()).thenReturn(true);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock).retractToIntake();
        assertTrue(sut.PREPARED_TO_INTAKE());
    }

    @Test
    public void retractingFromMiniIntakeAndNotRelaxedMiniIntake() {
        sut.setCurrentState(LimbFSM.States.RETRACTING_FROM_MINI_INTAKE);
        when(pawFSMMock.RELAXED_MINI_INTAKE()).thenReturn(false);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);

        sut.updateState(false, false, false, false, false, false, false, true);

        verify(armFSMMock, never()).retractToIntake();
        assertTrue(sut.RETRACTING_FROM_MINI_INTAKE());
    }


}

