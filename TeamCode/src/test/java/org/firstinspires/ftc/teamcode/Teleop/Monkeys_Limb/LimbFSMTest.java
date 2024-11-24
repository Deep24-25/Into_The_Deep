package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

class LimbFSMTest {
    private LimbFSM sut;
    private ArmFSM armFSMMock;
    private ShoulderFSM shoulderFSMMock;


    @BeforeEach
    public void setup() {
        sut = spy(new LimbFSM());
        armFSMMock = mock();
        shoulderFSMMock = mock();
    }

    /**
     * ------------------------------------findTargetState()-----------------------------------
     **/

    // Specimen State Tests
    @Test
    public void preparingToIntakeSpecimenState() {
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.PREPARED_TO_INTAKE());

        sut.findTargetState(true, false, false, false, false, false, false);

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

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
        //doReturn(true).when(monkeyPaw.MOVED_TO_MINI_INTAKE());

        sut.findTargetState(false, false, false, true, false, false, false);

        assertTrue(sut.RETRACTING_FROM_MINI_INTAKE());
    }


    /**
     * ------------------------------------updateState()-----------------------------------
     **/
    //Specimen: Preparing To Intake Specimen
   /* @Test
    public void whenArmIsFullyRetracted(){
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        sut.updateState();

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock).goToDepositPos();

        assertTrue(armFSMMock.FULLY_RETRACTED());
        assertTrue(pawFSM.PREPARED_TO_INTAKE_SPECIMEN);


    }
    */


}