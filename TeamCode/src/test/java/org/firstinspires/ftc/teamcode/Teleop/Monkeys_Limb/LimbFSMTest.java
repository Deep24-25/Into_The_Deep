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
        sut = spy(new LimbFSM(armFSMMock, shoulderFSMMock));
        armFSMMock = mock();
        shoulderFSMMock = mock();
        pawFSMMock = mock();
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
        doReturn(true).when(pawFSMMock.MINI_INTAKED());

        sut.findTargetState(false, false, false, true, false, false, false);

        assertTrue(sut.RETRACTING_FROM_MINI_INTAKE());
    }


    /**
     * ------------------------------------updateState()-----------------------------------
     **/
    //Specimen: Preparing To Intake Specimen
    @Test
    public void whenArmIsFullyRetracted(){
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState();

        verify(armFSMMock, never()).retractToIntake();
        verify(shoulderFSMMock).goToDepositPos();

        assertTrue(shoulderFSMMock.AT_DEPOSIT_CHAMBERS());
        assertTrue(pawFSMMock.PREPARED_TO_INTAKE_SPECIMEN());
        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }
    @Test
    public void whenArmIsNotFullyRetracted(){
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);



        sut.updateState();

        verify(armFSMMock, never()).retractToIntake();
        verify(shoulderFSMMock).goToDepositPos();

        assertTrue(shoulderFSMMock.AT_DEPOSIT_CHAMBERS());
        assertTrue(pawFSMMock.PREPARED_TO_INTAKE_SPECIMEN());
        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }
    //Specimen: Intaking Specimen
    @Test
    public void whenPawHasNotIntaked(){
        sut.setCurrentState(LimbFSM.States.INTAKING_SPECIMEN);
        when(pawFSMMock.INTAKED_SPECIMEN()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState();

        verify(armFSMMock, never()).goToSpecimenPickupHeight();
        assertFalse(sut.INTAKED_SPECIMEN());
        assertFalse(armFSMMock.atSpecimenPickupHeight());
    }
    @Test
    public void whenPawHasIntaked(){
        sut.setCurrentState(LimbFSM.States.INTAKING_SPECIMEN);
        when(pawFSMMock.INTAKED_SPECIMEN()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);


        sut.updateState();

        verify(armFSMMock).goToSpecimenPickupHeight();
        assertTrue(sut.INTAKED_SPECIMEN());
        assertTrue(armFSMMock.atSpecimenPickupHeight());
    }
    //Specimen: Extending Specimen
    @Test
    public void extendingSpecimenAndArmNotAtTargetPos(){
        sut.setCurrentState(LimbFSM.States.EXTENDING_TO_SPECIMEN);
        when(armFSMMock.atTargetPos()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState();

        verify(sut).checkIndexUpOrDown();

        assertFalse(sut.EXTENDED_SPECIMEN());
    }

    @Test
    public void extendingSpecimenAndArmAtTargetPos(){
        sut.setCurrentState(LimbFSM.States.EXTENDING_TO_SPECIMEN);
        when(armFSMMock.atTargetPos()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState();

        verify(sut).checkIndexUpOrDown();

        assertTrue(sut.EXTENDED_SPECIMEN());
    }
    //Specimen: Extended Specimen
    @Test
    public void extendedSpecimenAndArmNotAtTargetPos(){
        sut.setCurrentState(LimbFSM.States.EXTENDED_SPECIMEN);
        when(armFSMMock.atTargetPos()).thenReturn(false);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState();

        verify(sut).checkIndexUpOrDown();

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }

    @Test
    public void extendedSpecimenAndArmAtTargetPos(){
        sut.setCurrentState(LimbFSM.States.EXTENDED_SPECIMEN);
        when(armFSMMock.atTargetPos()).thenReturn(true);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);

        sut.updateState();

        verify(sut).checkIndexUpOrDown();

        assertFalse(sut.EXTENDING_TO_SPECIMEN());
    }
    //Specimen: Depositing Specimen

    @Test
    public void depositingSpecimenAndArmNotAtTargetPos(){
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(armFSMMock.atTargetPos()).thenReturn(false);

        sut.updateState();

        verify(armFSMMock).moveToChamberLockHeight();

        assertTrue(sut.DEPOSITING_SPECIMEN());
    }

    @Test
    public void depositingSpecimenAndArmAtTargetPos(){
        sut.setCurrentState(LimbFSM.States.DEPOSITING_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(armFSMMock.atTargetPos()).thenReturn(true);
        sut.updateState();

        verify(armFSMMock).moveToChamberLockHeight();

        assertTrue(sut.DEPOSITED_SPECIMEN());
    }
    //Specimen: Deposited Specimen
    @Test
    public void depositedSpecimenAndPawIsGripped(){
        sut.setCurrentState(LimbFSM.States.DEPOSITED_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(pawFSMMock.RELAXING_AFTER_DEPOSIT()).thenReturn(false);
        sut.updateState();

        assertTrue(sut.DEPOSITED_SPECIMEN());
    }
    @Test
    public void depositedSpecimenAndPawIsUngripped(){
        sut.setCurrentState(LimbFSM.States.DEPOSITED_SPECIMEN);
        sut.setCurrentMode(LimbFSM.Mode.SPECIMEN_MODE);
        when(pawFSMMock.RELAXING_AFTER_DEPOSIT()).thenReturn(true);
        sut.updateState();

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }
    //Sample: Preparing to Deposit Sample
    @Test
    public void armNotRetracted() {
        sut.setCurrentState(LimbFSM.States.PREPARING_TO_DEPOSIT_SAMPLE);
        sut.setCurrentMode(LimbFSM.Mode.SAMPLE_MODE);
        when(armFSMMock.FULLY_RETRACTED()).thenReturn(false);

        sut.updateState();

        verify(armFSMMock).retractToIntake();
        verify(shoulderFSMMock, never()).isShoulderTargetPosDepositBasketAngle();
        assertTrue(sut.PREPARING_TO_DEPOSIT_SAMPLE());
    }

}