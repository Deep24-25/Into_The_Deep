package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;


import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.verify;


import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.DeviatorFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.FingerFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class MonkeyPawFSMTest {
    private MonkeyPawFSM sut;

    private FingerFSM fingerFSM = mock();
    private DeviatorFSM deviatorFSM = mock();
    private WristFSM wristFSM = mock();
    private ElbowFSM elbowFSM = mock();

    private LimbFSM limbFSM = mock();
    private Logger logger = mock();

    @BeforeEach
    public void setup() {
        sut = spy(new MonkeyPawFSM(logger,limbFSM,fingerFSM,deviatorFSM,wristFSM,elbowFSM));
    }

    // FIND TARGET STATE TESTS
    @Test
    public void checkIfStateIsPreparingToIntakeSample() {
        when(limbFSM.PREPARING_TO_INTAKE()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.PREPARING_TO_INTAKE_SAMPLE());
    }
    @Test
    public void checkIfStateIsIntakingSample() {
        when(limbFSM.MOVED_TO_INTAKE_POS()).thenReturn(true);
        doReturn(true).when(sut).PREPARED_TO_INTAKE_SAMPLE();
        sut.findTargetState(false,false,false);
        assertTrue(sut.INTAKING_SAMPLE());
    }

    @Test
    public void checkIfStateIsDepositingSampleToHP() {
        sut.findTargetState(true,false,false);
        assertTrue(sut.DEPOSITING_SAMPLE_TO_HP());
    }

    @Test
    public void checkIfStateIsPreparingToIntakeSpecimen() {
        when(limbFSM.PREPARING_TO_INTAKE_SPECIMEN()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }

    @Test
    public void checkIfStateIsIntakingSpecimen() {
        when(limbFSM.INTAKING_SPECIMEN()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.INTAKING_SPECIMEN());
    }

    @Test
    public void checkIfStateIsDepositingSample() {
        when(limbFSM.DEPOSITING_SAMPLE()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.DEPOSITING_SAMPLE());
    }

    @Test
    public void checkIfStateIsDepositingSpecimen() {
        when(limbFSM.DEPOSITED_SPECIMEN()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.DEPOSITING_SPECIMEN());
    }

    @Test
    public void checkIfStateIsMiniIntaking() {
        when(limbFSM.MOVED_TO_MINI_INTAKE()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.MINI_INTAKING());
    }

    @Test
    public void checkIfStateIsRelaxingFromMiniIntake() {
        when(limbFSM.RETRACTING_FROM_MINI_INTAKE()).thenReturn(true);
        sut.findTargetState(false,false,false);
        assertTrue(sut.RELAXING_MINI_INTAKE());
    }


    //UPDATE STATE TESTS
    @Test
    public void elbowIsRelaxingInPreparingToIntakeSample() {
        when(elbowFSM.RELAXED()).thenReturn(false);
        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SAMPLE);
        sut.updateState(false,false,false);
        verify(elbowFSM).relax();
    }
    @Test
    public void wristIsRelaxingInPreparingToIntakeSample() {
        when(elbowFSM.RELAXED()).thenReturn(true);
        when(wristFSM.RELAXED()).thenReturn(false);

        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SAMPLE);
        sut.updateState(false,false,false);

        verify(wristFSM).relax();
    }

    @Test
    public void deviatorIsRelaxingInPreparingToIntakeSample() {
        when(elbowFSM.RELAXED()).thenReturn(true);
        when(wristFSM.RELAXED()).thenReturn(true);
        when(deviatorFSM.RELAXED()).thenReturn(false);

        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SAMPLE);
        sut.updateState(false,false,false);

        verify(deviatorFSM).relax();
    }

    @Test
    public void fingerIsRelaxingInPreparingToIntakeSample() {
        when(elbowFSM.RELAXED()).thenReturn(true);
        when(wristFSM.RELAXED()).thenReturn(true);
        when(deviatorFSM.RELAXED()).thenReturn(true);
        when(fingerFSM.RELEASED()).thenReturn(false);

        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SAMPLE);
        sut.updateState(false,false,false);

        verify(fingerFSM).release();
    }

    @Test
    public void whenAllMechanismsRelaxedStateIsPreparedToIntakeSample() {
        when(elbowFSM.RELAXED()).thenReturn(true);
        when(wristFSM.RELAXED()).thenReturn(true);
        when(deviatorFSM.RELAXED()).thenReturn(true);
        when(fingerFSM.RELEASED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SAMPLE);
        sut.updateState(false,false,false);

        assertTrue(sut.PREPARED_TO_INTAKE_SAMPLE());
    }

    @Test
    public void whenXPressedVerifyDeviatorActionInStateIntakeSample() {
        sut.setState(MonkeyPawFSM.States.INTAKING_SAMPLE);
        sut.updateState(false,true,false);
        verify(deviatorFSM).deviateRight();
    }

    @Test
    public void whenBPressedVerifyDeviatorActionInStateIntakeSample() {
        sut.setState(MonkeyPawFSM.States.INTAKING_SAMPLE);
        sut.updateState(false,false,true);
        verify(deviatorFSM).deviateLeft();
    }

    @Test
    public void verifyElbowAndWristFlexInStateIntakeSample() {
        when(deviatorFSM.RIGHT_DEVIATED()).thenReturn(true);
        when(deviatorFSM.LEFT_DEVIATED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.INTAKING_SAMPLE);
        sut.updateState(false,false,false);
        verify(elbowFSM).flexToSamplePos();
        verify(wristFSM).flex();
    }

    @Test
    public void verifyFingerGripsInStateIntakeSample() {
        when(deviatorFSM.RIGHT_DEVIATED()).thenReturn(true);
        when(deviatorFSM.LEFT_DEVIATED()).thenReturn(true);
        when(elbowFSM.FLEXED_TO_SAMPLE_INTAKE()).thenReturn(true);
        when(wristFSM.FLEXED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.INTAKING_SAMPLE);
        sut.updateState(true,false,false);
        verify(fingerFSM).grip();
    }

    @Test
    public void whenFingerIsGrippedInIntakingSampleStateBecomesRelaxingWithSample() {
        when(deviatorFSM.RIGHT_DEVIATED()).thenReturn(true);
        when(deviatorFSM.LEFT_DEVIATED()).thenReturn(true);
        when(elbowFSM.FLEXED_TO_SAMPLE_INTAKE()).thenReturn(true);
        when(wristFSM.FLEXED()).thenReturn(true);
        when(fingerFSM.GRIPPED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.INTAKING_SAMPLE);
        sut.updateState(true,false,false);
        assertTrue(sut.RELAXING_WITH_SAMPLE());
    }

    @Test
    public void verifyMechanismsRelaxingInStateRelaxingWithSample() {
        sut.setState(MonkeyPawFSM.States.RELAXING_WITH_SAMPLE);
        sut.updateState(false,false,false);

        verify(elbowFSM).relax();
        verify(wristFSM).relax();
        verify(deviatorFSM).relax();
    }

    @Test
    public void whenAllMechanismsRelaxedStateIsRelaxedPosWithSample() {
        when(elbowFSM.RELAXED()).thenReturn(true);
        when(wristFSM.RELAXED()).thenReturn(true);
        when(deviatorFSM.RELAXED()).thenReturn(true);
        when(fingerFSM.GRIPPED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.RELAXING_WITH_SAMPLE);
        sut.updateState(false,false,false);

        assertTrue(sut.RELAXED_POS_WITH_SAMPLE());
    }

    @Test
    public void verifyFingerReleasedInDepositingSampleToHP() {
        sut.setState(MonkeyPawFSM.States.DEPOSITING_SAMPLE_TO_HP);
        sut.updateState(false,false,false);

        verify(fingerFSM).release();
    }

    @Test
    public void whenFingerReleasedStateIsDepositedSampleToHP() {
        when(fingerFSM.RELEASED()).thenReturn(true);
        sut.setState(MonkeyPawFSM.States.DEPOSITING_SAMPLE_TO_HP);
        sut.updateState(false,false,false);

        assertTrue(sut.DEPOSITED_SAMPLE_TO_HP());
    }

    @Test
    public void verifyElbowIsFlexInStateDepositingSample() {
        sut.setState(MonkeyPawFSM.States.DEPOSITING_SAMPLE);
        sut.updateState(false,false,false);
        verify(elbowFSM).flexToDepositPos();
    }

    @Test
    public void whenBasketDepositCompleteStateBecomesRelaxingAfterDeposit() {
        when(elbowFSM.FLEXED_TO_DEPOSIT()).thenReturn(true);
        when(fingerFSM.RELEASED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.DEPOSITING_SAMPLE);
        sut.updateState(false,false,false);
        assertTrue(sut.RELAXING_AFTER_DEPOSIT());
    }

    @Test
    public void verifyElbowRelaxingInStateRelaxingAfterDeposit() {
        sut.setState(MonkeyPawFSM.States.RELAXING_AFTER_DEPOSIT);
        sut.updateState(false,false,false);
        verify(elbowFSM).relax();
    }

    @Test
    public void whenElbowRelaxedAfterDepositStateIsRelaxedAfterDeposit() {
        when(elbowFSM.RELAXED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.RELAXING_AFTER_DEPOSIT);
        sut.updateState(false,false,false);
        assertTrue(sut.RELAXED_AFTER_DEPOSIT());
    }

    @Test
    public void verifyElbowFlexsToSpecimenPosInStatePreparingToIntakeSpecimen() {
        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        sut.updateState(false,false,false);
        verify(elbowFSM).flexToSpecimenPos();
    }

    @Test
    public void whenElbowIsFlexedStateIsPreparedToIntakeSpecimen() {
        when(elbowFSM.FLEXED_TO_SPECIMEN_INTAKE()).thenReturn(true);
        sut.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
        sut.updateState(false,false,false);
        assertTrue(sut.PREPARED_TO_INTAKE_SPECIMEN());
    }


    @Test
    public void verifyFingerGripsInStateIntakingSpecimen() {
        sut.setState(MonkeyPawFSM.States.INTAKING_SPECIMEN);
        sut.updateState(false,false,false);
        verify(fingerFSM).grip();
    }

    @Test
    public void whenFingerGripsSpecimenStateBecomesIntakedSpecimen() {
        when(fingerFSM.GRIPPED()).thenReturn(true);
        sut.setState(MonkeyPawFSM.States.INTAKING_SPECIMEN);
        sut.updateState(false,false,false);
        assertTrue(sut.INTAKED_SPECIMEN());
    }
    @Test
    public void verifyElbowFlexsInStateDepositingSpecimen() {
        sut.setState(MonkeyPawFSM.States.DEPOSITING_SPECIMEN);
        sut.updateState(false,false,false);
        verify(elbowFSM).flexToDepositPos();
    }

    @Test
    public void verifyFingerReleasesInStateDepositingSpecimen() {
        when(elbowFSM.FLEXED_TO_DEPOSIT()).thenReturn(true);
        sut.setState(MonkeyPawFSM.States.DEPOSITING_SPECIMEN);
        sut.updateState(false,false,false);
        verify(fingerFSM).release();
    }


    @Test
    public void whenFingerReleasesSpecimenStateBecomesDepositedSpecimen() {
        when(fingerFSM.RELEASED()).thenReturn(true);
        when(elbowFSM.FLEXED_TO_DEPOSIT()).thenReturn(true);
        sut.setState(MonkeyPawFSM.States.DEPOSITING_SPECIMEN);
        sut.updateState(false,false,false);
        assertTrue(sut.DEPOSITED_SPECIMEN());
    }


    @Test
    public void verifyElbowAndWristFlexToIntakeInMiniIntaking() {
        sut.setState(MonkeyPawFSM.States.MINI_INTAKING);
        sut.updateState(false,false,false);
        verify(elbowFSM).flexToSamplePos();
        verify(wristFSM).flex();
    }

    @Test
    public void whenElbowAndWristDownStateIsMiniIntaked() {
        when(elbowFSM.FLEXED_TO_SAMPLE_INTAKE()).thenReturn(true);
        when(wristFSM.FLEXED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.MINI_INTAKING);
        sut.updateState(false,false,false);
        assertTrue(sut.MINI_INTAKED());

    }

    @Test public void whenXPressedInMiniIntakedToAlterDeviation() {
        sut.setState(MonkeyPawFSM.States.MINI_INTAKED);
        sut.updateState(false,true,false);
        verify(deviatorFSM).deviateRight();
    }

    @Test public void whenBPressedInMiniIntakedToAlterDeviation() {
        sut.setState(MonkeyPawFSM.States.MINI_INTAKED);
        sut.updateState(false,false,true);
        verify(deviatorFSM).deviateLeft();
    }

    @Test
    public void verifyFingerGripsWhenRelaxingMiniIntake() {
        sut.setState(MonkeyPawFSM.States.RELAXING_MINI_INTAKE);
        sut.updateState(true,false,false);
        verify(fingerFSM).grip();
    }

    @Test
    public void relaxElbowAndWristWhenFingerIsGrippedInStateRelaxingMiniIntake() {
        when(fingerFSM.GRIPPED()).thenReturn(true);
        sut.setState(MonkeyPawFSM.States.RELAXING_MINI_INTAKE);
        sut.updateState(false,false,false);

        verify(elbowFSM).relax();
        verify(wristFSM).relax();
    }

    @Test
    public void whenAllMiniIntakeMechanismsRelaxStateBecomesRelaxedMiniIntake() {
        when(fingerFSM.GRIPPED()).thenReturn(true);
        when(elbowFSM.RELAXED()).thenReturn(true);
        when(wristFSM.RELAXED()).thenReturn(true);

        sut.setState(MonkeyPawFSM.States.RELAXING_MINI_INTAKE);
        sut.updateState(false,false,false);

        assertTrue(sut.RELAXED_MINI_INTAKE());
    }

}

