package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElbowFSMTest {
    ElbowFSM sut;
    AxonServoWrapper axonServoWrapperMock = mock();
    Logger loggerMock = mock();
    private PIDFController pidfControllerMock = mock();



    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;
    private static final double F = 0;

    private static final double PID_TOLERANCE = 0;


    @BeforeEach
    public void setUp() {
        sut = spy(new ElbowFSM(axonServoWrapperMock,loggerMock, pidfControllerMock));
    }

    @Test
    public void verifyPidMethods() {
        sut.updateState();
        verify(pidfControllerMock).setPIDF(P,I,D,F);
        verify(pidfControllerMock).setSetPoint(0);
        verify(pidfControllerMock).setTolerance(PID_TOLERANCE);
        verify(sut).updatePID();

    }

    @Test
    public void isElbowAtSampleIntakeFlex() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.FLEXED_TO_SAMPLE_INTAKE());
    }
    @Test
    public void isElbowFlexingToSampleIntakePos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(false);
        sut.updateState();
        assertTrue(sut.FLEXING_TO_SAMPLE_INTAKE());
    }
    @Test
    public void isElbowFlexedToSpecimenIntakePos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToSpecimenFlexedPos()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.FLEXED_TO_SPECIMEN_INTAKE());
    }
    @Test
    public void isElbowFlexingToSpecimenIntakePos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToSpecimenFlexedPos()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(false);
        sut.updateState();
        assertTrue(sut.FLEXING_TO_SPECIMEN_INTAKE());
    }
    @Test
    public void isElbowFlexedToDepositPos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToSpecimenFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToDepositFlexedPos()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.FLEXED_TO_DEPOSIT());
    }

    @Test
    public void isElbowFlexingToDepositPos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToSpecimenFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToDepositFlexedPos()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(false);
        sut.updateState();
        assertTrue(sut.FLEXING_TO_DEPOSIT());
    }

    @Test
    public void isElbowFlexedToRelaxPos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToSpecimenFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToDepositFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToRelax()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.RELAXED());
    }

    @Test
    public void isElbowFlexingToRelaxPos() {
        when(sut.isTargetAngleToSampleIntakeReadyFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToSpecimenFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToDepositFlexedPos()).thenReturn(false);
        when(sut.isTargetAngleToRelax()).thenReturn(true);
        when(pidfControllerMock.atSetPoint()).thenReturn(false);
        sut.updateState();
        assertTrue(sut.RELAXING());
    }

}
