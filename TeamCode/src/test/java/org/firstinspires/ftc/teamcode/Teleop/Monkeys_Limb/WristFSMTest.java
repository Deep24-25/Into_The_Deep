package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.FingerFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class WristFSMTest {
    WristFSM sut;
    AxonServoWrapper axonServoWrapperMock = mock();
    Logger loggerMock = mock();
    private PIDController pidControllerMock = mock();



    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;

    private static final double PID_TOLERANCE = 0;


    @BeforeEach
    public void setUp() {
        sut = spy(new WristFSM(axonServoWrapperMock,loggerMock,pidControllerMock));
    }

    @Test
    public void verifyPidMethods() {
        sut.updateState();
        verify(pidControllerMock).setPID(P,I,D);
        verify(pidControllerMock).setSetPoint(0);
        verify(pidControllerMock).setTolerance(PID_TOLERANCE);
        verify(sut).updatePID();

    }

    @Test
    public void isWristFlexed() {
        when(sut.isTargetAngleToFlex()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(true);

        sut.updateState();
        assertTrue(sut.FLEXED());
    }

    @Test
    public void isWristFlexing() {
        when(sut.isTargetAngleToFlex()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(false);

        sut.updateState();
        assertTrue(sut.FLEXING());
    }

    @Test
    public void isWristRelaxed() {
        when(sut.isTargetAngleToFlex()).thenReturn(false);
        when(sut.isTargetAngleToRelax()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(true);

        sut.updateState();
        assertTrue(sut.RELAXED());
    }

    @Test
    public void isWristRelaxing() {
        when(sut.isTargetAngleToFlex()).thenReturn(false);
        when(sut.isTargetAngleToRelax()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(false);

        sut.updateState();
        assertTrue(sut.RELAXING());
    }

}
