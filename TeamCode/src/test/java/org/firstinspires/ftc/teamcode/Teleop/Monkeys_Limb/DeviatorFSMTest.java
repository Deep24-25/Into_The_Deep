/*
package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.DeviatorFSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DeviatorFSMTest {
    DeviatorFSM sut;
    AxonServoWrapper axonServoWrapperMock = mock();
    Logger loggerMock = mock();
    private PIDController pidControllerMock = mock();



    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;

    private static final double PID_TOLERANCE = 0;


    @BeforeEach
    public void setUp() {
        sut = spy(new DeviatorFSM(axonServoWrapperMock,loggerMock,pidControllerMock));
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
    public void isDeviatorDeviatedRight() {
        when(sut.isTargetAngleToDeviateRight()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.RIGHT_DEVIATED());
    }
    @Test
    public void isDeviatorDeviatingRight() {
        when(sut.isTargetAngleToDeviateRight()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(false);

        sut.updateState();
        assertTrue(sut.RIGHT_DEVIATING());
    }
    @Test
    public void isDeviatorDeviatedLeft() {
        when(sut.isTargetAngleToDeviateRight()).thenReturn(false);
        when(sut.isTargetAngleToDeviateLeft()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.LEFT_DEVIATED());
    }
    @Test
    public void isDeviatorDeviatingLeft() {
        when(sut.isTargetAngleToDeviateRight()).thenReturn(false);
        when(sut.isTargetAngleToDeviateLeft()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(false);

        sut.updateState();
        assertTrue(sut.LEFT_DEVIATING());
    }
    @Test
    public void isDeviatorRelaxed() {
        when(sut.isTargetAngleToDeviateRight()).thenReturn(false);
        when(sut.isTargetAngleToDeviateLeft()).thenReturn(false);
        when(sut.isTargetAngleToRelax()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(true);
        sut.updateState();
        assertTrue(sut.RELAXED());
    }

    @Test
    public void isDeviatorRelaxing() {
        when(sut.isTargetAngleToDeviateRight()).thenReturn(false);
        when(sut.isTargetAngleToDeviateLeft()).thenReturn(false);
        when(sut.isTargetAngleToRelax()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(false);
        sut.updateState();
        assertTrue(sut.RELAXING());
    }

}
*/
