package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.FingerFSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FingerFSMTest {
    FingerFSM sut;
    AxonServoWrapper axonServoWrapperMock = mock();
    Logger loggerMock = mock();
    private PIDController pidControllerMock = mock();



    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;

    private static final double PID_TOLERANCE = 0;


    @BeforeEach
    public void setUp() {
        sut = spy(new FingerFSM(axonServoWrapperMock,loggerMock,pidControllerMock));
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
    public void isFingerGripped() {
        when(sut.isTargetAngleToGrip()).thenReturn(true);
        when(axonServoWrapperMock.getLastReadPos()).thenReturn(5.0);

        sut.updateState();
        assertTrue(sut.GRIPPED());
    }


    @Test
    public void isFingerGripping() {
        when(sut.isTargetAngleToGrip()).thenReturn(true);
        when(axonServoWrapperMock.getLastReadPos()).thenReturn(-5.0);

        sut.updateState();
        assertTrue(sut.GRIPPING());
    }

    @Test
    public void isFingerReleased() {
        when(sut.isTargetAngleToGrip()).thenReturn(false);
        when(sut.isTargetAngleToRelease()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(true);

        sut.updateState();
        assertTrue(sut.RELEASED());
    }

    @Test
    public void isFingerReleasing() {
        when(sut.isTargetAngleToGrip()).thenReturn(false);
        when(sut.isTargetAngleToRelease()).thenReturn(true);
        when(pidControllerMock.atSetPoint()).thenReturn(false);

        sut.updateState();
        assertTrue(sut.RELEASING());
    }

}
