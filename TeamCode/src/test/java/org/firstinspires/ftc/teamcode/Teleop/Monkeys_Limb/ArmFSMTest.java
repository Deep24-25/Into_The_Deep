package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmFSMTest {
    private ArmFSM sut;
    private ArmMotorsWrapper armMotorsWrapperMock = mock();
    private PIDFController pidfController = mock();
    @BeforeEach
    public void setup(){
        sut = spy(new ArmFSM(armMotorsWrapperMock,pidfController));
    }
    /**---------------------------updateState()---------------------------------**/
    @Test
    public void FULLY_RETRACTED(){
        doReturn(true).when(pidfController).atSetPoint();
        doReturn(true).when(sut).isTargetPosAtFullyRetractedHeight();
        sut.updateState();

        verify(armMotorsWrapperMock).readPositionInCM();
        assertTrue(sut.FULLY_RETRACTED());
    }
    @Test
    public void AT_BASKET_HEIGHT(){
        doReturn(true).when(pidfController).atSetPoint();
        doReturn(false).when(sut).isTargetPosAtFullyRetractedHeight();
        doReturn(true).when(sut).isTargetPosAtBasketHeight();

        sut.updateState();

        verify(armMotorsWrapperMock).readPositionInCM();
        assertTrue(sut.AT_BASKET_HEIGHT());
    }

    @Test
    public void AT_SUBMERSIBLE_HEIGHT(){
        doReturn(true).when(pidfController).atSetPoint();
        doReturn(false).when(sut).isTargetPosAtFullyRetractedHeight();
        doReturn(false).when(sut).isTargetPosAtBasketHeight();
        doReturn(true).when(sut).isTargetPosAtSubmersibleHeight();

        sut.updateState();

        verify(armMotorsWrapperMock).readPositionInCM();
        assertTrue(sut.AT_SUBMERSIBLE_HEIGHT());
    }

    @Test
    public void FULLY_EXTENDED(){
        doReturn(true).when(sut).isFullyExtended();
        doReturn(false).when(sut).isTargetPosAtFullyRetractedHeight();
        doReturn(false).when(sut).isTargetPosAtBasketHeight();
        doReturn(false).when(sut).isTargetPosAtSubmersibleHeight();

        sut.updateState();

        verify(armMotorsWrapperMock).readPositionInCM();
        assertTrue(sut.FULLY_EXTENDED());
    }

    @Test
    public void MOVING_ABOVE_SAFE_HEIGHT(){
        doReturn(false).when(pidfController).atSetPoint();
        doReturn(false).when(sut).isFullyExtended();
        doReturn(true).when(sut).isTargetPosAboveSafeHeight();

        sut.updateState();

        verify(armMotorsWrapperMock).readPositionInCM();
        verify(armMotorsWrapperMock).getLastReadPositionInCM();
        assertTrue(sut.MOVING_ABOVE_SAFE_HEIGHT());
    }

    @Test
    public void MOVING_BELOW_SAFE_HEIGHT(){
        doReturn(false).when(pidfController).atSetPoint();
        doReturn(false).when(sut).isFullyExtended();
        doReturn(false).when(sut).isTargetPosAboveSafeHeight();
        doReturn(true).when(sut).isTargetPosBelowSafeHeight();

        sut.updateState();

        verify(armMotorsWrapperMock).readPositionInCM();
        verify(armMotorsWrapperMock).getLastReadPositionInCM();
        assertTrue(sut.MOVING_BELOW_SAFE_HEIGHT());
    }
}
