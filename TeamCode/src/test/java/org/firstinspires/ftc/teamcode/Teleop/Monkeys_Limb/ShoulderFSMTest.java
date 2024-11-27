package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShoulderFSMTest {


    private ShoulderFSM sut;
    private ShoulderWrapper shoulderWrapperMock = mock();
    private PIDFController pidfController = mock();
    private static double P = 0;
    private static double I = 0;
    private static double D = 0;
    private static double F = 0;
    private double TOLERANCE = 3;


    @BeforeEach
    public void setup(){
        sut = spy(new ShoulderFSM(shoulderWrapperMock,pidfController));
    }
    /**---------------------------updateState()---------------------------------**/

    @Test
    public void GOING_TO_CHAMBER(){
        doReturn(true).when(sut).isShoulderTargetPosDepositChamberAngle();
        when(pidfController.atSetPoint()).thenReturn(false);

        sut.updateState();
        verify(pidfController).setPIDF(P, I, D, F);
        verify(pidfController).setSetPoint(0);
        verify(pidfController).setTolerance(TOLERANCE);
        assertTrue(sut.GOING_TO_CHAMBER());

    }

    @Test
    public void AT_DEPOSIT_CHAMBERS(){
        doReturn(true).when(sut).isShoulderTargetPosDepositChamberAngle();
        when(pidfController.atSetPoint()).thenReturn(true);

        sut.updateState();

        assertTrue(sut.AT_DEPOSIT_CHAMBERS());

    }

    @Test
    public void GOING_TO_INTAKE(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(true).when(sut).isShoulderTargetPosSampleIntakeAngle();
        doReturn(false).when(pidfController).atSetPoint();
        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.GOING_TO_INTAKE());

    }

    @Test
    public void AT_INTAKE(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(true).when(sut).isShoulderTargetPosSampleIntakeAngle();
        doReturn(true).when(pidfController).atSetPoint();
        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.AT_INTAKE());

    }
    @Test
    public void GOING_TO_BASKET(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderTargetPosSampleIntakeAngle();
        doReturn(true).when(sut).isShoulderTargetPosDepositBasketAngle();
        doReturn(false).when(pidfController).atSetPoint();
        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.GOING_TO_BASKET());

    }

    @Test
    public void AT_BASKET_DEPOSIT(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderTargetPosSampleIntakeAngle();
        doReturn(true).when(sut).isShoulderTargetPosDepositBasketAngle();
        doReturn(true).when(pidfController).atSetPoint();
        sut.updateState();
        verify(shoulderWrapperMock).readAngle();

        assertTrue(sut.AT_BASKET_DEPOSIT());

    }
}