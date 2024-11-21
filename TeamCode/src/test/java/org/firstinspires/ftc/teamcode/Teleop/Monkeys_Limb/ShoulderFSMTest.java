package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ArmMotorsWrapper;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShoulderFSMTest {


    private ShoulderFSM sut;
    private ShoulderWrapper shoulderWrapperMock = mock();
    private HWMap hwMap = mock();
    @BeforeEach
    public void setup(){
        sut = spy(new ShoulderFSM());
    }
    /**---------------------------updateState()---------------------------------**/

    @Test
    public void GOING_TO_CHAMBER(){
        doReturn(true).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderCurrentPosDepositChamberAngle();

        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.GOING_TO_CHAMBER());

    }

    @Test
    public void AT_DEPOSIT_CHAMBERS(){
        doReturn(true).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(true).when(sut).isShoulderCurrentPosDepositChamberAngle();

        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.AT_DEPOSIT_CHAMBERS());

    }

    @Test
    public void GOING_TO_INTAKE(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderCurrentPosDepositChamberAngle();
        doReturn(true).when(sut).isShoulderTargetPosIntakeAngle();
        doReturn(false).when(sut).isShoulderCurrentPosIntakeAngle();

        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.GOING_TO_INTAKE());

    }

    @Test
    public void AT_INTAKE(){
        doReturn(true).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(true).when(sut).isShoulderCurrentPosDepositChamberAngle();
        doReturn(true).when(sut).isShoulderTargetPosIntakeAngle();
        doReturn(true).when(sut).isShoulderCurrentPosIntakeAngle();

        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.AT_INTAKE());

    }
    @Test
    public void GOING_TO_BASKET(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderCurrentPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderTargetPosIntakeAngle();
        doReturn(false).when(sut).isShoulderCurrentPosIntakeAngle();
        doReturn(true).when(sut).isShoulderTargetPosDepositBasketAngle();
        doReturn(false).when(sut).isShoulderCurrentPosDepositBasketAngle();

        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.GOING_TO_BASKET());

    }

    @Test
    public void AT_BASKET_DEPOSIT(){
        doReturn(false).when(sut).isShoulderTargetPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderCurrentPosDepositChamberAngle();
        doReturn(false).when(sut).isShoulderTargetPosIntakeAngle();
        doReturn(false).when(sut).isShoulderCurrentPosIntakeAngle();
        doReturn(true).when(sut).isShoulderTargetPosDepositBasketAngle();
        doReturn(true).when(sut).isShoulderCurrentPosDepositBasketAngle();
        sut.updateState();

        verify(shoulderWrapperMock).readAngle();
        assertTrue(sut.AT_BASKET_DEPOSIT());

    }
}