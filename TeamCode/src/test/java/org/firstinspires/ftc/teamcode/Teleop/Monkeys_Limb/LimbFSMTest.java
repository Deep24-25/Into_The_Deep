package org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.spy;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

class LimbFSMTest {
    private LimbFSM sut;

    @BeforeEach
    public void setup() {
        sut = spy(new LimbFSM());
    }

    /**
     * ------------------------------------findTargetState()-----------------------------------
     **/
    @Test
    public void preparingToIntakeSpecimenState() {
        sut.findTargetState(true, Mockito.anyBoolean(), Mockito.anyBoolean(), Mockito.anyBoolean());
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.PREPARED_TO_INTAKE());

        assertTrue(sut.PREPARING_TO_INTAKE_SPECIMEN());
    }
    @Test
    public void intakingSpecimenState(){
        sut.findTargetState(true, Mockito.anyBoolean(), Mockito.anyBoolean(), Mockito.anyBoolean());
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.PREPARED_TO_INTAKE_SPECIMEN());

        assertTrue(sut.INTAKING_SPECIMEN());
    }
    @Test
    public void extendingToSpecimenState(){
        sut.findTargetState(true, Mockito.anyBoolean(), Mockito.anyBoolean(), Mockito.anyBoolean());
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.INTAKED_SPECIMEN());

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }
    @Test
    public void depositingSpecimenState(){
        sut.findTargetState(true, Mockito.anyBoolean(), Mockito.anyBoolean(), Mockito.anyBoolean());
        doReturn(true).when(sut.SPECIMEN_MODE());
        doReturn(true).when(sut.EXTENDED_SPECIMEN());

        assertTrue(sut.EXTENDING_TO_SPECIMEN());
    }
    
}