package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HWMap {
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;

    public HWMap(HardwareMap hwMap){
        colorSensor1 = hwMap.get(RevColorSensorV3.class, "CS1");
        colorSensor2 = hwMap.get(RevColorSensorV3.class, "CS2");

    }

    public RevColorSensorV3 getColorSensor1() {
        return colorSensor1;
    }

    public RevColorSensorV3 getColorSensor2() {
        return colorSensor2;
    }
}
