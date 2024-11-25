package org.firstinspires.ftc.teamcode.Teleop.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Core.HWMap;
public class ShoulderWrapper {
    private final Motor shoulderMotor;
    private static double lastReadAngle;
    private static final int ENCODER_OFFSET = 360;
    public ShoulderWrapper(HWMap hwMap) {
        shoulderMotor = hwMap.getPivotMotor();
    }

    public void set(double power){
            shoulderMotor.set(power);
        }

        public double readAngle() {
            lastReadAngle = shoulderMotor.getCurrentPosition();
            return lastReadAngle;
        }
    public double getLastReadAngle() {
        return lastReadAngle;
    }
}
