package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;


public class  FieldCentricDrive {
    private final MecanumDrive mecanumDrive;


    public FieldCentricDrive(HWMap hwMap) {
        mecanumDrive = hwMap.getMecanumDrive();
    }

    public void drive (double strafe, double forward, double turn, double heading){
            this.mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }
}

