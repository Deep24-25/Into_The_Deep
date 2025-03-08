package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;


@Config
public class FieldCentricDrive {
    private final MecanumDrive mecanumDrive;
    private final Logger logger;
    private boolean headingLock = false;
    private boolean canHeadingLock = true;

    public final double SPEC_HEADING = 0;

    public static double P = 0.084, I = 0, D = 0, F = 1;
    private final PIDController pidController = new PIDController(P, I, D);
    private HWMap hwMap;

    private double turnSpeed = 0;
    private double feedforward = 0;

    public FieldCentricDrive(HWMap hwMap, Logger logger) {
        mecanumDrive = hwMap.getMecanumDrive();
        this.hwMap = hwMap;
        this.logger = logger;

    }


    public void drive(double strafe, double forward, double turn, double heading) {
        pidController.setPID(P, I, D);
        if (headingLock && canHeadingLock) {
            double normalizedTargetPos = SPEC_HEADING / 360;
            double normalizedHeading = heading / 360;
            turnSpeed = pidController.calculate(normalizedHeading, normalizedTargetPos) + feedforward;
            if (hwMap.getFrontRightMotor().getVelocity() < 0.05) {
                turnSpeed = turnSpeed/F;
            }
            this.mecanumDrive.driveFieldCentric(strafe, forward, turnSpeed, heading);
        } else
            this.mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);

    }

    public void setHeadingLock(boolean headingLock) {
        this.headingLock = headingLock;
    }

    public void setCanHeadingLock(boolean canHeadingLock) {
        this.canHeadingLock = canHeadingLock;
    }

    public boolean isCanHeadingLock() {
        return canHeadingLock;
    }

    public void log() {
        logger.log("-", turnSpeed, Logger.LogLevels.DEBUG);
    }
}

