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

import org.firstinspires.ftc.teamcode.pedroPathing.MainAuto;


@Config
public class FieldCentricDrive {
    private final MecanumDrive mecanumDrive;
    private final Logger logger;
    private boolean headingLock = false;

    public final double SPEC_HEADING = 0;

    public static double P = 0.084, I = 0, D = 0, F = 1;
    private final PIDController pidController = new PIDController(P, I, D);
    private HWMap hwMap;

    private double turnSpeed = 0;
    private double feedforward = 0;

    public static int TOTAL_HEADING = 180;
    public FieldCentricDrive(HWMap hwMap, Logger logger) {
        mecanumDrive = hwMap.getMecanumDrive();
        this.hwMap = hwMap;
        this.logger = logger;

    }


    public void drive(double strafe, double forward, double turn, double heading) {
        pidController.setPID(P, I, D);

        if (headingLock) {
            if(MainAuto.basketAuto)
                TOTAL_HEADING = 180;
            else
                TOTAL_HEADING = 360;
            double normalizedTargetPos = SPEC_HEADING / TOTAL_HEADING;
            double normalizedHeading = heading / TOTAL_HEADING;
            turnSpeed = pidController.calculate(normalizedHeading, normalizedTargetPos) + feedforward;
            if (hwMap.getFrontRightMotor().getVelocity() < 0.05) {
                turnSpeed = turnSpeed/F;
            }
            this.mecanumDrive.driveFieldCentric(strafe, forward, turnSpeed, heading);
        } else
            this.mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);

    }*/

    public void drive(double strafe, double forward, double turn, double heading) {
        if (MainAuto.basketAuto)
            TOTAL_HEADING = Math.toRadians(180);
        else
            TOTAL_HEADING = Math.toRadians(360);

        if (headingLock) {
            double headingError = TOTAL_HEADING - Math.toRadians(heading);
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);
            if (Math.abs(headingError) < Math.toRadians(2)) {
                turnSpeed = 0;
            } else {
                turnSpeed = pidController.calculate(headingError);
            }
            follower.setTeleOpMovementVectors(-strafe, forward, turnSpeed);
        } else {
            follower.setTeleOpMovementVectors(-strafe, forward, turn);
        }
    }

    public void setHeadingLock(boolean headingLock) {
        this.headingLock = headingLock;
    }

    public void log() {
        logger.log("-", turnSpeed, Logger.LogLevels.DEBUG);
    }
}

