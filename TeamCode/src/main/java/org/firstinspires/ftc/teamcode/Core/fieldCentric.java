package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;


@TeleOp(name="fieldCentric", group="Linear OpMode")
public class fieldCentric extends LinearOpMode {
    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;
    private BNO055IMU imu;
    private MecanumDrive drive;
    GamepadEx gamepadEx = new GamepadEx(gamepad1);


    



    @Override
    public void runOpMode() {
        try{
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            frontLeft = new Motor(hardwareMap, "FL");
            frontRight = new Motor(hardwareMap, "FR");
            backLeft  = new Motor(hardwareMap, "BL");
            backRight = new Motor(hardwareMap, "BR");
            MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight,
                    backLeft, backRight);
            drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        }catch (Error e) {
            telemetry.addData("Error: ", e.getMessage());
            telemetry.update();
        }
        waitForStart();

        double heading = imu.getAngularOrientation().firstAngle;
        double strafe = gamepadEx.getLeftX();
        double forward = -gamepadEx.getLeftY();
        double turn = gamepadEx.getRightX();
        drive.driveFieldCentric(
                forward,
                strafe,
                turn,
                heading
        );
        }

    }

