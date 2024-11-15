package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;



public class HWMap {
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private static IMU imu;
    private static double imuAngle;

    private final Motor frontLeftMotor;
    private final Motor backleftMotor;
    private final Motor backRightMotor;
    private final Motor frontRightMotor;
    private final MecanumDrive mecanumDrive;

    private HardwareMap hwMap;

    public HWMap(HardwareMap hwMap){
        this.hwMap = hwMap;
        colorSensor1 = hwMap.get(RevColorSensorV3.class, "CS1");
        colorSensor2 = hwMap.get(RevColorSensorV3.class, "CS2");
        frontRightMotor = hwMap.get(Motor.class, "RF");
        frontLeftMotor = hwMap.get(Motor.class, "LF");//CH Port 1. The right odo pod accesses this motor's encoder port
        backleftMotor = hwMap.get(Motor.class, "LB"); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        backRightMotor = hwMap.get(Motor.class, "RB");//CH Port 3. The left odo pod accesses this motor's encoder port.
        mecanumDrive =  new MecanumDrive(frontLeftMotor, frontRightMotor, backleftMotor, backRightMotor);
        imu = hwMap.get(IMU.class, "imu");
        initializeIMU();


    }

    public static double readFromIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imuAngle;
    }

    public static void initializeIMU() {
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
        imu.initialize(revParameters);
        imu.resetYaw();
    }

    public IMU getImu() {
        return imu;
    }


    public RevColorSensorV3 getColorSensor1() {

        return colorSensor1;
    }
    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }
    public RevColorSensorV3 getColorSensor2() {

        return colorSensor2;
    }
    public Motor getFrontLeftMotor() {
        return frontLeftMotor;
    }
    public Motor getBackleftMotor() {
        return backleftMotor;
    }
    public Motor getBackRightMotor() {
        return backRightMotor;
        }
    public Motor getFrontRightMotor() {
        return frontRightMotor;
    }
    public static void getImuAngle() {


    }}
