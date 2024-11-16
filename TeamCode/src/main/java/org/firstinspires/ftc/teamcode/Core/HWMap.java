package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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

    private HardwareMap hardwareMap;

    // Monkey's Limb
    private Motor pivotMotor;
    private Motor armMotorOne;
    private Motor armMotorTwo;
    private Motor armMotorThree;

    //Monkey's Paw

    // Paw Axon Servos
    private CRServo elbowServo;
    private CRServo wristFlexServo;
    private CRServo wristDeviServo;
    private CRServo fingerServo;

    // Paw Axon Encoders
    private AnalogInput elbowEncoder;
    private AnalogInput wristFlexEncoder;
    private AnalogInput wristDeviEncoder;
    private AnalogInput fingerEncoder;


    public HWMap(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        colorSensor1 = this.hardwareMap.get(RevColorSensorV3.class, "CS1");
        colorSensor2 = this.hardwareMap.get(RevColorSensorV3.class, "CS2");
        frontRightMotor = this.hardwareMap.get(Motor.class, "RF");
        frontLeftMotor = this.hardwareMap.get(Motor.class, "LF");//CH Port 1. The right odo pod accesses this motor's encoder port
        backleftMotor = this.hardwareMap.get(Motor.class, "LB"); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        backRightMotor = this.hardwareMap.get(Motor.class, "RB");//CH Port 3. The left odo pod accesses this motor's encoder port.
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backleftMotor, backRightMotor);
        imu = this.hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        //Monkey's Limb
        pivotMotor = hardwareMap.get(Motor.class, "PM");
        armMotorOne = hardwareMap.get(Motor.class, "AM1");
        armMotorTwo = hardwareMap.get(Motor.class, "AM2");
        armMotorThree = hardwareMap.get(Motor.class, "AM3");

        //Monkey's Paw
        elbowServo = hardwareMap.get(CRServo.class, "ES");
        wristFlexServo = hardwareMap.get(CRServo.class, "WFS");
        wristDeviServo = hardwareMap.get(CRServo.class, "WDS");
        fingerServo = hardwareMap.get(CRServo.class, "FS");

        elbowEncoder = hardwareMap.get(AnalogInput.class, "EE");
        wristFlexEncoder = hardwareMap.get(AnalogInput.class, "WFE");
        wristDeviEncoder = hardwareMap.get(AnalogInput.class, "WDE");
        fingerEncoder = hardwareMap.get(AnalogInput.class, "FE");
    }

    // Monkey's Limb Getters
    public Motor getPivotMotor() {
        return pivotMotor;
    }

    public Motor getArmMotorOne() {
        return armMotorOne;
    }

    public Motor getArmMotorTwo() {
        return armMotorTwo;
    }

    public Motor getArmMotorThree() {
        return armMotorThree;
    }

    // Monkey's Paw Axon Servo Getters

    public CRServo getElbowServo() {
        return elbowServo;
    }


    public CRServo getWristFlexServo() {
        return wristFlexServo;
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

    public CRServo getWristDeviServo() {
        return wristDeviServo;
    }

    public CRServo getFingerServo() {
        return fingerServo;
    }

    public IMU getImu() {
        return imu;
    }


    // Monkey's Paw Axon Encoder Getters

    public AnalogInput getElbowEncoder() {
        return elbowEncoder;
    }

    public RevColorSensorV3 getColorSensor1() {
        return colorSensor1;
    }

    public AnalogInput getWristFlexEncoder() {
        return wristFlexEncoder;
    }

    public AnalogInput getWristDeviEncoder() {
        return wristDeviEncoder;
    }

    public AnalogInput getFingerEncoder() {
        return fingerEncoder;
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

}
