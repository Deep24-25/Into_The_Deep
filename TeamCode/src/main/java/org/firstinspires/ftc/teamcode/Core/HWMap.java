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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class HWMap {

    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private static IMU imu;
    private static PinpointPod pinpointIMU;
    private static double imuAngle;

    private final Motor frontLeftMotor;
    private final Motor backleftMotor;
    private final Motor backRightMotor;
    private final Motor frontRightMotor;
    private final MecanumDrive mecanumDrive;


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

        //colorSensor1 = this.hardwareMap.get(RevColorSensorV3.class, "CS1");
        //colorSensor2 = this.hardwareMap.get(RevColorSensorV3.class, "CS2");
        frontRightMotor = new Motor(hardwareMap,"RF", Motor.GoBILDA.RPM_312);//CH Port 1
        frontLeftMotor = new Motor(hardwareMap,"LF", Motor.GoBILDA.RPM_312);//CH Port 3. The right odo pod accesses this motor's encoder port
        backleftMotor = new Motor(hardwareMap,"LB", Motor.GoBILDA.RPM_312); //CH Port 0. The perpendicular odo pod accesses this motor's encoder port
        backRightMotor = new Motor(hardwareMap,"RB", Motor.GoBILDA.RPM_312);//CH Port 2. The left odo pod accesses this motor's encoder port.

        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backleftMotor, backRightMotor);
        mecanumDrive.setRightSideInverted(false);
        backleftMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        imu = hardwareMap.get(IMU.class, "imu");
        pinpointIMU = hardwareMap.get(PinpointPod.class, "PP"); //IMU Port 1


        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        //Monkey's Limb
        //pivotMotor = new Motor(hardwareMap,"PM", Motor.GoBILDA.RPM_312);
        //armMotorOne = new Motor(hardwareMap,"AM1", Motor.GoBILDA.RPM_312);
        //armMotorTwo = new Motor(hardwareMap,"AM2", Motor.GoBILDA.RPM_312);
        //armMotorThree = new Motor(hardwareMap,"AM3", Motor.GoBILDA.RPM_312);

        //Monkey's Paw
        //elbowServo = new CRServo(hardwareMap, "ES");
        //wristFlexServo = new CRServo(hardwareMap, "WFS");
        //wristDeviServo = new CRServo(hardwareMap, "WDS");
        //fingerServo = new CRServo(hardwareMap, "FS");

        //elbowEncoder = hardwareMap.get(AnalogInput.class, "EE");
        //wristFlexEncoder = hardwareMap.get(AnalogInput.class, "WFE");
        //wristDeviEncoder = hardwareMap.get(AnalogInput.class, "WDE");
        //fingerEncoder = hardwareMap.get(AnalogInput.class, "FE");
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
//
//    public static double readFromIMU() {
//        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        return imuAngle;
//    }
//
//    public static void initializeIMU() {
//        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
//        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
//        imu.initialize(revParameters);
//        imu.resetYaw();
//    }

    public static double readFromIMU(){
        pinpointIMU.update(PinpointPod.readData.ONLY_UPDATE_HEADING);
        Pose2D pos = pinpointIMU.getPosition();
        imuAngle = pos.getHeading(AngleUnit.DEGREES);
        return imuAngle;
    }
    public static void initializeIMU(){
        pinpointIMU.resetPosAndIMU();
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

    public static double getImuAngle() {
        return imuAngle;
    }
}
