package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class HWMap {
  //  private static PinpointPod pinpointIMU;
  private static IMU imu;
    public static double imuAngle;
    private final MecanumDrive mecanumDrive;
    private final VoltageSensor voltageSensor;

    // Drivetrain
    private final MotorEx frontRightMotor;
    private final MotorEx frontLeftMotor;
    private final MotorEx backleftMotor;
    private final MotorEx backRightMotor;

    // Monkey's Limb
    private final MotorEx pivotMotor;
    private final MotorEx armMotorOne;
    private final MotorEx armMotorTwo;
    private final MotorEx armMotorThree;

    //Monkey's Paw

    // Paw Axon Servos
    private final Servo elbowServo;
    private final Servo wristFlexServo;
    private final Servo wristDeviServo;
    private final Servo fingerServo;

    // Paw Axon Encoders
    private final AnalogInput elbowEncoder;
    private final AnalogInput wristFlexEncoder;
    private final AnalogInput wristDeviEncoder;

    //private static Pose2D IMUpos;

    List<LynxModule> hubs;

    public HWMap(HardwareMap hardwareMap, boolean isAuto) {
        hubs = hardwareMap.getAll(LynxModule.class);

        frontRightMotor = new MotorEx(hardwareMap, "RF", Motor.GoBILDA.RPM_312);
        frontLeftMotor = new MotorEx(hardwareMap, "LF", Motor.GoBILDA.RPM_312);//CH Port 1. The right odo pod accesses this motor's encoder port
        backleftMotor = new MotorEx(hardwareMap, "LB", Motor.GoBILDA.RPM_312); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        backRightMotor = new MotorEx(hardwareMap, "RB", Motor.GoBILDA.RPM_312);//CH Port 3. The left odo pod accesses this motor's encoder port.
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backleftMotor, backRightMotor);
        //pinpointIMU = hardwareMap.get(PinpointPod.class, "PP"); //IMU Port 1
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        mecanumDrive.setRightSideInverted(false);
        backleftMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        //Monkey's Limb
        pivotMotor = new MotorEx(hardwareMap, "PM", Motor.GoBILDA.RPM_60);
        armMotorOne = new MotorEx(hardwareMap, "AM1", Motor.GoBILDA.BARE);
        armMotorTwo = new MotorEx(hardwareMap, "AM2", Motor.GoBILDA.BARE);
        armMotorThree = new MotorEx(hardwareMap, "AM3", Motor.GoBILDA.BARE);

        armMotorOne.setInverted(true);
        armMotorTwo.setInverted(true);
        armMotorThree.setInverted(true);

        //Monkey's Paw
        elbowServo = hardwareMap.get(Servo.class, "ES");
        wristFlexServo = hardwareMap.get(Servo.class, "WFS");
        wristDeviServo = hardwareMap.get(Servo.class, "WDS");
        fingerServo = hardwareMap.get(Servo.class, "FS");

        elbowEncoder = hardwareMap.get(AnalogInput.class, "EE");
        wristFlexEncoder = hardwareMap.get(AnalogInput.class, "WFE");
        wristDeviEncoder = hardwareMap.get(AnalogInput.class, "WDE");

        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        brakingOff();
        if(isAuto) {
            imu = hardwareMap.get(IMU.class, "imu");
            initializeIMU();
            armMotorOne.resetEncoder();
            pivotMotor.resetEncoder();
        }
        else {
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
    }

    // Monkey's Limb Getters
    public MotorEx getPivotMotor() {
        return pivotMotor;
    }

    public MotorEx getArmMotorOne() {
        return armMotorOne;
    }

    public MotorEx getArmMotorTwo() {
        return armMotorTwo;
    }

    public MotorEx getArmMotorThree() {
        return armMotorThree;
    }

    // Monkey's Paw Axon Servo Getters

    public Servo getElbowServo() {
        return elbowServo;
    }


    public Servo getWristFlexServo() {
        return wristFlexServo;
    }


    public static double readFromIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imuAngle;
        //pinpointIMU.update(PinpointPod.readData.ONLY_UPDATE_HEADING);
        //IMUpos = pinpointIMU.getPosition();
        //return IMUpos.getHeading(AngleUnit.DEGREES);

    }

    public static double getIMUangle() {
        return imuAngle;
        //return IMUpos.getHeading(AngleUnit.DEGREES);
    }

    public static void initializeIMU() {
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
        imu.initialize(revParameters);
        imu.resetYaw();
       // pinpointIMU.resetPosAndIMU();
    }

    public Servo getWristDeviServo() {
        return wristDeviServo;
    }

    public Servo getFingerServo() {
        return fingerServo;
    }

    // Monkey's Paw Axon Encoder Getters

    public AnalogInput getElbowEncoder() {
        return elbowEncoder;
    }

    public AnalogInput getWristFlexEncoder() {
        return wristFlexEncoder;
    }

    public AnalogInput getWristDeviEncoder() {
        return wristDeviEncoder;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public VoltageSensor getVoltageSensor() {
        return voltageSensor;
    }

    public void brakingOn() {

        armMotorOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotorTwo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotorThree.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void brakingOff() {
        armMotorOne.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotorTwo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotorThree.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void clearCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }



}