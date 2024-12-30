package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class HWMap {
    private static PinpointPod pinpointIMU;
    private final MecanumDrive mecanumDrive;

    // Monkey's Limb
    private final MotorEx pivotMotor;
    private final MotorEx armMotorOne;
    private final MotorEx armMotorTwo;
    private final MotorEx armMotorThree;

    //Monkey's Paw

    // Paw Axon Servos
    private final ServoEx elbowServo;
    private final ServoEx wristFlexServo;
    private final ServoEx wristDeviServo;
    private final ServoEx fingerServo;

    // Paw Axon Encoders
    private final AnalogInput elbowEncoder;
    private final AnalogInput wristFlexEncoder;
    private final AnalogInput wristDeviEncoder;


    public HWMap(HardwareMap hardwareMap) {

        MotorEx frontRightMotor = new MotorEx(hardwareMap, "RF", Motor.GoBILDA.RPM_312);
        MotorEx frontLeftMotor = new MotorEx(hardwareMap, "LF", Motor.GoBILDA.RPM_312);//CH Port 1. The right odo pod accesses this motor's encoder port
        MotorEx backleftMotor = new MotorEx(hardwareMap, "LB", Motor.GoBILDA.RPM_312); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        MotorEx backRightMotor = new MotorEx(hardwareMap, "RB", Motor.GoBILDA.RPM_312);//CH Port 3. The left odo pod accesses this motor's encoder port.
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backleftMotor, backRightMotor);
        pinpointIMU = hardwareMap.get(PinpointPod.class, "PP"); //IMU Port 1

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
        elbowServo = hardwareMap.get(ServoEx.class, "ES");
        wristFlexServo = hardwareMap.get(ServoEx.class, "WFS");
        wristDeviServo = hardwareMap.get(ServoEx.class, "WDS");
        fingerServo = hardwareMap.get(ServoEx.class, "FE");

        elbowEncoder = hardwareMap.get(AnalogInput.class, "EE");
        wristFlexEncoder = hardwareMap.get(AnalogInput.class, "WFE");
        wristDeviEncoder = hardwareMap.get(AnalogInput.class, "WDE");

        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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

    public ServoEx getElbowServo() {
        return elbowServo;
    }


    public ServoEx getWristFlexServo() {
        return wristFlexServo;
    }


    public static double readFromIMU() {
        pinpointIMU.update(PinpointPod.readData.ONLY_UPDATE_HEADING);
        Pose2D pos = pinpointIMU.getPosition();
        return pos.getHeading(AngleUnit.DEGREES);
    }

    public static void initializeIMU() {
        pinpointIMU.resetPosAndIMU();
    }

    public ServoEx getWristDeviServo() {
        return wristDeviServo;
    }

    public ServoEx getFingerServo() {
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
}