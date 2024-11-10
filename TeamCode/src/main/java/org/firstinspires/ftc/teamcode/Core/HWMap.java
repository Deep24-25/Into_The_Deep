package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HWMap {
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



    public HWMap(HardwareMap hardwareMap){
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

    public CRServo getWristDeviServo() {
        return wristDeviServo;
    }
    public CRServo getFingerServo() {
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

    public AnalogInput getFingerEncoder() {
        return fingerEncoder;
    }
}
