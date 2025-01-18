package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ShoulderFSM;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.ShoulderWrapper;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;

@TeleOp
@Config
public class ElbowTunerClass extends LinearOpMode {
    private HWMap hwMap;
    public static double targetAngle;
    private AxonServoWrapper elbowServoWrapper;
    private PIDController pidController;
    public static double P = 0.007;
    public static double I = 0;
    public static double D = 0;
    public static double F = -0.12;
    public static double PID_TOLERANCE = 5;
    private Logger logger;

    private ShoulderWrapper shoulderWrapper;
    private ShoulderFSM shoulderFSM;
    public static double pivotShoulder = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap, false);
        logger = new Logger(telemetry);
        elbowServoWrapper = new AxonServoWrapper(hwMap.getElbowServo(), hwMap.getElbowEncoder(), false,false, ElbowFSM.ENCODER_OFFSET);
        shoulderWrapper = new ShoulderWrapper(hwMap);
        pidController = new PIDController(P,I,D);
        shoulderFSM = new ShoulderFSM(hwMap, logger);
        waitForStart();
        while (opModeIsActive()) {
            pidController.setPID(P, I, D);
            pidController.setSetPoint(0); // PIDs the error to 0
            pidController.setTolerance(PID_TOLERANCE); // sets the buffer

            if (pivotShoulder == 0) {
                shoulderFSM.moveToIntakeAngle();
            } else if (pivotShoulder == 1) {
                shoulderFSM.setBasketTargetAngle();
            }
            //updatePID();
            shoulderFSM.updatePID();
            logger.log("Current Angle", elbowServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
            logger.log("Target Angle", targetAngle, Logger.LogLevels.PRODUCTION);
            logger.print();
        }
    }
/*
    public void updatePID() { // This method is used to update position every loop.
*//*
        if(targetAngle < 22) {
            targetAngle = 22;
        }
        if(targetAngle > 296) {
            targetAngle = 296;
        }*//*

        elbowServoWrapper.readPos();
        double angleDelta = angleDelta(elbowServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
       double sign = angleDeltaSign(elbowServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference


        if(!isActualSignEqualToDesiredSign(sign)) {
            sign = -sign;
            angleDelta = negateError(angleDelta);
        }

        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("PID Power", power, Logger.LogLevels.PRODUCTION);
        logger.log("Elbow Power",(power + (F*(Math.toRadians(Math.cos(elbowServoWrapper.getLastReadPos()))))), Logger.LogLevels.PRODUCTION);
        logger.log("Cosine", Math.cos(Math.toRadians(elbowServoWrapper.getLastReadPos() - shoulderWrapper.readAngle())), Logger.LogLevels.PRODUCTION);
        logger.log("Actual Servo Power", elbowServoWrapper.get(), Logger.LogLevels.PRODUCTION);
        elbowServoWrapper.set((power + ((F*(Math.cos(Math.toRadians(elbowServoWrapper.getLastReadPos() - shoulderWrapper.readAngle())))))*26.0/24.0));

    }*/


    // Finds the smallest distance between 2 angles, input and output in degrees
    private double angleDelta(double angle1, double angle2) {
        return Math.min(normalizeDegrees(angle1 - angle2), 360 - normalizeDegrees(angle1 - angle2));
    }


    // Finds the direction of the smallest distance between 2 angles
    private double angleDeltaSign(double position, double target) {
        return -(Math.signum(normalizeDegrees(target - position) - (360 - normalizeDegrees(target - position))));
    }

    // Takes input angle in degrees, returns that angle in the range of 0-360
    //Prevents the servos from looping around
    private static double normalizeDegrees(double angle) {
        return (angle + 360) % 360;
    }

    private boolean isActualSignEqualToDesiredSign(double actualSign) {
        return actualSign == desiredSign();
    }

    private double desiredSign() {
        if(targetAngle > elbowServoWrapper.getLastReadPos()) {
            return 1;
        }
        else if(targetAngle < elbowServoWrapper.getLastReadPos()) {
            return -1;
        }
        return 0;
    }

    private double negateError(double currentError) {
        return 360 - Math.abs(currentError);
    }





}
