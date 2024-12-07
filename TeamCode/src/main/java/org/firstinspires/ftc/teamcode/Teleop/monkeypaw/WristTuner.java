package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

@TeleOp
@Config
public class WristTuner extends LinearOpMode {
    private HWMap hwMap;
    public static double targetAngle;
    private AxonServoWrapper wristServoWrapper;
    private PIDController pidController;
    public static double P = 0.0075;
    public static double I = 0.00;
    public static double D = 0;
    public static double F = -0.05;
    public static double PID_TOLERANCE = 5;
    private Logger logger;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        logger = new Logger(telemetry);
        wristServoWrapper = new AxonServoWrapper(hwMap.getWristFlexServo(), hwMap.getWristFlexEncoder(), true,true, 0);
        pidController = new PIDController(P,I,D);
        waitForStart();
        while (opModeIsActive()) {
            pidController.setPID(P, I, D);
            pidController.setSetPoint(0); // PIDs the error to 0
            pidController.setTolerance(PID_TOLERANCE); // sets the buffer
            updatePID();
            logger.log("Current Angle", wristServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
            logger.log("Target Angle", targetAngle, Logger.LogLevels.PRODUCTION);
            logger.print();
        }
    }

    public void updatePID() { // This method is used to update position every loop.

        if(targetAngle < 42) {
            targetAngle = 42;
        }
        if(targetAngle > 305) {
            targetAngle = 305;
        }

        wristServoWrapper.readPos();
        double angleDelta = angleDelta(wristServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(wristServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference


        if(!isActualSignEqualToDesiredSign(sign)) {
            sign = -sign;
            angleDelta = negateError(angleDelta);
        }
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("PID Power", power, Logger.LogLevels.PRODUCTION);
        logger.log("Actual Servo Power", wristServoWrapper.get(), Logger.LogLevels.PRODUCTION);
        wristServoWrapper.set((power + (F*(Math.cos(Math.toRadians(wristServoWrapper.getLastReadPos()))))));
    }


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
        if(targetAngle > wristServoWrapper.getLastReadPos()) {
            return 1;
        }
        else if(targetAngle < wristServoWrapper.getLastReadPos()) {
            return -1;
        }
        return 0;
    }

    private double negateError(double currentError) {
        return 360 - Math.abs(currentError);
    }



}
