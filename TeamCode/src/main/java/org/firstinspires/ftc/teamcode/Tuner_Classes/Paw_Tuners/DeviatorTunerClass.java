package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Wrappers.AxonServoWrapper;

@Config
@TeleOp
public class DeviatorTunerClass extends LinearOpMode {
    private HWMap hwMap;
    public static double targetAngle;
    private AxonServoWrapper deviatorServoWrapper;
    private PIDController pidController;
    public static double P = 0.003;
    public static double I = 0.00;
    public static double D = 0;
    public static double PID_TOLERANCE = 5;
    private Logger logger;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap, false);
        logger = new Logger(telemetry);
        deviatorServoWrapper = new AxonServoWrapper(hwMap.getWristDeviServo(), hwMap.getWristDeviEncoder(), false,false, 0,1);
        pidController = new PIDController(P,I,D);
        waitForStart();
        while (opModeIsActive()) {
            pidController.setPID(P, I, D);
            pidController.setSetPoint(0); // PIDs the error to 0
            pidController.setTolerance(PID_TOLERANCE); // sets the buffer
            updatePID();
            logger.log("Current Angle", deviatorServoWrapper.getLastReadPos(), Logger.LogLevels.PRODUCTION);
            logger.log("Target Angle", targetAngle, Logger.LogLevels.PRODUCTION);
            logger.print();
        }
    }

    public void updatePID() { // This method is used to update position every loop.
        deviatorServoWrapper.readPos();
        double angleDelta = angleDelta(deviatorServoWrapper.getLastReadPos(), targetAngle); // finds the minimum difference between current angle and target angle
        double sign = angleDeltaSign(deviatorServoWrapper.getLastReadPos(), targetAngle); // sets the direction of servo based on minimum difference
        double power = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
        logger.log("Deviator Power",power, Logger.LogLevels.DEBUG);
        deviatorServoWrapper.set(power);
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



}
