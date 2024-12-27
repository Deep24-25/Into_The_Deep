


package org.firstinspires.ftc.teamcode.Tuner_Classes.Paw_Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;


@Config
@TeleOp(name = "Concept: Scan Servo", group = "Concept")
public class ConceptScanServo extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle

    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    Servo WDS;
    Servo WFS;
    Servo ES;
    AnalogInput DeviatorEncoder;
    AnalogInput WristEncoder;
    AnalogInput ElbowEncoder;
    HWMap hwMap;
    public static double deviatorPower = 0.0;
    public static double wristPower;
    private PIDController pidController;
    public static double P = 0.00;
    public static double I = 0;
    public static double D = 0;
    public static double setTargetAngle;
    public static double pidTolerance;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        try {
            hwMap = new HWMap(hardwareMap);
            WDS = hwMap.getWristDeviServo();
            WFS = hwMap.getWristFlexServo();
            ES = hwMap.getElbowServo();
            DeviatorEncoder = hwMap.getWristDeviEncoder();
            WristEncoder = hwMap.getWristFlexEncoder();
            ElbowEncoder = hwMap.getElbowEncoder();
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            pidController = new PIDController(P, I, D);
        } catch (Exception exception) {
            telemetry.addData("Message", exception.getMessage());
        }


        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            //hwMap.clearCache();

            pidController.setPID(P, I, D);
            pidController.setSetPoint(0); // PIDs the error to 0
            double currentAngle = WristEncoder.getVoltage() / 3.3 * 360; // converts voltage into degrees
            double angleDelta = angleDelta(currentAngle, setTargetAngle); // finds the minimum difference between current angle and target angle
            double sign = angleDeltaSign(currentAngle, setTargetAngle); // sets the direction of servo based on minimum difference
            wristPower = pidController.calculate(angleDelta * sign); // calculates the remaining error(PID)
            pidController.setTolerance(pidTolerance); // sets the buffer


            WFS.setPosition(0.5);
            WDS.setPosition(0.5);

            telemetry.addData("at target position", pidController.atSetPoint());
            telemetry.addData("target voltage", pidController.getSetPoint());
            telemetry.addData("left power", deviatorPower);
            telemetry.addData("right power", wristPower);
            telemetry.addData("Deviator Voltage", "%5.2f", DeviatorEncoder.getVoltage());
            telemetry.addData("Wrist Voltage", "%5.2f", WristEncoder.getVoltage());
            telemetry.addData("Deviator Angle", "%5.2f", ((DeviatorEncoder.getVoltage() / 3.3) * 360));
            telemetry.addData("Wrist Angle", "%5.2f", ((WristEncoder.getVoltage() / 3.3) * 360));
            telemetry.addData("Elbow Angle", "%5.2f", ((ElbowEncoder.getVoltage() / 3.3) * 360));
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

        }
    }

    // Finds the smallest distance between 2 angles, input and output in degrees
    private double angleDelta(double current, double target) {
        return Math.min(normalizeRadians(current - target), 360 - normalizeRadians(current - target));
    }

    // Finds the direction of the smallest distance between 2 angles
    private double angleDeltaSign(double current, double target) {
        return -(Math.signum(normalizeRadians(target - current) - (360 - normalizeRadians(target - current))));
    }

    // Converts angle from degrees to radians
    private double toRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    // Takes input angle in degrees, returns that angle in the range of 0-360
    //Prevents the servos from looping around
    private static double normalizeRadians(double angle) {
        return (angle + 360) % 360;
    }
}

