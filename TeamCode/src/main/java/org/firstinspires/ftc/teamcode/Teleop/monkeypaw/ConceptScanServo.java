/*
 Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


package org.firstinspires.ftc.teamcode.Teleop.monkeypaw;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Core.HWMap;


/*
* This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list

*/


@Config
@TeleOp(name = "Concept: Scan Servo", group = "Concept")
public class ConceptScanServo extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle

    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    CRServo FE;
    CRServo WFE;
    AnalogInput FingerEncoder;
    AnalogInput WristEncoder;
    HWMap hwMap;
    public static double fingerPower = 0.1;
    public static double wristPower;
    private PIDController pidController;
    public static double P = 0.01;
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
            FE = hwMap.getFingerServo();
            WFE = hwMap.getWristFlexServo();
            FingerEncoder = hwMap.getFingerEncoder();
            WristEncoder = hwMap.getWristFlexEncoder();
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            pidController = new PIDController(P,I,D);
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

            pidController.setPID(P,I,D);
            pidController.setSetPoint(0); // PIDs the error to 0
            double currentAngle = WristEncoder.getVoltage()/3.3 * 360; // converts voltage into degrees
            double angleDelta = angleDelta(currentAngle,setTargetAngle); // finds the minimum difference between current angle and target angle
            double sign = angleDeltaSign(currentAngle,setTargetAngle); // sets the direction of servo based on minimum difference
            wristPower = pidController.calculate(angleDelta*sign); // calculates the remaining error(PID)
            pidController.setTolerance(pidTolerance); // sets the buffer


           WFE.set(wristPower);
           FE.set(fingerPower);


            telemetry.addData("at target position",pidController.atSetPoint());
            telemetry.addData("target voltage",pidController.getSetPoint());
            telemetry.addData("left power",fingerPower);
            telemetry.addData("right power", wristPower);
            telemetry.addData("CALA Voltage", "%5.2f", FingerEncoder.getVoltage());
            telemetry.addData("CARA Voltage", "%5.2f", WristEncoder.getVoltage());
            telemetry.addData("CALA Angle", "%5.2f", ((FingerEncoder.getVoltage()/3.3) * 360));
            telemetry.addData("CARA Angle", "%5.2f", ((WristEncoder.getVoltage()/3.3)*360));
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

