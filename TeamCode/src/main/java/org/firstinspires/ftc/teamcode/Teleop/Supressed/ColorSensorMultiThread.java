package org.firstinspires.ftc.teamcode.Teleop.Supressed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

import java.util.concurrent.TimeUnit;

@TeleOp
@Disabled
public class ColorSensorMultiThread extends LinearOpMode {
    private HWMap hwMap;
    private RevColorSensorV3 csLeft;
    private RevColorSensorV3 csRight;

    private LeftSensorThread leftSensorThread;
    private RightSensorThread rightSensorThread;
    private GamepadEx gamepadEx;
    private Timing.Timer loopTimer;
    private Thread thread;
    private static final int LEFT_DISTANCE_THRESHOLD = 40;
    private static final int RIGHT_DISTANCE_THRESHOLD = 60;

    private double redLeft, blueLeft, greenLeft, distanceLeft;
    private double redRight, blueRight, greenRight, distanceRight;

    public static double colorCounterLeft = 0;
    public static double colorCounterRight = 0;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(hardwareMap);
            gamepadEx = new GamepadEx(gamepad1);
//            csLeft = hwMap.getColorSensor1();
//            csRight = hwMap.getColorSensor2();
            leftSensorThread = new LeftSensorThread();
            rightSensorThread = new RightSensorThread();
            thread = Thread.currentThread();
            loopTimer = new Timing.Timer(10000000, TimeUnit.MILLISECONDS);
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        } catch (Exception e) {
            throw new RuntimeException(e.getMessage());
        }

        waitForStart();

        while (opModeIsActive()) {
            loopTimer.start();

            // Create and start sensor threads

            leftSensorThread.setPriority(10);
            rightSensorThread.setPriority(10);
            thread.setPriority(10);


            rightSensorThread.start();
            leftSensorThread.start();

            // Process the sensor data after both threads are done
            if (distanceLeft < LEFT_DISTANCE_THRESHOLD) {
                if (greenLeft > redLeft && redLeft > blueLeft) {//GRB
                    telemetry.addData("-", "YELLOW by left");
                    colorCounterLeft = 1;
                } else if (redLeft > greenLeft && greenLeft > blueLeft) {//RGB
                    telemetry.addData("-", "RED by left");
                    colorCounterLeft = 2;
                } else if (blueLeft > greenLeft && greenLeft > redLeft) { //BGR
                    telemetry.addData("-", "BLUE by left");
                    colorCounterLeft = 3;
                }
            }else{
                colorCounterLeft = 0;
            }

            if (distanceRight < RIGHT_DISTANCE_THRESHOLD) {
                if (greenRight > redRight && redLeft > blueRight) {//GRB
                    telemetry.addData("-", "YELLOW by right");
                    colorCounterRight = 1;
                } else if (redRight > greenRight && greenRight > blueRight) { //RGB
                    telemetry.addData("-", "RED by right");
                    colorCounterRight = 2;
                } else if (blueRight > greenRight && greenRight > redRight) { //BGR
                    telemetry.addData("-", "BLUE by right");
                    colorCounterRight = 3;
                }

            }else{
                colorCounterRight = 0;
            }

            telemetry.addData("-", "Left Distance: " + distanceLeft);
            telemetry.addData("-", "Right Distance: " + distanceRight);
            telemetry.addData("-", "Loop Time: " + loopTimer.elapsedTime());
            telemetry.addData("colorRight", colorCounterRight);
            telemetry.addData("colorLeft", colorCounterLeft);
            telemetry.update();

            try {
                leftSensorThread.join();
                rightSensorThread.join();
            } catch (Exception e) {
                telemetry.addData("-", e.getMessage());
                telemetry.update();
            }
        }
    }

    // Thread to read data from the left sensor
    class LeftSensorThread extends Thread {
        @Override
        public void run() {
            redLeft = csLeft.red();
            blueLeft = csLeft.blue();
            greenLeft = csLeft.green();
            distanceLeft = csLeft.getDistance(DistanceUnit.MM);
        }
    }

    // Thread to read data from the right sensor
    class RightSensorThread extends Thread {
        @Override
        public void run() {
            redRight = csRight.red();
            blueRight = csRight.blue();
            greenRight = csRight.green();
            distanceRight = csRight.getDistance(DistanceUnit.MM);
        }
    }
}
