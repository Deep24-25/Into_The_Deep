package org.firstinspires.ftc.teamcode.Tuner;

import android.graphics.Color;
import android.os.Environment;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.opencsv.CSVWriter;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;


public class ColorSensorCSVTuner {
    //Core Objects
    private final GamepadEx gamepadEx;
    private final Telemetry telemetry;

    //Objects
    private final CSVWriter csvWriter;
    private final RevColorSensorV3 csLeft;
    private final RevColorSensorV3 csRight;

    //Booleans
    private boolean dpadLeftWasJustPressed;
    private boolean dpadLeftIsDown;
    private boolean addData;
    private boolean dataStopped;

    //Colors
    private String color;
    private final ArrayList<String[]> dataArray = new ArrayList<String[]>();

    public ColorSensorCSVTuner(Telemetry telemetry, RevColorSensorV3 csLeft, RevColorSensorV3 csRight, GamepadEx gamepadEx) throws IOException {
        File file = new File(String.format("%s/FIRST/data.csv", Environment.getExternalStorageDirectory().getAbsolutePath()));
        FileWriter fileWriter = new FileWriter(file);
        csvWriter = new CSVWriter(fileWriter);

        this.csLeft = csLeft;
        this.csRight = csRight;
        this.gamepadEx = gamepadEx;
        this.telemetry = telemetry;

        dpadLeftWasJustPressed = false;
        dpadLeftIsDown = false;
        addData = true;
        dataStopped = false;

        color = "Purple";
        dataArray.add(new String[]{"Color", "Red_left", "Green_left", "Blue_left", "Alpha_left", "Red_right", "Green_right", "Blue_right", "Alpha_right"});
    }

    public void dataCollection() {
        String redLeft = csLeft.red() + "";
        String greenLeft = csLeft.green() + "";
        String blueLeft = csLeft.blue() + "";
        String alphaLeft = csLeft.alpha() + "";
        String redRight = csRight.red() + "";
        String greenRight = csRight.green() + "";
        String blueRight = csRight.blue() + "";
        String alphaRight = csRight.alpha() + "";

        dpadLeftWasJustPressed = gamepadEx.isDown(GamepadKeys.Button.DPAD_LEFT) & !dpadLeftIsDown;
        dpadLeftIsDown = gamepadEx.isDown(GamepadKeys.Button.DPAD_LEFT);

        if (gamepadEx.isDown(GamepadKeys.Button.Y))
            color = "Yellow";
        else if (gamepadEx.isDown(GamepadKeys.Button.X))
            color = "Blue";
        else if (gamepadEx.isDown(GamepadKeys.Button.B))
            color = "Red";


        if (dpadLeftWasJustPressed)
            addData = !addData;

        if (addData)
            dataArray.add(new String[]{color, redLeft, greenLeft, blueLeft, alphaLeft, redRight, greenRight, blueRight, alphaRight});
    }

    public void exportData() {
        dataStopped = true;
        try {
            csvWriter.writeAll(dataArray);
            csvWriter.close();
        } catch (Exception ignored) {

        }
    }

    public void log() {
        if (!addData)
            telemetry.addData("-", "DATA IS NOT BEING ADDED");
        if (dataStopped)
            telemetry.addData("-", "DATA HAS BEEN EXPORTED");

        telemetry.addData("CURRENT COLOR: ", color);
        telemetry.addData("","");
        telemetry.addData("-", "Y is Yellow");
        telemetry.addData("-", "X is Blue");
        telemetry.addData("-", "B is Red");
        telemetry.addData("-", "DpadLeft to toggle readings");
        telemetry.addData("-", "DpadDown to save and close readings");
        telemetry.update();
    }

}
