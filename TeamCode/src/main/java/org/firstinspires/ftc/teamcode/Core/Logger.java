package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger {

    public static enum LogLevels {
        DEBUG,
        PRODUCTION,
        DRIVER_DATA
    }

    private Telemetry telemetry;
    private LogLevels state;

    public Logger(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        state = LogLevels.PRODUCTION;
    }

    public void log(String caption, Object data, LogLevels logLevel) {
        switch (state) {
            case DEBUG:
                if(!(logLevel == LogLevels.DRIVER_DATA)) {
                    telemetry.addData(caption, data);
                }
                break;
            case PRODUCTION:
                if (logLevel == LogLevels.PRODUCTION) {
                    telemetry.addData(caption, data);
                }
                break;
            case DRIVER_DATA:
                if (logLevel == LogLevels.DRIVER_DATA) {
                    telemetry.addData(caption, data);
                }
                break;
        }
    }

    public void updateLoggingLevel(boolean touchpad) {
        if(touchpad) {
            if (state == LogLevels.PRODUCTION) {
                state = LogLevels.DEBUG;
            } else if (state == LogLevels.DEBUG) {
                state = LogLevels.DRIVER_DATA;
            } else {
                state = LogLevels.PRODUCTION;
            }
        }
    }
    public void print() {
        telemetry.update();
    }

    public boolean DEBUG() {
        return state == LogLevels.DEBUG;
    }

    public boolean PRODUCTION() {
        return state == LogLevels.DEBUG;
    }

    public boolean DRIVER_DATA() {
        return state == LogLevels.DEBUG;
    }
}

