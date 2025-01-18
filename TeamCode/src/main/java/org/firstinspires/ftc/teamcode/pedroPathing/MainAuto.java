package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ArmFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;
import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.ShoulderFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.DeviatorFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.ElbowFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.WristFSM;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous
public class MainAuto extends LinearOpMode {
    HWMap hwMap;
    Logger logger;
    private LimbFSM limbFSM;
    private MonkeyPawFSM monkeyPawFSM;

    private Timer pathTimer, actionTimer, opModeTimer;
    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(8, 55, Math.toRadians(0));  // Starting position
    private final Pose preloadScorePose = new Pose(28, 65, Math.toRadians(0)); // Scoring position

    private Path scorePreload, park;
    public void buildPaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(preloadScorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());
    }
    public void updatePath() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        try{
            hwMap = new HWMap(hardwareMap, true);
            logger = new Logger(telemetry);
            ShoulderFSM shoulderFSM = new ShoulderFSM(hwMap, logger, limbFSM);
            ElbowFSM elbowFSM = new ElbowFSM(hwMap, logger, shoulderFSM);
            ArmFSM armFSM = new ArmFSM(hwMap, logger, shoulderFSM, elbowFSM);
            DeviatorFSM deviatorFSM = new DeviatorFSM(hwMap, logger);
            WristFSM wristFSM = new WristFSM(hwMap, logger, elbowFSM);
            limbFSM = new LimbFSM(hwMap, shoulderFSM, armFSM, monkeyPawFSM, logger);
            monkeyPawFSM = new MonkeyPawFSM(hwMap, logger, limbFSM, elbowFSM, deviatorFSM, wristFSM, armFSM);

            limbFSM.setMonkeyPawFSM(monkeyPawFSM);
            shoulderFSM.setLimbFSM(limbFSM);
            elbowFSM.setArmFSM(armFSM);

            pathTimer = new Timer();
            actionTimer = new Timer();
            opModeTimer = new Timer();
            opModeTimer.resetTimer();
            Constants.setConstants(FConstants.class, LConstants.class);
            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);
            buildPaths();
            pathState = 0;
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        while (opModeInInit()) {
            monkeyPawFSM.updateState(false,false,false,false,false, false,false,false,false,false);
            monkeyPawFSM.updatePID();
        }
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            updatePath();
            logger.log("path state",pathState, Logger.LogLevels.PRODUCTION);
            logger.print();
        }

    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
    }
}
