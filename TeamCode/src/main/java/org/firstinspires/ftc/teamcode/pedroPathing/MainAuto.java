package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.pathgen.PathChain;
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

    private final Pose startPose = new Pose(7, 55, Math.toRadians(180));  // Starting position
    private final Pose preloadScorePose = new Pose(28.5, 65, Math.toRadians(180)); // Scoring position
    private final Pose parkPose = new Pose(8,15,  Math.toRadians(180));

    private PathChain scorePreload,park;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(parkPose)))
                .setConstantHeadingInterpolation(preloadScorePose.getHeading())
                .build();
    }
    public void updatePath() {
        monkeyPawFSM.updateState(false,false,false,false,false,false,false,false,false,false,true);
        limbFSM.updateState(false,false,false,false,false,false,false,false,false,false,0,false,true);
        monkeyPawFSM.updatePID();
        limbFSM.updatePID();
        limbFSM.setMode(LimbFSM.Mode.SPECIMEN_MODE);
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    limbFSM.setStates(LimbFSM.States.INTAKING_SPECIMEN);
                    monkeyPawFSM.setState(MonkeyPawFSM.States.INTAKING_SPECIMEN);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    if(monkeyPawFSM.INTAKED_SPECIMEN() && limbFSM.INTAKED_SPECIMEN()) {
                        limbFSM.setStates(LimbFSM.States.EXTENDING_SPECIMEN);
                        monkeyPawFSM.setState(MonkeyPawFSM.States.GETTING_READY_TO_DEPOSIT_SPECIMEN);
                        setPathState(3);
                    }
                }
            case 3:
                if(!follower.isBusy()) {
                    if (monkeyPawFSM.READY_TO_DEPOSIT_SPECIMEN() && limbFSM.EXTENDED_SPECIMEN()) {
                        limbFSM.setStates(LimbFSM.States.DEPOSITING_SPECIMEN);
                        setPathState(4);
                    }
                }
            case 4:
                if(!follower.isBusy()) {
                    if (limbFSM.DEPOSITED_SPECIMEN()) {
                        monkeyPawFSM.setState(MonkeyPawFSM.States.DEPOSITING_SPECIMEN);
                        setPathState(5);
                    }
                }
            case 5:
                if(!follower.isBusy()) {
                    if (monkeyPawFSM.DEPOSITED_SPECIMEN()) {
                        limbFSM.setStates(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
                        monkeyPawFSM.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
                        setPathState(6);
                    }
                }
            case 6:
                if(!follower.isBusy()) {
                    if(monkeyPawFSM.PREPARED_TO_INTAKE_SPECIMEN() && limbFSM.PREPARED_TO_INTAKE_SPECIMEN()) {
                        follower.followPath(park);
                        setPathState(7);
                    }
                }
                break;
            case 7:
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
            monkeyPawFSM.setState(MonkeyPawFSM.States.GETTING_READY_TO_DEPOSIT_SPECIMEN);
            monkeyPawFSM.updateState(false,false,false,false,false, false,false,false,false,false, true);
            monkeyPawFSM.updatePID();
            logger.log("Monkey Paw State", monkeyPawFSM.getState(), Logger.LogLevels.PRODUCTION);
            logger.print();
        }
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            updatePath();
            logger.log("x", follower.getPose().getX(), Logger.LogLevels.PRODUCTION);
            logger.log("y", follower.getPose().getX(), Logger.LogLevels.PRODUCTION);
            logger.log("heading", follower.getPose().getHeading(), Logger.LogLevels.PRODUCTION);
            logger.log("path state",pathState, Logger.LogLevels.PRODUCTION);
            logger.log("limb state", limbFSM.getStates(), Logger.LogLevels.PRODUCTION);
            logger.log("monkey paw state", monkeyPawFSM.getState(), Logger.LogLevels.PRODUCTION);
            monkeyPawFSM.log();
            limbFSM.log();
            logger.print();
        }

    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
    }
}
