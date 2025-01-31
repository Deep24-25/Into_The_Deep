package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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
    private int depositSpecState;

    private final Pose startPose = new Pose(7, 55, Math.toRadians(180));  // Starting position
    private final Pose preloadScorePose = new Pose(42, 69, Math.toRadians(180)); // Scoring position

    private final Pose pushSampleIntermediatary = new Pose(34,36,Math.toRadians(180));
    private final Pose pushSampleIntermediatary2 = new Pose(60,36,Math.toRadians(180));
    private final Pose pushSampleIntermediatary3 = new Pose(55,24,Math.toRadians(180));
    private final Pose samplePushed = new Pose(25,24,Math.toRadians(180));
    private final Pose sample2PushIntermediatary = new Pose(60,12,Math.toRadians(180));
    private final Pose sample2Pushed = new Pose(25,12,Math.toRadians(180));

    private final Pose sampleIntakePos = new Pose(17,31,Math.toRadians(180));
    private final Pose parkPose = new Pose(8,15,  Math.toRadians(180));
    private final Pose firstSpecScorePose = new Pose(42.00, 67, Math.toRadians(180)); // Scoring position


    private PathChain scorePreload,pushSamples, scoreFirstSpec, park;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadScorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        pushSamples = follower.pathBuilder()
               // .addPath(new BezierCurve(new Point(preloadScorePose), new Point(pushSampleIntermediatary)))
                .addPath(new BezierCurve(new Point(42.00, 69.00, Point.CARTESIAN), new Point(24.00,72.00, Point.CARTESIAN), new Point(34.00, 36.00, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(preloadScorePose.getHeading())
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(34.000, 36.000, Point.CARTESIAN),
                                new Point(60.000, 40.000, Point.CARTESIAN),
                                new Point(55.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(pushSampleIntermediatary.getHeading())
                .addPath(new BezierCurve(new Point(pushSampleIntermediatary3), new Point(samplePushed)))
                .setConstantHeadingInterpolation(pushSampleIntermediatary3.getHeading())
                //.addPath(new BezierCurve(new Point(samplePushed), new Point(pushSampleIntermediatary3)))
               // .setConstantHeadingInterpolation(samplePushed.getHeading())
                //.addPath(new BezierCurve(new Point(pushSampleIntermediatary3), new Point(sample2PushIntermediatary)))
                //.setConstantHeadingInterpolation(pushSampleIntermediatary3.getHeading())
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(25.000, 24.000, Point.CARTESIAN),
                                new Point(55.000, 24.000, Point.CARTESIAN),
                                 new Point(60.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(sample2Pushed.getHeading())
                .addPath(new BezierCurve(new Point(sample2PushIntermediatary), new Point(sample2Pushed)))
                .setConstantHeadingInterpolation(sample2PushIntermediatary.getHeading())
                .addPath(new BezierCurve(new Point(sample2Pushed), new Point(sampleIntakePos)))
                .setConstantHeadingInterpolation(sample2Pushed.getHeading())
                .build();

      /*  pushSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(pushSampleIntermediatary)))
                .setConstantHeadingInterpolation(preloadScorePose.getHeading())
                .addPath(new BezierCurve(new Point(pushSampleIntermediatary), new Point(pushSampleIntermediatary2)))
                .setConstantHeadingInterpolation(pushSampleIntermediatary.getHeading())
                .addPath(new BezierCurve(new Point(pushSampleIntermediatary2), new Point(pushSampleIntermediatary3)))
                .setConstantHeadingInterpolation(pushSampleIntermediatary2.getHeading())
                .addPath(new BezierCurve(new Point(pushSampleIntermediatary3), new Point(samplePushed)))
                .setConstantHeadingInterpolation(pushSampleIntermediatary3.getHeading())
                .addPath(new BezierCurve(new Point(samplePushed), new Point(pushSampleIntermediatary3)))
                .setConstantHeadingInterpolation(samplePushed.getHeading())
                .addPath(new BezierCurve(new Point(pushSampleIntermediatary3), new Point(sample2PushIntermediatary)))
                .setConstantHeadingInterpolation(pushSampleIntermediatary3.getHeading())
                .addPath(new BezierCurve(new Point(sample2PushIntermediatary), new Point(sample2Pushed)))
                .setConstantHeadingInterpolation(sample2PushIntermediatary.getHeading())
                .addPath(new BezierCurve(new Point(sample2Pushed), new Point(sampleIntakePos)))
                .setConstantHeadingInterpolation(sample2Pushed.getHeading())
                .build();
      */

        scoreFirstSpec = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sampleIntakePos), new Point(firstSpecScorePose)))
                .setConstantHeadingInterpolation(sampleIntakePos.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(parkPose)))
                .setConstantHeadingInterpolation(pushSampleIntermediatary.getHeading())
                .build();
    }
    public void updatePath() {
        monkeyPawFSM.updateState(false,false,false,false,false,false,false,false,false,false,true);
        limbFSM.updateState(false,false,false,false,false,false,false,false,false,false,0,false,true);
        monkeyPawFSM.updatePID();
        limbFSM.updatePID(true);
        limbFSM.setMode(LimbFSM.Mode.SPECIMEN_MODE);
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setDepositSpecState(0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    depositSpec(2);
                }
                break;
            case 2:
                setDepositSpecState(0);
                if(!follower.isBusy()) {
                   // if(monkeyPawFSM.PREPARED_TO_INTAKE_SPECIMEN() && limbFSM.PREPARED_TO_INTAKE_SPECIMEN()) {
                        follower.followPath(pushSamples, true);
                        setPathState(3);
                   // }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    limbFSM.setArmPowerCap(0.2);
                    limbFSM.setStates(LimbFSM.States.AUTO_SPEC_INTAKING);
                    setPathState(4);
                }
                break;
            case 4:
                if(limbFSM.AUTO_SPEC_INTAKED()) {
                    monkeyPawFSM.setState(MonkeyPawFSM.States.INTAKING_SPECIMEN);
                    setPathState(5);
                }
                break;
            case 5:
                if(monkeyPawFSM.INTAKED_SPECIMEN()) {
                    follower.followPath(scoreFirstSpec, true);
                    limbFSM.setArmPowerCap(0.6);
                    limbFSM.setStates(LimbFSM.States.RETRACTING_FOR_AUTO);
                    //  if(limbFSM.RETRACTED_FOR_AUTO()) {  -- Not working for some reason with this
                    setPathState(6);
                    //  }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    depositSpec(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    // if(monkeyPawFSM.PREPARED_TO_INTAKE_SPECIMEN() && limbFSM.PREPARED_TO_INTAKE_SPECIMEN()) {
                        follower.followPath(park, true);
                        setPathState(8);
                    //}
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void depositSpec(int pathStateAfterDepositComplete){
        switch (depositSpecState) {
            case 0:
                if(!follower.isBusy()) {
                    limbFSM.setStates(LimbFSM.States.INTAKING_SPECIMEN);
                    monkeyPawFSM.setState(MonkeyPawFSM.States.INTAKING_SPECIMEN);
                    setDepositSpecState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(monkeyPawFSM.INTAKED_SPECIMEN() && limbFSM.INTAKED_SPECIMEN()) {
                        limbFSM.setStates(LimbFSM.States.EXTENDING_SPECIMEN);
                        monkeyPawFSM.setState(MonkeyPawFSM.States.GETTING_READY_TO_DEPOSIT_SPECIMEN);
                        setDepositSpecState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    if (monkeyPawFSM.READY_TO_DEPOSIT_SPECIMEN() && limbFSM.EXTENDED_SPECIMEN()) {
                        monkeyPawFSM.setState(MonkeyPawFSM.States.DEPOSITING_SPECIMEN);
                        setDepositSpecState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if (monkeyPawFSM.DEPOSITED_SPECIMEN()) {
                        limbFSM.setStates(LimbFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
                        monkeyPawFSM.setState(MonkeyPawFSM.States.PREPARING_TO_INTAKE_SPECIMEN);
                        setPathState(pathStateAfterDepositComplete);
                    }
                }
                break;

        }
    }
    // tune heading PID, battery voltage compensation,
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
            monkeyPawFSM.setState(MonkeyPawFSM.States.AUTO_START);
            monkeyPawFSM.updateState(false,false,false,false,false, false,false,false,false,false, true);
            monkeyPawFSM.updatePID();
            logger.log("Monkey Paw State", monkeyPawFSM.getState(), Logger.LogLevels.PRODUCTION);
            logger.print();
        }
        waitForStart();
        while (opModeIsActive()) {
            follower.setMaxPower(0.7);

           // follower.setMaxPower(0.7*(12.0/(hardwareMap.voltageSensor.iterator().next().getVoltage())));
            follower.update();
            updatePath();


            logger.log("voltage", hardwareMap.voltageSensor.iterator().next().getVoltage(), Logger.LogLevels.PRODUCTION);
            logger.log("x", follower.getPose().getX(), Logger.LogLevels.PRODUCTION);
            logger.log("y", follower.getPose().getY(), Logger.LogLevels.PRODUCTION);
            logger.log("heading", follower.getPose().getHeading(), Logger.LogLevels.PRODUCTION);
            logger.log("path state",pathState, Logger.LogLevels.PRODUCTION);
            logger.log("limb state", limbFSM.getStates(), Logger.LogLevels.PRODUCTION);
            logger.log("monkey paw state", monkeyPawFSM.getState(), Logger.LogLevels.PRODUCTION);
            logger.log("Deposit Specimen State", depositSpecState, Logger.LogLevels.PRODUCTION);
            logger.log("Robot at pos", !follower.isBusy(), Logger.LogLevels.PRODUCTION);
            monkeyPawFSM.log();
            limbFSM.log();
            logger.print();
        }

    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
    }

    public void setDepositSpecState(int depositSpecState) {
        this.depositSpecState = depositSpecState;
    }
}
