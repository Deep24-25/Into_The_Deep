package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;
import org.firstinspires.ftc.teamcode.Teleop.monkeypaw.MonkeyPawFSM;

public class MainTeleOp extends LinearOpMode {
    private HWMap hwMap;
    private Logger logger;
    private LimbFSM limbFSM;
    private MonkeyPawFSM monkeyPawFSM;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            hwMap = new HWMap(hardwareMap);
            limbFSM = new LimbFSM();
            monkeyPawFSM = new MonkeyPawFSM(hwMap,logger,limbFSM);
        }catch (Exception e){
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
