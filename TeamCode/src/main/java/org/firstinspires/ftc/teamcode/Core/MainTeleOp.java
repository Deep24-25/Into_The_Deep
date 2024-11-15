package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Teleop.Monkeys_Limb.LimbFSM;

public class MainTeleOp extends LinearOpMode {
    private LimbFSM limbFSM;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            limbFSM = new LimbFSM();
        }catch (Exception e){
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
