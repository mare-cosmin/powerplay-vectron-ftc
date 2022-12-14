package org.firstinspires.ftc.teamcode.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.HardwareTeleOp;

@Autonomous(name="testAuto", group="Autonomie")
public class Autonomie extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareTeleOp robot = new HardwareTeleOp();
    Detection detectare = new Detection();
    int tag;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);
        if(!opModeIsActive() && !isStopRequested()){
            sleep(100);
            detectare.initCV();
            while (tag == 0){
                tag = detectare.detect();
            }
        }

        waitForStart();

        while(opModeIsActive()){
            robot.driveToPosition(1, 70);
            switch(tag){
                case 3:
                    robot.strafeToPosition(1, 60);
                    telemetry.addData("parking", "Left");
                    break;
                case 4:
                    telemetry.addData("parking", "Center");
                    break;
                case 5:
                    robot.strafeToPosition(1, 60);
                    telemetry.addData("parking", "Right");
                    break;
            }

        }
    }
}