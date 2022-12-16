package org.firstinspires.ftc.teamcode.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.HardwareTeleOp;

@Autonomous(name="testAutoRLow", group="Autonomie")
public class AutonomieRLow extends LinearOpMode {
    HardwareTeleOp robot;
        Detection detectare = new Detection();
    int tag = 0;
    boolean first_time = true;

    public void init(HardwareMap hardwareMap) {
        robot = new HardwareTeleOp();
        robot.initialize(hardwareMap);
        robot.cone_up_low_fata();
        sleep(100);
        detectare.initCV(hardwareMap);
        while (tag == 0){
            tag = detectare.detect();
        }
        robot.stopAndResetEncodersCHASSIS();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        telemetry.addData("tag", tag);
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;
        robot.moveRight(1, 50);
        sleep(500);
        robot.moveForward(1, 5);
        sleep(300);
        robot.moveReverse(1, 5);
        robot.moveRight(1, 20);

        if(first_time) {
            sleep(2000);
            robot.setPowerZeroCHASSIS();
            robot.stopAndResetEncodersCHASSIS();
            first_time = false;
        }
        switch (tag) {
            case 3:
                robot.moveForward(1, 60);
                telemetry.addData("parking", "Left");
                break;

            case 4:
                telemetry.addData("parking", "Center");
                break;
            case 5:
                robot.moveReverse(1, 60);
                telemetry.addData("parking", "Right");
                break;
        }
        sleep(30000);
        while(!isStopRequested() && opModeIsActive()){


            telemetry.addData("tag", tag);
            telemetry.addData("tpos", robot.leftFront.getTargetPosition());
            telemetry.addData("cpos", robot.leftFront.getCurrentPosition());
            telemetry.addData("pow", robot.leftFront.getPower());
            telemetry.addData("wheel", robot.leftFront.getDeviceName());
            telemetry.update();
        }
        while (!isStopRequested()){
            idle();
        }
    }
}