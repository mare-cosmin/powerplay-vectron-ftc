package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.custom.teleop.TeleOpAdaptedMecanumDrive;

@Autonomous(name="testAutoParking", group="Autonomie")
public class AutonomieParking extends LinearOpMode {
    TeleOpAdaptedMecanumDrive robot;
    Detection detectare = new Detection();
    ElapsedTime runtime = new ElapsedTime();
    int tag = 0;
    boolean first_time = true;

    public void init(HardwareMap hardwareMap) {
        robot = new TeleOpAdaptedMecanumDrive(hardwareMap);
        robot.initialize(hardwareMap);
        sleep(100);
        detectare.initCV(hardwareMap);
        robot.stopAndResetEncodersCHASSIS();
        while(!opModeIsActive()){
            tag = detectare.detect();
            telemetry.addData("tag", tag);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        telemetry.addData("tag", tag);
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;
        runtime.reset();
        double timeflag = 15;
        sleep(300);
        while (tag == 0 && runtime.seconds() <= timeflag && !isStopRequested()){
            tag = detectare.detect();
        }
        robot.closeGripper();
        sleep(600);
        robot.auto_position();
        sleep(600);
        robot.moveRight(1, 80);
        if(tag != 0) {
            if (first_time) {
                sleep(2000);
                robot.setPowerZeroCHASSIS();
                robot.stopAndResetEncodersCHASSIS();
                first_time = false;
            }
            switch (tag) {
                case 3:
                    robot.moveForward(1, 55);
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
        }
        robot.closeGripper();
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