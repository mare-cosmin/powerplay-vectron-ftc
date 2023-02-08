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
        while(opModeInInit()){
            tag = detectare.detect();
            telemetry.addData("tag", (tag == 0 ? tag : tag%3+1));
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        telemetry.addData("tag", tag%3+1);
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        robot.closeGripper();
        sleep(600);
        robot.auto_position();
        sleep(600);
        robot.moveForward(1, 100);
//        if(tag != 0) {
//            if (first_time) {
//                sleep(2000);
//                robot.setPowerZeroCHASSIS();
//                robot.stopAndResetEncodersCHASSIS();
//                first_time = false;
//            }
//            switch (tag) {
//                case 3:
//                    robot.moveForward(0.8, 58);
//                    telemetry.addData("parking", "Left");
//                    break;
//
//                case 4:
//                    robot.moveForward(0.3, 2);
//                    telemetry.addData("parking", "Center");
//                    break;
//                case 5:
//                    robot.moveReverse(0.8, 55);
//                    telemetry.addData("parking", "Right");
//                    break;
//            }
//            telemetry.update();
//        }
//        robot.closeGripper();
//        sleep(2000);
//        robot.servo_brat_jos.setPosition(0);

        while (opModeIsActive()){
            idle();
        }
    }
}