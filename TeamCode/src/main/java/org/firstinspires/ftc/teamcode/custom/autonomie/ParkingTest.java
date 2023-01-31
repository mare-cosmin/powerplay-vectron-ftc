package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.custom.teleop.TeleOpAdaptedMecanumDrive;

@Disabled
@Autonomous(name="testParking", group="Autonomie")
public class ParkingTest extends LinearOpMode {
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
        telemetry.addData("before", "before");
        telemetry.update();
        while(!isStopRequested() && !isStarted()){
            tag = detectare.detect();
            telemetry.addData("tag", tag);
            telemetry.update();
        }
        telemetry.addData("after", "after");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        telemetry.addData("tag", tag);
        telemetry.update();

        waitForStart();
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