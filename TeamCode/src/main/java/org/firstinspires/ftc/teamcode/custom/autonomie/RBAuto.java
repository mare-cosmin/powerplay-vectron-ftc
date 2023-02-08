package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.custom.teleop.TeleOpAdaptedMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="RBTest", group="Autonomie")
public class RBAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.servo_brat_sus.setPosition(1);
        Detection detectare = new Detection();
        ElapsedTime runtime = new ElapsedTime();
        int tag = 0;
        detectare.initCV(hardwareMap);

        while (opModeInInit()) {
            tag = detectare.detect();
            telemetry.addData("tag", tag);
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(35.4, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
//        drive.cone_down_low_fata();

        double x_pickup = 50.65;
        double x_drop = 4;
        double x_mid = 35.4;

        double y = 11;
        TrajectorySequence baseSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(x_mid, -y + 6))

                .lineToSplineHeading(new Pose2d(x_mid+1, -y-2, Math.toRadians(120)))

                .UNSTABLE_addTemporalMarkerOffset(0, drive::cone_up_high_fata)

                .waitSeconds(2)

//                .turn(-Math.toRadians(23))
//                .forward(1.5)
//                .strafeLeft(3)

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::openGripper)
                .waitSeconds(0.5)

//                .turn(-Math.toRadians(155))

//                .waitSeconds(0.5)

//                .lineToSplineHeading(new Pose2d(x_mid, -y, Math.toRadians(0)))

                .splineToSplineHeading(new Pose2d(x_pickup+2, -y-2, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    drive.pickup_stack(1);
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.servo_brat_sus.setPosition(0.65);
                })

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::closeGripper)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::cone_up_low_fata)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    drive.servo_brat_sus.setPosition(1);
                })
                .waitSeconds(0.5)

                //cone 1
                .lineToConstantHeading(new Vector2d(x_mid, -y))

                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_up_high_spate)


                .lineToSplineHeading(new Pose2d(x_drop + 1, -y-5, Math.toRadians(35)))

                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::openGripper)
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    drive.pickup_stack(2);
                })

                .lineToSplineHeading(new Pose2d(x_mid-1, -y+3, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(x_pickup + 3, -y - 1), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.servo_brat_sus.setPosition(0.65);
                })

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::closeGripper)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::cone_up_low_fata)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    drive.servo_brat_sus.setPosition(1);
                })
                .waitSeconds(0.5)

                //cone 2

                .lineToConstantHeading(new Vector2d(x_mid, -y + 2))

                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_up_high_spate)

                .lineToSplineHeading(new Pose2d(x_drop + 8, -y - 2, Math.toRadians(50)))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::openGripper)
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    drive.pickup_stack(3);
                })

                .lineToSplineHeading(new Pose2d(x_mid, -y + 4, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(x_pickup + 6, -y + 5 ), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.servo_brat_sus.setPosition(0.68);
                })

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::closeGripper)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::cone_up_low_fata)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    drive.servo_brat_sus.setPosition(1);
                })
                .waitSeconds(0.5)

                //cone 3

                .lineToConstantHeading(new Vector2d(x_mid, -y + 4))

                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_up_high_spate)


                .lineToSplineHeading(new Pose2d(x_drop + 12, -y - 3, Math.toRadians(50)))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::openGripper)
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(35.4, -y-2, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(50, -y-2), Math.toRadians(0))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                .waitSeconds(0.5)


                //cone 4
                .lineToConstantHeading(new Vector2d(35.4, -y-2))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })

                .lineToSplineHeading(new Pose2d(6, -y-2, Math.toRadians(50)))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .waitSeconds(0.5)

                .lineToSplineHeading(new Pose2d(35.4, -y-2, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(50, -y-2), Math.toRadians(0))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                .waitSeconds(0.5)

                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(baseSeq);
            if (tag == 4) {
                TrajectorySequence park2Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(x_mid-5, -y + 3, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_down_low_fata)
                        .build();
                drive.followTrajectorySequence(park2Seq);
            } else {
                if (tag == 3) {
                    TrajectorySequence park1Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(x_drop+5, -y + 3, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_down_low_fata)
                            .build();
                    drive.followTrajectorySequence(park1Seq);
                } else {
                    TrajectorySequence park3Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                                drive.pickup_stack(1);
                            })
                            .lineToLinearHeading(new Pose2d(x_pickup+7, -y + 3, Math.toRadians(0)))

                            .build();
                    drive.followTrajectorySequence(park3Seq);
                }
            }
        }
        while (opModeIsActive()) {
            idle();
        }
    }
}
