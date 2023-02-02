package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="RBTest", group="Autonomie")
public class RBAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

        double x_pickup = 51;
        double x_drop = 4;
        double x_mid = 35.4;

        double y = 9;
        TrajectorySequence baseSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(x_mid, -y + 10))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(x_mid, -y))

                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_up_high_fata)

                .turn(-Math.toRadians(25))
                .forward(1.5)
//                .strafeLeft(3)
//                .lineToSplineHeading(new Pose2d(x_drop, -y, Math.toRadians(50)))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::openGripper)
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    drive.pickup_stack(1);
                })

                .turn(-Math.toRadians(155))

                .waitSeconds(0.5)

//                .lineToSplineHeading(new Pose2d(x_mid, -y, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(x_pickup, -y), Math.toRadians(0))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::closeGripper)
                .UNSTABLE_addTemporalMarkerOffset(0.1, drive::cone_up_low_fata)
                .waitSeconds(1)

                //cone 1
                .lineToConstantHeading(new Vector2d(x_mid, -y))

                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_up_high_spate)


                .lineToSplineHeading(new Pose2d(x_drop + 4, -y-2, Math.toRadians(50)))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::openGripper)
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    drive.pickup_stack(2);
                })

                .lineToSplineHeading(new Pose2d(x_mid, -y, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(x_pickup + 2, -y), Math.toRadians(0))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::closeGripper)
                .UNSTABLE_addTemporalMarkerOffset(0.1, drive::cone_up_low_fata)
                .waitSeconds(1)

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

                .splineToConstantHeading(new Vector2d(x_pickup + 4, -y + 4), Math.toRadians(0))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::closeGripper)
                .UNSTABLE_addTemporalMarkerOffset(0.1, drive::cone_up_low_fata)
                .waitSeconds(1)

                //cone 3

                .lineToConstantHeading(new Vector2d(x_mid, -y + 4))

                .UNSTABLE_addTemporalMarkerOffset(0.5, drive::cone_up_high_spate)


                .lineToSplineHeading(new Pose2d(x_drop + 12, -y - 3, Math.toRadians(50)))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, drive::openGripper)
                .waitSeconds(0.5)
////                .lineToSplineHeading(new Pose2d(35.4, -y-2, Math.toRadians(0)))
////
////                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
////                    //lift down and open gripper
////                })
////
////                .splineToConstantHeading(new Vector2d(50, -y-2), Math.toRadians(0))
////
////                .waitSeconds(0.5)
////                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)


                //cone 4
//                .lineToConstantHeading(new Vector2d(35.4, -y-2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(6, -y-2, Math.toRadians(50)))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(35.4, -y-2, Math.toRadians(0)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(50, -y-2), Math.toRadians(0))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)

                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(baseSeq);
            if (tag == 4) {
                TrajectorySequence park2Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            drive.pickup_stack(4);
                        })
                        .lineToLinearHeading(new Pose2d(12, -y + 3, Math.toRadians(0)))
                        .build();
                drive.followTrajectorySequence(park2Seq);
            } else {
                if (tag == 5) {
                    TrajectorySequence park3Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                drive.pickup_stack(4);
                            })
                            .lineToLinearHeading(new Pose2d(12, -y + 3, Math.toRadians(0)))
                            .build();
                    drive.followTrajectorySequence(park3Seq);
                } else {
                    TrajectorySequence park1Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                drive.pickup_stack(4);
                            })
                            .lineToLinearHeading(new Pose2d(12, -y + 3, Math.toRadians(0)))
                            .build();
                    drive.followTrajectorySequence(park1Seq);
                }
            }
        }
        while (opModeIsActive()) {
            idle();
        }
    }
}
