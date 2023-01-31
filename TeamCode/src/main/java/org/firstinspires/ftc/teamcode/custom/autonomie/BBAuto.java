package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="BBTest", group="Autonomie")
public class BBAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Detection detectare = new Detection();
        ElapsedTime runtime = new ElapsedTime();
        int tag = 0;
        detectare.initCV(hardwareMap);

        while(!isStopRequested() && !isStarted()){
            tag = detectare.detect();
            telemetry.addData("tag", tag);
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(-35.4, 63, Math.toRadians(360));
        drive.setPoseEstimate(startPose);

        double y = 5;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35.4, y))

//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })

//                .lineToSplineHeading(new Pose2d(-6, y, Math.toRadians(220)))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(-35.4, y, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)
//                //cone 1
//                .lineToConstantHeading(new Vector2d(-35.4, y+2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)
//                //cone 2
//                .lineToConstantHeading(new Vector2d(-35.4, y+2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)
//
//                //cone 3
//                .lineToConstantHeading(new Vector2d(-35.4, y+2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)
//
//                //cone 4
//                .lineToConstantHeading(new Vector2d(-35.4, y+2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToConstantHeading(new Vector2d(-35.4, y+2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)

                .build();

        waitForStart();

        if(!isStopRequested()){
            drive.followTrajectorySequence(trajSeq);
            if(tag == 4) {
//                TrajectorySequence park2Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
////                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
////                            //lift down and open gripper
////                        })
//                        .lineToSplineHeading(new Pose2d(35.4, -y, Math.toRadians(0)))
//                        .build();
//                drive.followTrajectorySequence(park2Seq);
            }else {
                if (tag == 5) {
                    TrajectorySequence park3Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                //lift down and open gripper
//                            })
                            .lineToConstantHeading(new Vector2d(-60, y))
                            .build();
                    drive.followTrajectorySequence(park3Seq);
                } else {
                    TrajectorySequence park1Seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                //lift down and open gripper
//                            })
                            .lineToConstantHeading(new Vector2d(-12, y-3))
                            .build();
                    drive.followTrajectorySequence(park1Seq);
                }
            }
        }
        while(opModeIsActive()){
            idle();
        }
    }
}
