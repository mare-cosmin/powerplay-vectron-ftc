package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

//@Disabled
@Autonomous(name="RRTest", group="Autonomie")
public class RRAuto extends LinearOpMode {
    private double y = 11;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Detection detectare = new Detection();
        ElapsedTime runtime = new ElapsedTime();
        int tag = 0;
        detectare.initCV(hardwareMap);

        Pose2d startPose = new Pose2d(-35.4, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        drive.closeGripper();
        drive.cone_up_low_fata();

        double x_pickup = -55;
        double x_drop = -10;
        double x_mid = -35.4;

        TrajectorySequenceBuilder trajSeqbuild = drive.trajectorySequenceBuilder(startPose)
                //preload
                .lineToConstantHeading(new Vector2d(x_mid, -y))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })

                .splineToConstantHeading(new Vector2d(x_mid, -y-2), Math.toRadians(0))
                .waitSeconds(0.7)

                .lineToSplineHeading(new Pose2d(x_drop, -y-2, Math.toRadians(130)))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    //lift down and open gripper
                })

                .lineToSplineHeading(new Pose2d(x_mid, -y-2, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(x_pickup, -y-2), Math.toRadians(180))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                .waitSeconds(0.5)

                //cone 1
                .UNSTABLE_addTemporalMarkerOffset(0, this::decrementY)

                .lineToConstantHeading(new Vector2d(x_mid, -y-2))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })

                .lineToSplineHeading(new Pose2d(x_drop, -y-2, Math.toRadians(130)))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .waitSeconds(0.5)

                .lineToSplineHeading(new Pose2d(x_mid, -y-2, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(x_pickup, -y-2), Math.toRadians(180))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                .waitSeconds(0.5)

                //cone 2
                .UNSTABLE_addTemporalMarkerOffset(0, this::decrementY)

                .lineToConstantHeading(new Vector2d(x_mid, -y-2))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })

                .lineToSplineHeading(new Pose2d(x_drop, -y-2, Math.toRadians(130)))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .waitSeconds(0.5)

                .lineToSplineHeading(new Pose2d(x_mid, -y-2, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(x_pickup, -y-2), Math.toRadians(180))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                .waitSeconds(0.5)

                //cone 3
                .UNSTABLE_addTemporalMarkerOffset(0, this::decrementY)

                .lineToConstantHeading(new Vector2d(x_mid, -y-2))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })

                .lineToSplineHeading(new Pose2d(x_drop, -y-2, Math.toRadians(130)))

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .waitSeconds(0.5);

//                .lineToSplineHeading(new Pose2d(x_mid, -y-2, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(x_pickup, -y-2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5)
//
//                //cone 4
//                .lineToConstantHeading(new Vector2d(x_mid, -y-2))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift up
//                })
//
//                .lineToSplineHeading(new Pose2d(x_drop, -y-2, Math.toRadians(130)))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //drop cone
//                })
//                .waitSeconds(0.5)
//
//                .lineToSplineHeading(new Pose2d(x_mid, -y-2, Math.toRadians(180)))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    //lift down and open gripper
//                })
//
//                .splineToConstantHeading(new Vector2d(x_pickup, -y-2), Math.toRadians(180))
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //pickup cone
//                })
//                .waitSeconds(0.5);
        telemetry.addData("init", "done");

        while(opModeInInit()){
            tag = detectare.detect();
            telemetry.addData("tag", tag);
            telemetry.update();
        }



        waitForStart();

        if(!isStopRequested()){
            if(tag == 4) {
                TrajectorySequence park2Seq = trajSeqbuild
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //lift down and open gripper
                        })
                        .lineToSplineHeading(new Pose2d(-35.4, -y-2, Math.toRadians(180)))
                        .build();
                drive.followTrajectorySequence(park2Seq);
            }else {
                if (tag == 3) {
                    TrajectorySequence park1Seq = trajSeqbuild
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                //lift down and open gripper
                            })
                            .lineToConstantHeading(new Vector2d(-60, -y-2))
                            .build();
                    drive.followTrajectorySequence(park1Seq);
                } else {
                    TrajectorySequence park3Seq = trajSeqbuild
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                //lift down and open gripper
                            })
                            .lineToConstantHeading(new Vector2d(-12, -y+1.5))
                            .build();
                    drive.followTrajectorySequence(park3Seq);
                }
            }
        }
        while(opModeIsActive()){
            idle();
        }
    }
    private void decrementY(){
        --y;
    }
}
