package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BRTest", group="Autonomie")
public class BRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35.4, 63, Math.toRadians(360));
        drive.setPoseEstimate(startPose);

        float y = 2;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .lineToSplineHeading(new Pose2d(35.4, y, Math.toRadians(310)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //lift up
                })
                .splineToConstantHeading(new Vector2d(6, y), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(35.4, y+2, Math.toRadians(360)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(50, y+2), Math.toRadians(360))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })

                //cone 2
                .lineToConstantHeading(new Vector2d(35.4, y+2))
                .lineToSplineHeading(new Pose2d(6, y+2, Math.toRadians(310)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(35.4, y+2, Math.toRadians(360)))
                .splineToConstantHeading(new Vector2d(50, y+2), Math.toRadians(360))
                //cone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                //cone 1
                .lineToConstantHeading(new Vector2d(35.4, y+2))
                .lineToSplineHeading(new Pose2d(6, y+2, Math.toRadians(310)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(35.4, y+2, Math.toRadians(360)))
                .splineToConstantHeading(new Vector2d(50, y+2), Math.toRadians(360))
                //cone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .lineToConstantHeading(new Vector2d(35.4, y+2))
                .lineToSplineHeading(new Pose2d(6, y+2, Math.toRadians(310)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(35.4, y+2, Math.toRadians(360)))
                .splineToConstantHeading(new Vector2d(50, y+2), Math.toRadians(360))
                //cone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .lineToConstantHeading(new Vector2d(35.4, y+2))
                .lineToSplineHeading(new Pose2d(6, y+2, Math.toRadians(310)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(35.4, y+2, Math.toRadians(360)))
                .splineToConstantHeading(new Vector2d(50, y+2), Math.toRadians(360))
                //cone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .build();

        waitForStart();

        if(!isStopRequested()){
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
