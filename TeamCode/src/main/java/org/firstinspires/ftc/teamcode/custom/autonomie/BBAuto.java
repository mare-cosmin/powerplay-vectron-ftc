package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BBTest", group="Autonomie")
public class BBAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.4, 63, Math.toRadians(360));
        drive.setPoseEstimate(startPose);

        double y = 5;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35.4, y))
                .lineToSplineHeading(new Pose2d(-6, y, Math.toRadians(220)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, y, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                //cone 1
                .lineToConstantHeading(new Vector2d(-35.4, y+2))
                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                //cone 2
                .lineToConstantHeading(new Vector2d(-35.4, y+2))
                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                //cone 3
                .lineToConstantHeading(new Vector2d(-35.4, y+2))
                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                //cone 4
                .lineToConstantHeading(new Vector2d(-35.4, y+2))
                .lineToSplineHeading(new Pose2d(-6, y+2, Math.toRadians(220)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, y+2, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                .splineToConstantHeading(new Vector2d(-50, y+2), Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })
                .build();

        waitForStart();

        if(!isStopRequested()){
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
