package org.firstinspires.ftc.teamcode.custom.autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RRTest", group="Autonomie")
public class RRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.4, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        float y = 10 ;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //preload
                .lineToSplineHeading(new Pose2d(-35.4, -y, Math.toRadians(130)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //lift up
                })
                .splineToConstantHeading(new Vector2d(-6, -y), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, -y-2, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })

                .splineToConstantHeading(new Vector2d(-50, -y-2), Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //pickup cone
                })

                //cone 2
                .lineToConstantHeading(new Vector2d(35.4, y+2))
                .lineToSplineHeading(new Pose2d(6, y+2, Math.toRadians(130)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(35.4, y+2, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(50, y+2), Math.toRadians(180))
                //cone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                //cone 1
                .lineToConstantHeading(new Vector2d(-35.4, -y-2))
                .lineToSplineHeading(new Pose2d(-6, -y-2, Math.toRadians(130)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, -y-2, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-50, -y-2), Math.toRadians(180))
                //cone 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                //cone 1
                .lineToConstantHeading(new Vector2d(-35.4, -y-2))
                .lineToSplineHeading(new Pose2d(-6, -y-2, Math.toRadians(130)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .lineToSplineHeading(new Pose2d(-35.4, -y-2, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-50, -y-2), Math.toRadians(180))
                //cone 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift down and open gripper
                })
                //cone 1
                .lineToConstantHeading(new Vector2d(-35.4, -y-2))
                .lineToSplineHeading(new Pose2d(-6, -y-2, Math.toRadians(130)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //lift up
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .build();

        waitForStart();

        if(!isStopRequested()){
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
