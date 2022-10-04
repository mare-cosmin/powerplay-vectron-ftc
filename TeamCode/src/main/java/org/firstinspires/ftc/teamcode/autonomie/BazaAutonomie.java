package org.firstinspires.ftc.teamcode.autonomie;

//import necessary ftc packages
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//import easy opencv packages
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public abstract class BazaAutonomie extends LinearOpMode{
    OpenCvCamera webcam;
    SignalReaderPipeline pipeline;


    @Override
    public void runOpMode() {
        HardwareAutonomie robot = new HardwareAutonomie(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new SignalReaderPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // supported camera resolutions (for the gold E4) are: 1280x720, 960x720, 768x432, 720x480, 640x480, 320x240, 176x144
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int i) {
                // display given error because I don't know what else to do
                telemetry.addData("Error integer", i);
            }
        });
        while(opModeIsActive()){
            telemetry.addData("TSE position", pipeline.position);
            telemetry.addData("\ngreen value", pipeline.greenValue);
            telemetry.addData("red value", pipeline.redValue);
            telemetry.addData("blue value", pipeline.blueValue);

            telemetry.addData("hue value", pipeline.hueValue);
            telemetry.addData("saturation value", pipeline.saturationValue);
            telemetry.addData("value value", pipeline.valueValue);

            telemetry.addData("green before", pipeline.greenBefore);
            telemetry.addData("red before", pipeline.redBefore);
            telemetry.addData("blue before", pipeline.blueBefore);
            telemetry.update();
        }
        telemetry.clear();
        telemetry.update();
        //runs autonomous code
        runRobotAuto();
    }

    public void runRobotAuto() {

    }
}
