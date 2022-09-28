package org.firstinspires.ftc.teamcode.autonomie;

//import necessary opencv packages
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//instantiating the class
public class SignalReaderPipeline extends OpenCvPipeline {
    private final Mat firstFrame = new Mat();
    public int position = 0; //0 = unknown; 1 = left; 2 = center; 3 = right;
    public final int sleeveX = 0;
    public final int sleeveY = 0;
    public final int sleeveWidth = 1200;
    public final int sleeveHeight = 700;

    public double blueValue, redValue, greenValue;

    public double hueValue, saturationValue, valueValue;

    public double blueBefore, redBefore, greenBefore;



    @Override
    public Mat processFrame(Mat input) {


        return firstFrame;
    }
}
