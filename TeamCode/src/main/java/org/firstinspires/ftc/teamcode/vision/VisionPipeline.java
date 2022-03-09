package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    
    public static Rect LEFTBOX = new Rect(
            new Point(5, 30),
            new Point(90, 100)
    );
    public static Rect CENTERBOX = new Rect(
            new Point(115, 30),
            new Point(195, 100)
    );
    public static Rect RIGHTBOX = new Rect(
            new Point(220, 30),
            new Point(305, 100)
    );

    public Scalar lowHSV = new Scalar(150, 50, 50);
    public Scalar highHSV = new Scalar(180, 255, 255);

    public enum POS {
        LEFT,
        RIGHT,
        CENTER
    }
    private POS pos;

    public VisionPipeline(Telemetry t) {
        telemetry = t;
    }

    public Mat processFrame(Mat input) {
        // convert RBB to HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // convert to b/w by removing all colors not in range
        Core.inRange(mat, lowHSV, highHSV, mat);

        // create 3 submats for left/right/center
        Mat left = mat.submat(LEFTBOX);
        Mat center = mat.submat(CENTERBOX);
        Mat right = mat.submat(RIGHTBOX);

        // get val for each submat by dividing average hue by area of box
        double leftValue = Core.sumElems(left).val[0] / LEFTBOX.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTERBOX.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHTBOX.area() / 255;

        left.release();
        center.release();
        right.release();

        // max = max value (highest amount of green/yellow)
        double maxLeftRight = Math.max(leftValue, rightValue);
        double max = Math.max(maxLeftRight, centerValue);

        if (max == leftValue)
           pos = POS.RIGHT; // swapped bc camera upside down
        else if (max == centerValue)
            pos = POS.CENTER;
        else if (max == rightValue)
            pos = POS.LEFT; // swapped bc camera upside down

        telemetry.addData("Vision target location: ", pos);
        telemetry.update();

        // b/w to RGB
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar colorTarget = new Scalar(0, 255, 0);
        Scalar colorEmpty = new Scalar(255, 0, 0);

        // draw green rect on target box/red on empty box
        Imgproc.rectangle(mat, RIGHTBOX, pos == POS.LEFT ? colorTarget:colorEmpty);
        Imgproc.rectangle(mat, CENTERBOX, pos == POS.CENTER ? colorTarget:colorEmpty);
        Imgproc.rectangle(mat, LEFTBOX, pos == POS.RIGHT ? colorTarget:colorEmpty);

        return mat;
    }

    public POS getPosition() {
        return pos;
    }
}
