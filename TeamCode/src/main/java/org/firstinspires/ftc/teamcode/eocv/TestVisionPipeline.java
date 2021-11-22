package org.firstinspires.ftc.teamcode.eocv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestVisionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    static final Rect LEFTBOX = new Rect(
            new Point(0, 40),
            new Point(106, 140)
    );
    static final Rect CENTERBOX = new Rect(
            new Point(106, 40),
            new Point(212, 140)
    );
    static final Rect RIGHTBOX = new Rect(
            new Point(212, 40),
            new Point(320, 140)
    );

    public enum POS {
        LEFT,
        RIGHT,
        CENTER
    }
    private POS pos;

    public TestVisionPipeline(Telemetry t) {
        telemetry = t;
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Scalar lowHSVGreen = new Scalar(35, 50, 70); // green test lower
        Scalar highHSVGreen = new Scalar(70, 255, 255); // green test upper

        Scalar lowHSVPurple = new Scalar(110, 50, 70); // purple test lower
        Scalar highHSVPurple = new Scalar(145, 255, 255); // purple test upper

        Core.inRange(mat, lowHSVPurple, highHSVPurple, mat);

        Mat left = mat.submat(LEFTBOX);
        Mat center = mat.submat(CENTERBOX);
        Mat right = mat.submat(RIGHTBOX);

        double leftValue = Core.sumElems(left).val[0] / LEFTBOX.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTERBOX.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHTBOX.area() / 255;

        left.release();
        center.release();
        right.release();

        double maxLeftRight = Math.max(leftValue, rightValue);
        double max = Math.max(maxLeftRight, centerValue);

        if (max == leftValue)
           pos = POS.LEFT;
        else if (max == centerValue)
            pos = POS.CENTER;
        else if (max == rightValue)
            pos = POS.RIGHT;

        telemetry.addData("Vision target location: ", pos);
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar colorTarget = new Scalar(0, 255, 0);
        Scalar colorEmpty = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFTBOX, pos == POS.LEFT ? colorTarget:colorEmpty);
        Imgproc.rectangle(mat, CENTERBOX, pos == POS.CENTER ? colorTarget:colorEmpty);
        Imgproc.rectangle(mat, RIGHTBOX, pos == POS.RIGHT ? colorTarget:colorEmpty);

        return mat;
    }

    public POS getPosition() {
        return pos;
    }
}
