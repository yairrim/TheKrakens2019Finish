package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="Example: Gold Vision Demo")
public class ExampleGoldVisionDemo extends LinearOpMode {
    private ExampleGoldVision blueVision;

    @Override
    public void runOpMode() {
        blueVision = new ExampleGoldVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowCountours(false);

        // start the vision system
        blueVision.enable();
        telemetry.addData("d","d");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // update the settings of the vision pipeline
            // get a list of contours from the vision system
            int Y;
            int X;
            int Mid = 300;
            List<MatOfPoint> contours = blueVision.getContours();
            if (contours.isEmpty()) {
                telemetry.addData(">", "right");
            } else {
                for (int i = 0; i < contours.size(); i++) {
                    // get the bounding rectangle of a single contour, we use it to get the x/y center
                    // yes there's a mass center using Imgproc.moments but w/e

                    Rect boundingRect = Imgproc.boundingRect(contours.get(i));

                    Y = boundingRect.y;
                    X = boundingRect.x;
                    telemetry.addData(">", "X-" + X + ",Y-" + Y);
                    if (Y < Mid) {
                        telemetry.addData(">", "Left");
                    } else if (Y > Mid) {
                        telemetry.addData(">", "center");
                    }
                    telemetry.update();
                }
            }
        }
    }
}
