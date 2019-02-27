/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "TestFunctionGoldMineral", group = "Test")
public class TestFunctionGoldMineral extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private ElapsedTime activetime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    public double ArmSpeed = 0.4;
    static final double HEADING_THRESHOLD = 1;
    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;
    public String GoldMineralPosition="Unknown";
    int goldMineralX = -1;
    int silverMineral1X = -1;
    int silverMineral2X = -1;
    int silver1x = -1;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AS8eG1D/////AAABmX48aAq78EHqo6ydKng94/FbHwnQDn1IzrimDYhxOoE71bLZBHIqJVuXSnzlk7QCiVGgw9m+vUNJwBhvlcsMIgFm3kZ1xzZ75yW9WMgYim/dNvB4ku54BlppUz/6E1od3CJE0LjheC1aRX+FddticX8gnCF4hgfWM9vPTNFv8LTr2nhLDju+pGEBr2Vq9mc9Lzu4nfHT/8VWwjo8iBGx8z5hoTg+x94sgqOcdJjv1lx5XFmW+q6w45rtJrv41fOBTZ7xJIZ60t8ztt5IP868y1h2QRIUiorzZQQevkBlP/vn39kjrhqLfagfcVZ10gmI2zR+/ZLugk5qs1Zzh4jQRKXCMr77m0TWbROUtPMJIQIx";

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        robot.init(hardwareMap);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        // make sure the gyro is calibrated before continuing
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        /** Wait for the game to begin */

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        // make sure the gyro is calibrated before continuing
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        GoldRun();
        while (opModeIsActive()) {
            telemetry.addData("finish", "finish");
            telemetry.addData("pos", GoldMineralPosition);
            telemetry.addData("gold",goldMineralX);
            telemetry.addData("silver1",silverMineral1X);
            telemetry.addData("silver2",silverMineral2X);
            telemetry.update();
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */

    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        distance = distance * 25;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance);
            newLeftTarget = robot.MotorLeftFront.getCurrentPosition() + moveCounts;
            newRightTarget = robot.MotorRightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.MotorLeftFront.setTargetPosition(newLeftTarget);
            robot.MotorRightFront.setTargetPosition(newRightTarget);

            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.MotorLeftFront.setPower(speed);
            robot.MotorRightFront.setPower(speed);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.AllWheelsPower(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.MotorLeftFront.isBusy() &&
                            robot.MotorRightFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.AllWheelsPower(leftSpeed, rightSpeed);


            }

            // Stop all motion;
            robot.AllWheelsPower(0, 0);

            // Turn off RUN_TO_POSITION
            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.AllWheelsPower(0, 0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.AllWheelsPower(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData(">", "Robot Heading = %f", getGyroAngle());

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public float getGyroAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void ArmDownFromStart(double speed) {
        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDownR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = robot.ArmUpDownR.getCurrentPosition() + 1700;
        robot.ArmUpDownR.setTargetPosition(Target);

        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDownR.isBusy()) {
            robot.ArmUpDownR.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }


    }

    public void ArmDown(double speed) {
        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDownR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = robot.ArmUpDownR.getCurrentPosition() + 850;
        robot.ArmUpDownR.setTargetPosition(Target);

        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDownR.isBusy()) {
            robot.ArmUpDownR.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }


    }

    public void ArmUp(double speed) {
        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDownR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = robot.ArmUpDownR.getCurrentPosition() - 850;
        robot.ArmUpDownR.setTargetPosition(Target);

        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDownR.isBusy()) {
            robot.ArmUpDownR.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }


    }



    public void MineralIn() {
        runtime.reset();
        while (runtime.seconds() < 1) {
            robot.c.setPower(1);
        }
    }

    public void MineralOut() {
        runtime.reset();
        while (runtime.seconds() < 1) {
            robot.c.setPower(-1);
        }
    }

    public void posL() {
        gyroTurn(0.5, -30);
        ArmDownFromStart(ArmSpeed);
        gyroDrive(0.5, 80, -30);
        gyroDrive(0.5, -80, -30);
        gyroTurn(0.5, 0);
        gyroDrive(0.5, 80, 0);

    }

    public void posR() {
        gyroTurn(0.5, 30);
        ArmDownFromStart(ArmSpeed);
        gyroDrive(0.5, 80, 30);
        gyroDrive(0.5, -80, 30);
        gyroTurn(0.5, 0);
        gyroDrive(0.5, 80, 0);

    }

    public void posC() {
        ArmDownFromStart(ArmSpeed);
        gyroDrive(0.5, 80, 0);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void  GoldRun(){
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (GoldMineralPosition=="Unknown") {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.addData("gold",goldMineralX);
                        telemetry.addData("silver1",silverMineral1X);
                        telemetry.addData("silver2",silverMineral2X);
                        if (updatedRecognitions.size() == 2 ) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if(silverMineral1X != -1 && silverMineral2X == -1){
                                silver1x=silverMineral1X;
                            }

                            if(silverMineral1X == -1 && silverMineral2X != -1){
                                silver1x=silverMineral2X;
                            }
                            if(goldMineralX!=-1 && silver1x!=-1) {
                                if (goldMineralX < silver1x) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    GoldMineralPosition = "C";
                                } else if (goldMineralX > silver1x) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    GoldMineralPosition = "L";
                                }
                            }
                            else{
                                telemetry.addData("Gold Mineral Position", "Right");
                                GoldMineralPosition="R";
                            }
                            if(goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1){
                                telemetry.addData("you are dumb", "Need 3 object detection");
                            }


                            telemetry.update();
                        }
                    }
                }
            }
        }

    }

}