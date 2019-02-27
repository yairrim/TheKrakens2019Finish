
package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name ="Autonomous krakens 15768 Dimona Doll")
@Disabled
public class AutonomousKrakensDimonaDoll extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();
    private ExampleGoldVision blueVision;
    private boolean rtkFound;      // Sound file present flags

    private boolean playRtk = true;

    private ElapsedTime activetime = new ElapsedTime();
    private ElapsedTime idtime = new ElapsedTime();
    public String GoldMineralPosition="Unknown";
    static final double HEADING_THRESHOLD = 1;
    static final double P_TURN_COEFF = 0.1;

    @Override
    public void runOpMode() {
        blueVision = new ExampleGoldVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowCountours(false);

        // start the vision system
        blueVision.enable();

        int rtkSoundID   = hardwareMap.appContext.getResources().getIdentifier("rtk",   "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (rtkSoundID != 0)
            rtkFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, rtkSoundID);


        // Display sound status
        telemetry.addData("rtk resource",   rtkFound ?   "Found" : "NOT found\n Add rtk.wav to /teamcode/src/main/res/raw" );

        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** Wait for the game to begin */
        while (!isStarted()){
            telemetry.addData("angle",getGyroAngle());
            telemetry.update();
        }
        if (opModeIsActive()) {
            activetime.reset();
            while (opModeIsActive()) {
                if(playRtk){
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, rtkSoundID);
                    telemetry.addData("Playing", "Resource rtk");
                    telemetry.update();
                    playRtk=false;
                }
                GoldRun();
                if(opModeIsActive() && GoldMineralPosition=="C") {
                    while(opModeIsActive() && activetime.seconds()<4){
                        telemetry.addData("time remaining",activetime.seconds());
                        telemetry.addData("finish","GoldRun      C");
                        telemetry.update();
                    }
                    while(opModeIsActive() && activetime.seconds()<6){
                        posC();
                    }
                }
                if(opModeIsActive() && GoldMineralPosition=="L") {
                    while(opModeIsActive() && activetime.seconds()<4){
                        telemetry.addData("time remaining",activetime.seconds());
                        telemetry.addData("finish","GoldRun     L");
                        telemetry.update();
                    }

                    while(opModeIsActive() && activetime.seconds()<6){
                        posL();
                    }
                }
                if(opModeIsActive() && GoldMineralPosition=="R") {
                    while(opModeIsActive() && activetime.seconds()<4){
                        telemetry.addData("time remaining",activetime.seconds());
                        telemetry.addData("finish","GoldRun R");
                        telemetry.update();
                    }
                    while(opModeIsActive() && activetime.seconds()<6){
                        posR();
                    }

                }
                robot.ResetId();

            }
            telemetry.update();
        }

    }


    public void gyroDrive(double speed,
                          int distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        distance=-distance;

        double leftSpeed = 0;
        double rightSpeed= 0;
        distance = distance * 25;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int startPosLeft=robot.MotorLeftFront.getCurrentPosition();
            newLeftTarget = (int) (robot.MotorLeftFront.getCurrentPosition() + (distance/0.926));
            newRightTarget = (int) (robot.MotorRightFront.getCurrentPosition() + (distance/0.926));

            // Set Target and Turn On RUN_TO_POSITION
            robot.MotorLeftFront.setTargetPosition(newLeftTarget);
            robot.MotorRightFront.setTargetPosition(newRightTarget);
            rightSpeed=speed;
            leftSpeed=speed;
            robot.AllWheelsPower(leftSpeed,rightSpeed);
            // keep looping while we are still active, and BOTH motors are running.
            if(robot.MotorRightFront.getCurrentPosition()<robot.MotorRightFront.getTargetPosition() &&
                    robot.MotorLeftFront.getCurrentPosition()<robot.MotorLeftFront.getTargetPosition()) {
                while (opModeIsActive() &&
                        (-robot.MotorRightFront.getCurrentPosition() < robot.MotorRightFront.getTargetPosition() &&
                                -robot.MotorLeftFront.getCurrentPosition() < robot.MotorLeftFront.getTargetPosition() )) {

                    if (getGyroAngle() < angle - 1) {
                        leftSpeed = leftSpeed + 0.1;
                        telemetry.addData("direction", "left");

                    }
                    if (getGyroAngle() > angle + 1) {
                        rightSpeed = rightSpeed + 0.1;
                        telemetry.addData("direction", "right");
                    }
                    if (getGyroAngle() > angle - 1 && getGyroAngle() < angle + 1) {
                        leftSpeed = speed;
                        rightSpeed = speed;
                        telemetry.addData("direction", "forward");

                    }
                    robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.AllWheelsPower(-leftSpeed, -rightSpeed);
                    robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("gyroAngel", getGyroAngle());
                    telemetry.addData("start position left ", startPosLeft);
                    telemetry.addData("speed left", leftSpeed);
                    telemetry.addData("speed right", rightSpeed);
                    telemetry.addData("Target left", robot.MotorLeftFront.getTargetPosition());
                    telemetry.addData("correct position", -robot.MotorLeftFront.getCurrentPosition());
                    telemetry.addData("Power left", robot.MotorLeftFront.getPower());

                    telemetry.update();

                    rightSpeed=speed;
                    leftSpeed=speed;


                }
            }
            if(robot.MotorRightFront.getCurrentPosition()>robot.MotorRightFront.getTargetPosition() &&
                    robot.MotorLeftFront.getCurrentPosition()>robot.MotorLeftFront.getTargetPosition()) {
                while (opModeIsActive() &&
                        (-robot.MotorRightFront.getCurrentPosition() > robot.MotorRightFront.getTargetPosition() &&
                                -robot.MotorLeftFront.getCurrentPosition() > robot.MotorLeftFront.getTargetPosition())) {


                    // adjust relative speed based on heading error.
                    if (getGyroAngle() < angle - 1) {
                        leftSpeed = leftSpeed + 0.1;
                        telemetry.addData("direction", "left");

                    }
                    if (getGyroAngle() > angle + 1) {
                        rightSpeed = rightSpeed + 0.1;
                        telemetry.addData("direction", "right");
                    }
                    if (getGyroAngle() > angle - 1 && getGyroAngle() < angle + 1) {
                        leftSpeed = speed;
                        rightSpeed = speed;
                        telemetry.addData("direction", "forward");

                    }
                    robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.AllWheelsPower(leftSpeed, rightSpeed);
                    robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("gyroAngel", getGyroAngle());
                    telemetry.addData("start position left ", startPosLeft);
                    telemetry.addData("speed left", leftSpeed);
                    telemetry.addData("speed right", rightSpeed);
                    telemetry.addData("Target left", robot.MotorLeftFront.getTargetPosition());
                    telemetry.addData("correct position", -robot.MotorLeftFront.getCurrentPosition());

                    telemetry.update();

                }
            }
        }

        robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.AllWheelsPower(0, 0);
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


        double leftSpeed=0;
        double rightSpeed=0;
        // keep looping while we are still active, and not on heading.

        while (opModeIsActive() && (getGyroAngle()<angle-5 || getGyroAngle()>angle+5)) {
            telemetry.addData("in loop", getGyroAngle());
            telemetry.update();
            if(angle>0){
                leftSpeed=-speed;
                rightSpeed=speed;

            }
            if(angle<0){
                leftSpeed=speed;
                rightSpeed=-speed;

            }
            robot.AllWheelsPower(leftSpeed,rightSpeed);
            telemetry.update();
        }
        robot.AllWheelsPower(0,0);

    }



    public void posR() {
        gyroTurn(0.4, -23);
        gyroDrive(0.7, -80, 30);
        gyroTurn(0.4, 20);
        gyroDrive(0.7,-90,20);
        idtime.reset();
        DropId();
     /*   gyroTurn(0.4, -54);
        gyroDrive(0.7,100,-54);
        gyroTurn(0.4, -54);
        gyroDrive(0.3,60,-54);
        gyroTurn(0.4, -54);
        gyroDrive(0.3,80,-54);
   */
    }
    public void posL() {
        gyroTurn(0.4, 23);
        gyroDrive(0.7, -70, -30);
        gyroTurn(0.4, -35);
        gyroDrive(0.7,-90,20);

        idtime.reset();
        DropId();
        /*
        gyroTurn(0.4, -54);
        gyroDrive(0.7,100,-54);
        gyroTurn(0.4, -54);
        gyroDrive(0.3,60,-54);
        gyroTurn(0.4, -64);
        gyroDrive(0.3,70,-54);
    */
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


    public void posC() {
        gyroDrive(0.7, -150, 0);
        idtime.reset();
        DropId();
/*
        gyroDrive(0.7,20,0);
        gyroTurn(0.4, -44);
        gyroDrive(0.7,100,-44);
        gyroTurn(0.4, -44);
        gyroDrive(0.3,60,-44);
        gyroTurn(0.4, -44);
        gyroDrive(0.3,100,-44);
*/
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void  GoldRun(){
        int Y;
        int X;
        int Mid = 300;
        List<MatOfPoint> contours = blueVision.getContours();
        if (contours.isEmpty()) {
            telemetry.addData(">", "right");
            GoldMineralPosition="R";
        } else {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e

            Rect boundingRect = Imgproc.boundingRect(contours.get(0));

            Y = boundingRect.y;
            X = boundingRect.x;
            telemetry.addData(">", "X-" + X + ",Y-" + Y);
            if (Y < Mid) {
                telemetry.addData(">", "Left");
                GoldMineralPosition="L";
            } else if (Y > Mid) {
                telemetry.addData(">", "center");
                GoldMineralPosition="C";
            }
        }

    }



    public void DropId() {
        while(idtime.seconds()<3 && opModeIsActive()){
            robot.ArmUpDown.setPower(0.4);
            robot.ArmUpDownR.setPower(0.4);
            telemntryDropId();
        }
    }
    public void telemntryDropId(){
        if(idtime.seconds()<2) {
            telemetry.addData(String.valueOf(idtime.seconds()), "Dropping");
        }

        if(idtime.seconds()>2) {
            telemetry.addData(String.valueOf(idtime.seconds()), "Resenting");
        }
        telemetry.update();
    }
}
