
package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Function.Position.LEFT;
import static org.firstinspires.ftc.teamcode.Function.Position.MIDDLE;
import static org.firstinspires.ftc.teamcode.Function.Position.RIGHT;
import static org.firstinspires.ftc.teamcode.Function.UPdDir.Down;
import static org.firstinspires.ftc.teamcode.Function.UPdDir.Release;
import static org.firstinspires.ftc.teamcode.Function.UPdDir.SlightlyDown;
import static org.firstinspires.ftc.teamcode.Function.UPdDir.Up;


@Autonomous(name ="Function",group="Auto")
@Disabled
public class Function extends LinearOpMode{
    HardwareKrakens robot = new HardwareKrakens();
    private ExampleGoldVision GoldVision;
    private boolean rtkFound;      // Sound file present flags

    private boolean playRtk = true;

    private ElapsedTime activetime = new ElapsedTime();
    private ElapsedTime OpModetime = new ElapsedTime();
    private ElapsedTime InTime = new ElapsedTime();
    private ElapsedTime idtime = new ElapsedTime();
    public Position GoldMineralPosition;
    static final double HEADING_THRESHOLD = 1;
    public double CountPerCm=1120/(3.14159265359*11);


    @Override
    public void runOpMode() {
        GoldVision = new ExampleGoldVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        GoldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        GoldVision.setShowCountours(false);

        // start the vision system
        GoldVision.enable();

        int rtkSoundID   = hardwareMap.appContext.getResources().getIdentifier("rtk",   "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (rtkSoundID != 0)
            rtkFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, rtkSoundID);


        // Display sound status
        telemetry.addData("rtk resource",   rtkFound ?   "Found" : "NOT found\n Add rtk.wav to /teamcode/src/main/res/raw" );

        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        /** Wait for the game to begin */
        while (!isStarted()){
            telemetry.addData("angle",getGyroAngle());
            telemetry.update();
        }
        if (opModeIsActive()) {
            activetime.reset();
            if (opModeIsActive()) {
                if(playRtk){
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, rtkSoundID);
                    telemetry.addData("Playing", "Resource rtk");
                    telemetry.update();
                    playRtk=false;
                }
                GoldMineralPosition=GoldRun();
                OutOfLander();
                switch (GoldMineralPosition) {

                    case MIDDLE:
                        posC();
                    case LEFT:
                        posL();
                    case RIGHT:
                        posR();


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
        distance = -distance;

        double leftSpeed = 0;
        double rightSpeed = 0;
        distance = (int) (distance * CountPerCm);
        double max;
        double error;
        double steer=0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int startPosLeft = robot.MotorLeftFront.getCurrentPosition();
            newLeftTarget = (robot.MotorLeftFront.getCurrentPosition() + (distance));
            newRightTarget = ((robot.MotorRightFront.getCurrentPosition() + distance));

            // Set Target and Turn On RUN_TO_POSITION
            robot.MotorLeftFront.setTargetPosition(newLeftTarget);
            robot.MotorRightFront.setTargetPosition(newRightTarget);
            rightSpeed = speed;
            leftSpeed = speed;
            robot.AllWheelsPower(leftSpeed, rightSpeed);
            // keep looping while we are still active, and BOTH motors are running.
            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (distance < 0)
                steer *= -1.0;
            while (opModeIsActive() && !robot.MotorInPosition(robot.MotorLeftFront) && !robot.MotorInPosition(robot.MotorRightFront)) {
                error = getError(angle);
                steer = getSteer(error, 0.15);

                // if driving in reverse, the motor correction also needs to be reversed


                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                robot.AllWheelsPower(-leftSpeed, -rightSpeed);
                telemetry.addData("gyroAngel", getGyroAngle());
                telemetry.addData("start position left ", startPosLeft);
                telemetry.addData("speed left", leftSpeed);
                telemetry.addData("speed right", rightSpeed);
                telemetry.addData("Target left", robot.MotorLeftFront.getTargetPosition());
                telemetry.addData("correct position", -robot.MotorLeftFront.getCurrentPosition());
                telemetry.addData("Power left", robot.MotorLeftFront.getPower());

                telemetry.update();

            }

            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.AllWheelsPower(0, 0);
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
        while (opModeIsActive() && !onHeading(speed, angle, 0.2)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }





    public void posR() {

        gyroDrive(0.2,-10,0);
        gyroTurn(0.5, -20);
        CubeToLander();
        gyroTurn(0.5, -20);
        gyroDrive(1,-80,-27);
        gyroTurn(0.5, 27);
        gyroDrive(1, -55, 27);
        idtime.reset();
        DropId();
    }
    public void posL() {

        gyroDrive(0.2,-10,0);
        gyroTurn(0.5, 24);
        CubeToLander();
        gyroTurn(0.5, 30);
        gyroDrive(1,-80,27);
        gyroTurn(0.5, -22);
        gyroDrive(1, -50, -27);
        idtime.reset();
        DropId();
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
        gyroDrive(1,-20,0);
        gyroTurn(0.7,5);
        CubeToLander();
        gyroTurn(0.7,5);
        gyroDrive(1, -135 , 0);
        ToCrater();
        while(opModeIsActive()){

        }
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    enum Position{
        RIGHT,
        LEFT,
        MIDDLE
    }

    public Position  GoldRun(){

        int X;
        int Mid = 220;
        List<MatOfPoint> contours = GoldVision.getContours();
        if (contours.isEmpty()) {
            return RIGHT;
        }
        else {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e

            Rect boundingRect = Imgproc.boundingRect(contours.get(0));
            X = boundingRect.x;
            if(X>Mid){
                return LEFT;
            }
            if(X<Mid){
                return MIDDLE;
            }
        }
        return null;
    }

    public void ToCrater(){
        gyroDrive(0.7,15,0);
        idtime.reset();
        DropId();
        gyroTurn(0.7,75);
        gyroDrive(0.7,40,75);
        gyroTurn(0.7,50);
        gyroDrive(1,60,50);
        gyroTurn(0.7,50);
        gyroDrive(1,60,50);
        gyroTurn(0.7,50);
        gyroDrive(1,60,50);
    }



    public void DropId() {
        robot.IdDropper.setPosition(1);
        idtime.reset();
        while(opModeIsActive() &&idtime.seconds()<0.5){
            telemetry.addData(String.valueOf(idtime.seconds()), "Dropping");
            telemetry.update();
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
    public void CubeToLander(){
        WireControllEncoder(1,90);
        MineralIn();
        WireControllEncoder(1,-80);
        if(GoldMineralPosition!=MIDDLE) {
            gyroTurn(0.55, 0);
        }
        gyroDrive(0.7,29,5);
        while(robot.Climb.getPower()!=0){
        }
        activetime.reset();
        UpDownArmEncoder(1,Up);
        SortMineral();
        UpDownArmEncoder(1,SlightlyDown);

        robot.ArmUpDownR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmUpDown.setPower(robot.ArmUpDownR.getPower());
        gyroDrive(0.4,-4,0);
    }
    enum UPdDir{
        Up,
        Down,
        Release,
        SlightlyDown;
    }
    public void UpDownArmEncoder(double speed, UPdDir Dir) {

        int newTarget=0;

        // Ensure that the opmode is still active



        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDownR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (opModeIsActive() && Dir == Up){
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() + 1400);
        }
        if (opModeIsActive() && Dir == Down){
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() - 1350);
            speed=-speed;
        }

        if (opModeIsActive() && Dir == SlightlyDown){
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() - 300);
            speed=-speed;
        }

        if(opModeIsActive() && Dir == UPdDir.Release) {
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() + 600);
        }
        robot.ArmUpDownR.setTargetPosition(newTarget);
        if (opModeIsActive() && Dir == Up || Dir == UPdDir.Release){
            while(opModeIsActive() && !robot.MotorInPosition(robot.ArmUpDownR)){
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.ArmUpDown.setPower(speed);
                robot.ArmUpDownR.setPower(speed);
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("CurrentPos", robot.ArmUpDown.getCurrentPosition());
                telemetry.addData("Target", robot.ArmUpDown.getTargetPosition());
                telemetry.update();
            }
        }
        if (opModeIsActive() && Dir == Down || Dir == SlightlyDown){
            while(opModeIsActive() && !robot.MotorInPosition(robot.ArmUpDownR)){
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.ArmUpDown.setPower(speed);
                robot.ArmUpDownR.setPower(speed);
                robot.WireController.setPower(-0.1);
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("CurrentPos", robot.ArmUpDown.getCurrentPosition());
                telemetry.addData("Target", robot.ArmUpDown.getTargetPosition());
                telemetry.update();
            }
        }
        // Set Target and Turn On RUN_TO_POSITION
        // keep looping while we are still active, and BOTH motors are running.

        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ArmUpDown.setPower(0);
        robot.ArmUpDownR.setPower(0);
        robot.WireController.setPower(0);
    }
    public void WireControllEncoder(double speed,
                                           int distance) {

        int newTarget;
        distance = distance * 25;

        // Ensure that the opmode is still active

        robot.WireController.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.WireController.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = (int) (robot.WireController.getCurrentPosition() + (distance/0.926));

        // Set Target and Turn On RUN_TO_POSITION
        robot.WireController.setTargetPosition(newTarget);
        // keep looping while we are still active, and BOTH motors are running.
        robot.WireController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && !robot.MotorInPosition(robot.WireController)){

            robot.WireController.setPower(speed);

            telemetry.addData("CurrentPos", robot.WireController.getCurrentPosition());
            telemetry.addData("Target", robot.WireController.getTargetPosition());
            telemetry.update();
        }
        robot.WireController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.WireController.setPower(0);
    }

    public void MineralIn(){
        InTime.reset();
        while(opModeIsActive() && InTime.seconds()<0.35){
        }
        while(opModeIsActive() && InTime.seconds()<0.8){
            robot.c.setPower(-1);
        }
        robot.c.setPower(0);
    }

    public void SortMineral(){
        InTime.reset();
        while(opModeIsActive() && InTime.seconds()<0.35){
        }
        while(opModeIsActive() && InTime.seconds()<1.5){
            robot.c.setPower(-0.3);
        }
        robot.c.setPower(0);
    }

    public void OutOfLander(){
        boolean DropdArm=false;

        activetime.reset();
        robot.Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive() && robot.Climb.getCurrentPosition()>robot.TickClimb){
            robot.Climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Climb.setPower(-1);
            robot.Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(!DropdArm){
                ReleseArm();
                DropdArm=true;
            }
            telemetry.addData("Position",robot.Climb.getCurrentPosition());
            telemetry.addData("Gold Mineral Position = ",GoldMineralPosition);
            telemetry.update();
        }
        robot.Climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Climb.setPower(0);
    }

    public void ReleseArm(){
        UpDownArmEncoder(1,Release);
    }
}

