
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name ="Pos L/R",group = "Test")
public class PosL extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();

    private ElapsedTime activetime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    public double ArmSpeed = 0.4;
    static final double HEADING_THRESHOLD = 1;
    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;

    @Override
    public void runOpMode() {
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
            while(opModeIsActive()){
                telemetry.addData("X for Left","Y for Right");
                telemetry.update();
                posL();


            }

        }

    }



    public void gyroDrive(double speed,
                          int distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        distance=-distance;
        double max;
        double error;
        double steer;
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
        while(opModeIsActive() && !gamepad1.a) {
            telemetry.addData("state", "before loop");

        }
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



    public float getGyroAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void posR() {
        gyroTurn(0.4, -20);
        gyroDrive(0.7, -80, 30);

    }
    public void posL() {
        gyroTurn(0.4, 20);
        gyroDrive(0.7, -80, -30);
    }

}
