
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

import static java.lang.Math.abs;

@Autonomous(name ="Gyro drive 100cm and -100cm",group = "Test")
public class gyrodrive1 extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();

    private ElapsedTime activetime = new ElapsedTime();
    static final double P_DRIVE_COEFF = 0.15;
    public double CountPerCm = 1120 / (3.14159265359 * 11);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /** Wait for the game to begin */
        while (!isStarted()){
            telemetry.addData("angle",getGyroAngle());
            telemetry.update();
        }
        if (opModeIsActive()) {
            activetime.reset();
            gyroDrive(.4,100,0);
            gyroDrive(.4,-100,0);

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
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                robot.AllWheelsPower(-leftSpeed, -rightSpeed);
                telemetry.addData("gyroAngel", getGyroAngle());
                telemetry.addData("start position left ", startPosLeft);
                telemetry.addData("speed left", leftSpeed);
                telemetry.addData("speed right", rightSpeed);
                telemetry.addData("Target left", abs(robot.MotorLeftFront.getTargetPosition()));
                telemetry.addData("correct position", abs(robot.MotorLeftFront.getCurrentPosition()));
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


}
