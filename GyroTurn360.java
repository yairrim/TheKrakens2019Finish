
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

@Autonomous(name ="Gyro Turn360",group = "Test")
public class GyroTurn360 extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();

    private ElapsedTime activetime = new ElapsedTime();
    static final double HEADING_THRESHOLD = 1;

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
            activetime.reset();
            gyroTurn(.4,90);
            gyroTurn(.4,180);
            gyroTurn(.4,270);
            gyroTurn(.4,360);
            }
            telemetry.update();
        }




    public void gyroTurn(double speed, double angle) {


        double leftSpeed=0;
        double rightSpeed=0;
        // keep looping while we are still active, and not on heading.
        while(opModeIsActive() && !gamepad1.a) {
            telemetry.addData("state", "before loop");
            telemetry.update();
        }
        while (opModeIsActive() && (getGyroAngle()<angle-1 || getGyroAngle()>angle+1)) {
            telemetry.addData("in loop", getGyroAngle());
            if(angle>0){
                leftSpeed=speed;
                rightSpeed=-speed;

            }
            if(angle<0){
                leftSpeed=-speed;
                rightSpeed=speed;

            }
            robot.AllWheelsPower(leftSpeed,rightSpeed);
            telemetry.update();
        }
        robot.AllWheelsPower(0,0);

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

}
