
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name ="Gyro Turn180",group = "Test")
public class GyroTurn90 extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();

    private ElapsedTime activetime = new ElapsedTime();
    static final double HEADING_THRESHOLD = 1;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        /** Wait for the game to begin */
        while (!isStarted()){
            telemetry.addData("angle",getGyroAngle());
            telemetry.update();
        }
        if (opModeIsActive()) {
            activetime.reset();
            gyroTurn(.4,180);
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
                telemetry.addData("left", robot.MotorLeftFront.getPower());
                telemetry.addData("right", robot.MotorRightFront.getPower());
                leftSpeed=-speed;
                rightSpeed=speed;

            }
            if(angle<0){
                telemetry.addData("left", robot.MotorLeftFront.getPower());
                telemetry.addData("right", robot.MotorRightFront.getPower());
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

}
