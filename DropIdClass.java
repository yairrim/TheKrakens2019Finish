
package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.SoundPlayer;
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

@Autonomous(name ="DropIdClass")
public class DropIdClass extends LinearOpMode {
    HardwareKrakens robot = new HardwareKrakens();
    ElapsedTime idtime= new ElapsedTime();
    @Override
    public void runOpMode() {


        // Determine Resource IDs for sounds built into the RC application.

        robot.init(hardwareMap);
        robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** Wait for the game to begin */
        while (!isStarted()) {
            telemetry.addData("angle", "0");
            telemetry.update();
        }
        if (opModeIsActive()) {
            if (opModeIsActive()) {
                idtime.reset();
                DropId();
            }


        }
    }
    public void DropId() {

        while(idtime.seconds()<1.5 && opModeIsActive()){
            robot.ArmUpDown.setPower(0.5);
            robot.ArmUpDownR.setPower(0.5);
            telemntryDropId();
        }
        while(idtime.seconds()<2 && opModeIsActive()){
            robot.ArmUpDown.setPower(0.3);
            robot.ArmUpDownR.setPower(0.3);
            telemntryDropId();
        }
        while(idtime.seconds()<3.5 && opModeIsActive()){
            robot.ArmUpDown.setPower(-0.5);
            robot.ArmUpDownR.setPower(-0.5);
            telemntryDropId();
        }
        while(idtime.seconds()<4 && opModeIsActive()){
            robot.ArmUpDown.setPower(-0.3);
            robot.ArmUpDownR.setPower(-0.3);
            telemntryDropId();
        }
        robot.ArmUpDown.setPower(0);
        robot.ArmUpDownR.setPower(0);

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