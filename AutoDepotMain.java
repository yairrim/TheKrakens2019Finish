
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name ="AoutoSepot Main")
@Disabled
public class AutoDepotMain extends LinearOpMode {
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


        }
    }