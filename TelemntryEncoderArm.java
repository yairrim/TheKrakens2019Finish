/* Copyright (c) 2017 FIRST. All rights reserved.
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

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Telemntry Encoder Arm", group="Test")
public class TelemntryEncoderArm extends LinearOpMode {

    HardwareKrakens robot = new HardwareKrakens();
    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
          //
        telemetry.update();
        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDownR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Say", "Hello Kraken get ready");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("correct position", robot.ArmUpDownR.getCurrentPosition());
            telemetry.update();
            if(gamepad1.a){
                telemetry.addData(">","A id Pressed");
                telemetry.update();

                UpDownArmEncoder(0.7,UdDir.Up);
            }

            if(gamepad1.b){

                telemetry.addData(">","B id Pressed");
                telemetry.update();
                UpDownArmEncoder(0.7,UdDir.Down);
            }
        }
    }
    enum  UdDir{
        Up,
        Release,
        Down;
    }
    public void UpDownArmEncoder(double speed,UdDir Dir) {

        int newTarget=0;

        // Ensure that the opmode is still active



        robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDownR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (Dir == UdDir.Up){
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() + 1550);
        }
        if (Dir == UdDir.Down){
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() - 1450);
            speed=-speed;
        }

        if(Dir == UdDir.Release) {
            newTarget = (int) (robot.ArmUpDownR.getCurrentPosition() + 600);
        }
        robot.ArmUpDownR.setTargetPosition(newTarget);
        if (Dir == UdDir.Up){
            while(opModeIsActive() && !robot.MotorInPosition(robot.ArmUpDownR)){
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.WireController.setPower(-0.3);
                robot.ArmUpDown.setPower(speed);
                robot.ArmUpDownR.setPower(speed);
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("CurrentPos", robot.ArmUpDown.getCurrentPosition());
                telemetry.addData("Target", robot.ArmUpDown.getTargetPosition());
                telemetry.update();
            }
        }
        if (Dir == UdDir.Down){
            while(opModeIsActive() && !robot.MotorInPosition(robot.ArmUpDownR)){
                robot.ArmUpDownR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.WireController.setPower(-0.3);
                robot.ArmUpDown.setPower(speed);
                robot.ArmUpDownR.setPower(speed);
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



}
