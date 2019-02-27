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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleopKrakens", group="Krakens")
public class TeleopKrakens extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareKrakens   robot           = new HardwareKrakens();              // Use a K9'shardware
    @Override
    public void runOpMode() {
        boolean ReverseDrive =false;
        double LeftSpeed;
        double RightSpeed;
        double cPower =0;
        double udPower;
        boolean SortMinerals=false;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Yahav,Yair and Talia, Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //setting the speed/power of the wheeels acording the y position in the left and right stick
            LeftSpeed = -gamepad1.left_stick_y;
            RightSpeed = -gamepad1.right_stick_y;
            telemetry.addData(">","Robot Current Status:");

            cPower = gamepad2.left_trigger-gamepad2.right_trigger;
            udPower=-gamepad2.right_stick_y;


            if(gamepad2.a){
                SortMinerals=!SortMinerals;
            }
            if(SortMinerals){
                robot.c.setPower(-0.55);
            }
            else{
                robot.c.setPower(cPower);
            }
            if(gamepad2.dpad_up){
                robot.Climb.setPower(-1);

            }
            if(gamepad2.dpad_down){
                robot.Climb.setPower(1);
            }
            if(!gamepad2.dpad_down && !gamepad2.dpad_up){
                robot.Climb.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            }
            if(gamepad1.x){
                robot.IdDropper.setPosition(1);
            }
            if(gamepad1.y){
                robot.IdDropper.setPosition(0);
            }
            if(gamepad1.right_stick_button && gamepad1.left_stick_button){
                ReverseDrive=!ReverseDrive;
            }
            if(gamepad2.dpad_left){
                robot.MineralBlocker.setPosition(robot.IdDropper.getPosition()+1);
            }
            if(gamepad2.dpad_right) {
                robot.MineralBlocker.setPosition(robot.IdDropper.getPosition() - 1);
            }
            if(gamepad1.y){
                robot.MineralBlocker.setPosition(0.8);
            }
            robot.ArmUpDown.setPower(udPower);
            robot.ArmUpDownR.setPower(udPower);
            robot.AllWheelsPower(LeftSpeed,RightSpeed);
            robot.WireController.setPower(gamepad2.left_stick_y);
            /**Printing Robot Status*/

            if(!ReverseDrive){
                robot.MotorLeftFront.setDirection(DcMotor.Direction.FORWARD);
                robot.MotorRightFront.setDirection(DcMotor.Direction.REVERSE);
            }
            if(ReverseDrive){
                robot.MotorLeftFront.setDirection(DcMotor.Direction.REVERSE);
                robot.MotorRightFront.setDirection(DcMotor.Direction.FORWARD);
            }

            if(gamepad2.left_stick_y>0){
                telemetry.addData(">","Arm Going Forward");
            }
            if(gamepad2.left_stick_y<0){
                telemetry.addData(">","Arm Going Backward");
            }

            if(gamepad2.right_stick_y>0){
                telemetry.addData(">","Arm Spinning In");
            }
            if(gamepad2.right_stick_y<0){
                telemetry.addData(">","Arm Spinning Up");
            }
            if(cPower>0){
                telemetry.addData(">","Mineral going Out");
            }
            if(cPower<0){
                telemetry.addData(">","Mineral going In");
            }
            if(gamepad1.dpad_left){
                robot.CloseMineralBlock();
                telemetry.addData("<","Closing Block");
            }
            if(gamepad1.dpad_right){
                robot.OpenMineralBlock();
                telemetry.addData("<","Opening Block");
            }
            if(ReverseDrive){
                telemetry.addData("Drive Direction","NORMAL");
            }
            else{
                telemetry.addData("Drive Direction","REVERSE");
            }
            telemetry.addData("Left Motor Power",String.valueOf(robot.MotorLeftFront.getPower()));
            telemetry.addData("Right Motor Power",String.valueOf(robot.MotorRightFront.getPower()));

            telemetry.update();
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
            }

    }

}
