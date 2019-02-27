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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

public class HardwareKrakens
{
    /* Public OpMode members. */
    public DcMotor MotorLeftFront  = null;
    public DcMotor MotorRightFront = null;
    public DcMotor ArmUpDown       = null;
    public DcMotor c               = null;
    public DcMotor ArmUpDownR      = null;
    public DcMotor Climb           = null;
    public Servo IdDropper         = null;
    public Servo MineralBlocker   = null;
    public DcMotor WireController  = null;
    private boolean stepFound;      // Sound file present flags
    public double IdStartpos;
    BNO055IMU imu;
    double TickClimb=-17100;

    Orientation angles;
    Acceleration gravity;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    /* Constructor */
    public HardwareKrakens() {}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        int stepSoundID = hwMap.appContext.getResources().getIdentifier("step", "raw", hwMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (stepSoundID != 0)
            stepFound   = SoundPlayer.getInstance().preload(hwMap.appContext, stepSoundID);

        // Display sound status
        // Define and Initialize Motors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //setting the parameters for the imu(gyro)
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        MotorLeftFront  = hwMap.get(DcMotor.class, "mlf");//port 3 rev 2
        MotorRightFront = hwMap.get(DcMotor.class, "mrf");//port 2 rev2
        ArmUpDown       =hwMap.get(DcMotor.class,"aud");//port 3 rev 1
        ArmUpDownR     = hwMap.get(DcMotor.class, "aud2");//port 0 rev 1
        c =hwMap.get(DcMotor.class,"s");//port o rev 2
        Climb = hwMap.get(DcMotor.class,"cl");//port 1 rev 1
        WireController = hwMap.get(DcMotor.class,"wc");//port 1 rev 1
        imu = hwMap.get(BNO055IMU.class, "imu");//gyro/imu in rev
        imu.initialize(parameters);

        MotorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        MotorRightFront.setDirection(DcMotor.Direction.REVERSE);
        ArmUpDown.setDirection(DcMotor.Direction.REVERSE);
        ArmUpDownR.setDirection(DcMotor.Direction.FORWARD);
        WireController.setDirection(DcMotor.Direction.FORWARD);
        Climb.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        AllWheelsPower(0,0);
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmUpDownR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WireController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IdDropper = hwMap.get(Servo.class,"id");
        MineralBlocker = hwMap.get(Servo.class,"mb");//rev 1 port 5
        IdStartpos = IdDropper.getPosition();



    }
    public void AllWheelsPower(double LeftMotorPower,double RightMotorPower){
        MotorLeftFront.setPower(LeftMotorPower);
        MotorRightFront.setPower(RightMotorPower);
    }

    public void OpenMineralBlock(){
        MineralBlocker.setPosition(MineralBlocker.getPosition()+1);
    }
    public void CloseMineralBlock(){
        MineralBlocker.setPosition(MineralBlocker.getPosition()-1);
    }

    public float getGyroAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void ResetId(){
        IdDropper.setPosition(IdStartpos);
    }
    public void initIMU(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //setting the parameters for the imu(gyro)
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
    public boolean MotorInPosition(DcMotor Motor){
        if (abs(Motor.getCurrentPosition()) < abs(Motor.getTargetPosition())) {
                return false;
        }
            else{
                return true;
            }

     }

}


