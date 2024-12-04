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


package org.firstinspires.ftc.teamcode.Teleop;

import org.firstinspires.ftc.teamcode.opencv.TeamElementDetector;
import org.firstinspires.ftc.teamcode.opencv.CamEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



@Autonomous(name="Basic: Linear OpMode", group="Linear OpMode")
public class Auton extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;
    DcMotor biggerSucc = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        biggerSucc = hardwareMap.get(DcMotor.class, "biggerSucc");



        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);


        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            static void moveForward(int speed, int time);
            static void moveBackward(int speed, int time);
            static void moveLeft(int speed, int time);
            static void moveRight(int speed, int time);

            //ADD AUTON CODE HERE

            //Function definitions
           static void moveForward(int speed, int time)
            {
                motorBL.setPower(speed);
                motorBR.setPower(speed);
                motorFL.setPower(speed);
                motorFR.setPower(speed);

                sleep(time);

                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            static void moveBackward(int speed, int time)
            {
                motorBL.setPower(-speed);
                motorBR.setPower(-speed);
                motorFL.setPower(-speed);
                motorFR.setPower(-speed);

                sleep(time);

                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            static void moveLeft(int speed, int time)
            {
                motorBL.setPower(-speed);
                motorBR.setPower(speed);
                motorFL.setPower(speed);
                motorFR.setPower(-speed);

                sleep(time);

                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            static void moveRight(int speed, int time)
            {
                motorBL.setPower(speed);
                motorBR.setPower(-speed);
                motorFL.setPower(-speed);
                motorFR.setPower(speed);

                sleep(time);

                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }
        }
    }
}
