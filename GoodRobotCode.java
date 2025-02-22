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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//192.168.49.1:8080/dash
@Config
@TeleOp(name="GoodRobotCode", group="Linear OpMode")
//@Disabled
public class GoodRobotCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private DcMotor tallStickL = null;
    //Right linear slide
    private DcMotor tallStickR = null;
    //Left linear slide
    private DcMotor LongSlide = null;


    private Servo knees = null;
    private Servo ankleL = null;
    private Servo ankleR = null;
    private Servo foot = null;
    private Servo grippers = null;
    private Servo Arm = null;
    //Arm servos on linear slide port 0
    private Servo Elbow = null;
    //Elbow servos on arm port 1
    private Servo Wrist = null;
    //Wrist servo on elbow port 2
    private Servo Grip = null;
    //Block Gripper port 3
    static int[] slideMode = {2, 0};
    //[0] stores which case is active; [1] checks if the operation has been completed

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        tallStickL = hardwareMap.get(DcMotor.class, "tallStickL");
        tallStickR = hardwareMap.get(DcMotor.class, "tallStickR");
        LongSlide = hardwareMap.get(DcMotor.class,"longSlide");

        knees = hardwareMap.get(Servo.class,"Knees");
        foot = hardwareMap.get(Servo.class,"Foot");
        ankleL = hardwareMap.get(Servo.class,"AnkleL");
        ankleR = hardwareMap.get(Servo.class,"AnkleR");
        grippers = hardwareMap.get(Servo.class,"Grippers");

        Grip = hardwareMap.get(Servo.class,"grip");
        Arm = hardwareMap.get(Servo.class,"arm");
        Elbow = hardwareMap.get(Servo.class,"elbow");
        Wrist = hardwareMap.get(Servo.class,"wrist");

        //controller = new PIDController(p, i, d);

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        LongSlide.setDirection((DcMotorSimple.Direction.REVERSE));
        tallStickL.setDirection(DcMotorSimple.Direction.REVERSE);

        tallStickL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tallStickR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tallStickL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tallStickR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Grip.setPosition(0);
        Wrist.setPosition(0);
        knees.setPosition(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double x = 0.1;
        double sped = .5;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double SlideUpL = tallStickL.getCurrentPosition();
            double SlideUpR = tallStickR.getCurrentPosition();

            telemetry.addData("SlidesPos", "(%.2f), (%.2f)",
                    SlideUpR,
                    SlideUpL);
            telemetry.update();

            double speed = 1;

            //Gamepad1 Code--------------------------------------------------------------------------------------------------------------------

            //Driving Code
            motorFR.setPower((-gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) + (gamepad1.right_stick_x * speed) - gamepad1.left_trigger + gamepad1.right_trigger);
            motorBR.setPower((-gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) + (gamepad1.right_stick_x * speed) + gamepad1.left_trigger - gamepad1.right_trigger);
            motorFL.setPower((-gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) - (gamepad1.right_stick_x * speed) + gamepad1.left_trigger - gamepad1.right_trigger);
            motorBL.setPower((-gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) - (gamepad1.right_stick_x * speed) - gamepad1.left_trigger + gamepad1.right_trigger);

            //Long slide code
            if (gamepad1.dpad_up) {
                LongSlide.setPower(.5);
            } else if (gamepad1.dpad_down) {
                LongSlide.setPower(-.5);
            } else {
                LongSlide.setPower(0);
            }

            if(gamepad1.dpad_left) {
                ankleL.setPosition(.1);
            } else if (gamepad1.dpad_right) {
                ankleL.setPosition(.4);
            }

            if (gamepad1.left_bumper)
            {
                grippers.setPosition(0);
            }
            else if (gamepad1.right_bumper)
            {
                grippers.setPosition(.5);
            }

            if (gamepad1.y) {
                knees.setPosition(.5);
                ankleL.setPosition(.4);
                foot.setPosition(0);
            } else if (gamepad1.x) {
                knees.setPosition(.1);
                ankleL.setPosition(.4);
                foot.setPosition(0);
            } else if (gamepad1.b) {
                grippers.setPosition(.5);
                sleep(200);
                ankleL.setPosition(0);
                foot.setPosition(.7);
            }

            //Gamepad2 Code--------------------------------------------------------------------------------------------------------------------

            //Linear Slide Code
            if (gamepad2.y) {
                tallStickR.setTargetPosition(400);
                tallStickR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tallStickR.setPower(1);
                tallStickL.setTargetPosition(400);
                tallStickL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tallStickL.setPower(1);
            } else if (gamepad2.left_stick_y < 0) {
                tallStickL.setPower((-1)* sped);
                tallStickR.setPower((1)* sped);
            } else {
                tallStickL.setPower(0);
                tallStickR.setPower(0);
            }

            //Block Grip Code
            if (gamepad2.right_bumper) {
                Grip.setPosition(0);
            } else if (gamepad2.left_bumper) {
                Grip.setPosition(.30);
            }

            //Elbow Code
            if (gamepad2.dpad_up) {
                Elbow.setPosition(.45);
            } else if (gamepad2.dpad_down) {
                Elbow.setPosition(.80);
            }

            //Wrist Code
            if (gamepad2.dpad_left) {
                Wrist.setPosition(0);
            } else if (gamepad2.dpad_right) {
                Wrist.setPosition(.35);
            }

            //Gamepad2 Arm Controls
            if (gamepad2.a) {
                //Grab from intake position
                sleep(300);
                Elbow.setPosition(.70);
                Arm.setPosition(.8);
                Wrist.setPosition(.35);
                Grip.setPosition(.30);
            } else if (gamepad2.b) {
                //Wall/Score position
                Arm.setPosition(.30);
                sleep(300);
                Elbow.setPosition(.80);
                Wrist.setPosition(.35);
            } else if (gamepad2.x) {
                //Hold position
                Arm.setPosition(.50);
                sleep(300);
                Elbow.setPosition(.5);

            /*if (slideMode[1] == 1)
            {

                if (gamepad2.a)
                {
                    //grab from intake position
                    slideMode[0] = 1;
                    slideMode[1] = 0;
                }
                else if (gamepad2.b)
                {
                    //wall position
                    slideMode[0] = 2;
                    slideMode[1] = 0;
                }
                else if (gamepad2.x)
                {
                    //drop position
                    slideMode[0] = 0;
                    slideMode[1] = 0;
                }

            }


            if (slideMode[1] == 0) {

                switch (slideMode[0]) {
                    case 1:

                        LongSlide.setPosition(1);
                        Flapper.setPosition(0.10);
                        Elbow.setPosition(.75);
                        Wrist.setPosition(0);
                        Arm.setPosition(.97);
                        Grip.setPosition(0);

                        slideMode[1] = 1;
                        break;

                    case 2:

                        LongSlide.setPosition(.95);
                        Flapper.setPosition(0.10);
                        Arm.setPosition(.20);
                        Elbow.setPosition(.80);

                        slideMode[1] = 1;
                        break;

                    case 0:

                        LongSlide.setPosition(1);
                        Flapper.setPosition(0.10);
                        Arm.setPosition(.40);
                        Elbow.setPosition(.25);

                        slideMode[1] = 1;
                        break;

                } */

            }



        }
    }
}
