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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="RobotCode", group="Linear OpMode")
//@Disabled
public class RobotCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private PIDController controller;
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private DcMotor tallStickL = null;
    //Right linear slide
    private DcMotor tallStickR = null;
    //Left linear slide

    private CRServo intake = null;
    //Intake Servo on port 0 on Expansion Hub
    private Servo Flapper = null;
    //Intake turner on port 1 on Expansion Hub
    private Servo Arm = null;
    //Arm servos on linear slide port 0
    private Servo Elbow = null;
    //Elbow servos on arm port 1
    private Servo Wrist = null;
    //Wrist servo on elbow port 2
    private Servo Grip = null;
    //Block Gripper port 3
    private Servo LongSlide = null;
    //Flipper extender port 4

    //private final double ticks_in_degree = 84 / 360;
    //public static double p = 0,i = 0,d = 0;
    //public static double f = 0;
    //public static int target = 0;

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

        intake = hardwareMap.get(CRServo.class, "intake");

        Flapper = hardwareMap.get(Servo.class, "Flapper");
        Grip = hardwareMap.get(Servo.class,"grip");
        Arm = hardwareMap.get(Servo.class,"arm");
        Elbow = hardwareMap.get(Servo.class,"elbow");
        Wrist = hardwareMap.get(Servo.class,"wrist");
        LongSlide = hardwareMap.get(Servo.class,"longSlide");

        //controller = new PIDController(p, i, d);

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        tallStickL.setDirection(DcMotorSimple.Direction.REVERSE);
        tallStickR.setDirection(DcMotorSimple.Direction.REVERSE);

        Grip.setPosition(.18);
        Wrist.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double x = 0.1;
        double sped = .5;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();

            double speed = 1;

            //Driving Code
            motorFR.setPower((gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) + (gamepad1.right_stick_x * speed));
            motorBR.setPower((gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) + (gamepad1.right_stick_x * speed));
            motorFL.setPower((gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) - (gamepad1.right_stick_x * speed));
            motorBL.setPower((gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) - (gamepad1.right_stick_x * speed));

            //Linear Slide Code
            if (gamepad2.left_stick_y < 0) {
                tallStickL.setPower((-1)* sped);
                tallStickR.setPower((1)* sped);
            } else if (gamepad2.left_stick_y > 0) {
                tallStickL.setPower((.5)* sped);
                tallStickR.setPower((-.5)* sped);
            } else {
                tallStickL.setPower(0);
                tallStickR.setPower(0);
            }

            //Intake Servo on port 0 on Expansion Hub
            if (gamepad1.dpad_down) {
                Flapper.setPosition(0.10);
            } else if (gamepad1.dpad_up) {
                Flapper.setPosition(0);
            }

            //Intake turner on port 1 on Expansion Hub
            if (gamepad2.right_trigger > 0) {
                intake.setPower(1);
            } else if (gamepad2.left_trigger > 0) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            //Block Gripper port 3
            if (gamepad2.x) {
                Grip.setPosition(.18);
            } else if (gamepad2.y) {
                Grip.setPosition(0);
            }

            //Arm servos on linear slide port 0
            if (gamepad2.a) {
                Arm.setPosition(1);
            } else if (gamepad2.b) {
                Arm.setPosition(.20);
            }

            //Elbow servos on arm port 1
            if (gamepad2.dpad_left) {
                Elbow.setPosition(.65);
            } else if (gamepad2.dpad_down) {
                Elbow.setPosition(.5);
            } else if (gamepad2.dpad_up) {
                Elbow.setPosition(.80);
            }

            //Wrist servo on elbow port 2
            if (gamepad2.right_bumper) {
                Wrist.setPosition(.35);
            } else if (gamepad2.left_bumper) {
                Wrist.setPosition(0);
            }

            //Flipper extender port 4
            if (gamepad1.x) {
                LongSlide.setPosition(1);
            } else if (gamepad1.y) {
                LongSlide.setPosition(0);
            }

            /* controller.setPID(p, i, d);
            int armPos = turnOnner.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toDegrees(target / ticks_in_degree)) * f;

            double power = pid + ff;

            if (gamepad2.a) {
                bigDumpie.setPosition(x += 0.08);
            } else  {
                bigDumpie.setPosition(x = 0.1);
            }
            bigDumpieTwo.setPosition(bigDumpie.getPosition()); */

            // Show the elapsed game time and wheel power.
            // telemetry.addData("pos", armPos);
            //telemetry.addData("target", target);
        }
    }
}
