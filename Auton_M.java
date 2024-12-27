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



@Autonomous(name="CycledRightSide", group="Linear OpMode")
public class Auton_M extends LinearOpMode {


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

    private Servo Flapper = null;
    //Intake turner on port 0 on Expansion Hub
    private CRServo intake = null;
    //Intake Servo on port 1 on Expansion Hub
    private CRServo intake1 = null;
    //Intake Servo on port 2 on Expansion Hub
    private Servo LongSlide = null;
    //Flipper extender port 4 Expansion Hub
    private Servo Arm = null;
    //Arm servos on linear slide port 0
    private Servo Elbow = null;
    //Elbow servos on arm port 1
    private Servo Wrist = null;
    //Wrist servo on elbow port 2
    private Servo Grip = null;
    //Block Gripper port 3

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


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
        intake1 = hardwareMap.get(CRServo.class, "intake1");

        Flapper = hardwareMap.get(Servo.class, "Flapper");
        Grip = hardwareMap.get(Servo.class,"grip");
        Arm = hardwareMap.get(Servo.class,"arm");
        Elbow = hardwareMap.get(Servo.class,"elbow");
        Wrist = hardwareMap.get(Servo.class,"wrist");
        LongSlide = hardwareMap.get(Servo.class,"longSlide");


        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


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

        //ADD AUTON CODE HERE
        startup();
        holdScoringPiece(100);

        moveForward(.5, 300);
        moveStop(1, 200);
        rotateRight(.5, 875);
        moveStop(1, 200);
        moveRight(.5, 400);
        moveStop(1, 200);
        moveBackward(.5, 575);
        moveStop(1, 200);

        scoreSpeciman(200);
        moveForward(.5, 150);
        moveStop(1, 200);
        Grip.setPosition(.18);
        neutralPosition(200);
        moveStop(1, 200);

    }
    //Function definitions
    protected void moveForward(double speed, int time) {
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        sleep(time);
    }

    protected void moveBackward(double speed, int time)
    {
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);

        sleep(time);
    }

    protected void moveLeft(double speed, int time)
    {
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
        motorFL.setPower(-speed);
        motorFR.setPower(speed);

        sleep(time);
    }

    protected void moveRight(double speed, int time)
    {
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(-speed);

        sleep(time);
    }

    protected void moveStop(double speed, int time) {
        motorBL.setPower(speed * 0);
        motorBR.setPower(speed * 0);
        motorFL.setPower(speed * 0);
        motorFR.setPower(speed * 0);

        sleep(time);
    }

    protected void startup() {
        Grip.setPosition(.18);
        Wrist.setPosition(0);
        LongSlide.setPosition(1);
        Flapper.setPosition(0.10);
    }

    protected void rotateRight(double speed, int time) {
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
        motorFL.setPower(speed);
        motorFR.setPower(-speed);

        sleep(time);
    }

    protected void rotateLeft(double speed, int time) {
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
        motorFL.setPower(-speed);
        motorFR.setPower(speed);

        sleep(time);
    }

    protected void grabFromWall(int time) {
        LongSlide.setPosition(.95);
        Flapper.setPosition(0.10);
        Arm.setPosition(.20);
        Elbow.setPosition(.80);
        Grip.setPosition(.18);
        sleep(1000);
        Grip.setPosition(0);

        sleep(time);
    }

    protected void holdScoringPiece(int time) {
        LongSlide.setPosition(1);
        Flapper.setPosition(0.10);
        Elbow.setPosition(.75);
        Wrist.setPosition(.35);
        Arm.setPosition(.70);
        Grip.setPosition(0);

        sleep(time);
    }

    protected void scoreSpeciman(int time) {
        Grip.setPosition(0);
        LongSlide.setPosition(1);
        Flapper.setPosition(0.10);
        Arm.setPosition(.40);
        Elbow.setPosition(.25);

        sleep(time);
    }

    protected void neutralPosition(int time) {
        LongSlide.setPosition(1);
        Flapper.setPosition(0.10);
        Elbow.setPosition(.75);
        Wrist.setPosition(.35);
        Arm.setPosition(.80);
        Grip.setPosition(.18);

        sleep(time);
    }
}
