package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Avi",group = "Linear OpMode")
public class BigRobot extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;
    public Servo turningBrick;
    public Servo Ducky;
    public Servo Intake;
    public Servo Bucky;
    public Servo Dumpy;
    public Servo Dump;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        turningBrick = hardwareMap.get(Servo.class, "turningBrick");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            int speed = 1;
            double y_input = -gamepad1.left_stick_y;
            double x_input = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            telemetry.addLine(gamepad1.left_stick_y + " " + gamepad1.left_stick_x);
            telemetry.update();
            motorFR.setPower((y_input * speed) - (x_input * speed) - (turn * speed));
            motorBR.setPower((y_input * speed) + (x_input * speed) - (turn * speed));
            motorFL.setPower((y_input * speed) + (x_input * speed) + (turn * speed));
            motorBL.setPower((y_input * speed) - (x_input * speed) + (turn * speed));

            /*
            motorFR.setPower((gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) + gamepad1.right_trigger - gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorBR.setPower((gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) - gamepad1.right_trigger + gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorFL.setPower(-(gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) + gamepad1.right_trigger + gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorBL.setPower(-(gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) - gamepad1.right_trigger - gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            */

            double suckIntake1 = gamepad1.right_trigger;
            if (suckIntake1 > .1) {
                turningBrick.setPosition(180);
            } else if (gamepad1.left_trigger > .1) {
                turningBrick.setPosition(0);
            }

            double suckIntake2 = gamepad1.right_trigger;
            if (suckIntake2 > .1) {
                Intake.setPosition(280);
            } else if (gamepad1.left_trigger > .1) {
                Intake.setPosition(10);
            }

            if (gamepad2.a) {
                Ducky.setPosition(280);
            } else if (gamepad2.b) ;
            {
                Ducky.setPosition(10);
            }
            if (gamepad2.b) {
                Bucky.setPosition(360);
            } else if (gamepad2.y) ;
            {
                Bucky.setPosition(10);
            }
            double Dump1 = gamepad2.right_trigger;
            if (Dump1 > .1) {
                Dumpy.setPosition(180);
            }  else if (gamepad2.left_trigger > .1)  {
                Dumpy.setPosition(90);
            }

            double Dump2 = gamepad2.right_trigger;
            if (Dump2 > .1) {
                Dump.setPosition(180);
            } else if (gamepad2.left_trigger > .1)  {
                Dump.setPosition(90);
            }
        }
    }
}
