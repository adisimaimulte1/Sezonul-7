package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.opmode.PIDF_Controller.d;
import static org.firstinspires.ftc.teamcode.drive.opmode.PIDF_Controller.i;
import static org.firstinspires.ftc.teamcode.drive.opmode.PIDF_Controller.p;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class TeleOp_Slide_Test1 extends LinearOpMode {

    double mls(double x)
    {
        return x*x*x;
    }

    boolean o_c = true;
    double movementSensitivity=-0.6, servo = 0.5;

    private PIDController controller;

    public static double p = 0.0010782000149, i = 0, d = 0.000100964053001252;
    public static double f = 0.00095984;
    public static int targetHigh = 3080, targetMid = 1920, targetLow = 770, target = 200, ante_target = 200;

    private final double ticks_in_degree = 70 / 18.0;

    public void runOpMode() throws InterruptedException{

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor LeftMotor = hardwareMap.dcMotor.get("slider_stanga");
        DcMotor RightMotor = hardwareMap.dcMotor.get("slider_dreapta");
        Servo cot_stanga = hardwareMap.get(Servo.class, "cot_stanga");
        Servo cot_dreapta = hardwareMap.get(Servo.class, "cot_dreapta");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cot_dreapta.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            drive.setWeightedDrivePower( //movement
                    new Pose2d(
                            mls(gamepad1.left_stick_y) * movementSensitivity,
                            mls(gamepad1.left_stick_x) * movementSensitivity,
                            mls(-gamepad1.right_stick_x) * movementSensitivity
                    )
            );

            if (gamepad2.b) { //gripper position
                if (o_c) {
                    o_c = false;  //open gripper
                    gripper.setPosition(0.67);
                    sleep(500);
                }
                else {
                    o_c = true;  //close gripper
                    gripper.setPosition(0.58);
                    sleep(500);
                }
            }

            /*if (gamepad2.y) { //ground pick //arm position
                cot_stanga.setPosition(0.15);
                cot_dreapta.setPosition(0.15);
            }
            if (gamepad2.x) {  //pre-drop
                cot_stanga.setPosition(0.85);
                cot_dreapta.setPosition(0.85);
            }*/


            if (gamepad2.a) { //drop
                cot_stanga.setPosition(0.9);
                cot_dreapta.setPosition(0.9);
            }

            if (gamepad2.dpad_up) //slider position
            {
                ante_target = target;
                target = targetHigh;
                if (ante_target!=target)
                {
                    cot_stanga.setPosition(0.20);
                    cot_dreapta.setPosition(0.20);
                    gripper.setPosition(0.58);
                }
            }
            if (gamepad2.dpad_left) {
                ante_target = target;
                target = targetMid;
                if (ante_target!=target)
                {
                    cot_stanga.setPosition(0.20);
                    cot_dreapta.setPosition(0.20);
                    gripper.setPosition(0.58);
                }
            }
            if (gamepad2.dpad_right) {
                ante_target = target;
                target = targetLow;
                if (ante_target!=target)
                {
                    cot_stanga.setPosition(0.20);
                    cot_dreapta.setPosition(0.20);
                    gripper.setPosition(0.58);
                }
            }
            if (gamepad2.dpad_down) {
                ante_target = target;
                target = 200;
                if (ante_target!=target)
                {
                    cot_stanga.setPosition(0.85);
                    cot_dreapta.setPosition(0.85);
                    gripper.setPosition(0.67);
                }
            }

            /*if (gamepad1.dpad_up)
            {
                servo+=0.05;
                sleep(500);
            }
            if (gamepad1.dpad_down)
            {
                servo-=0.05;
                sleep(500);
            }
            cot_stanga.setPosition(servo);
            cot_dreapta.setPosition(servo);*/

            controller.setPID(p, i, d);
            int armPos = RightMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            LeftMotor.setPower(power);
            RightMotor.setPower(power);

            /*telemetry.addData("pos servo: ", servo);
            telemetry.update();*/
        }
    }
}
