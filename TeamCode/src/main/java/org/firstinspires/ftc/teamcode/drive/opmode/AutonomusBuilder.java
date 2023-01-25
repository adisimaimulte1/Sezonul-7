package org.firstinspires.ftc.teamcode.drive.opmode;

import android.text.method.Touch;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.hardware.*;

@TeleOp(group = "drive")

public class AutonomusBuilder extends LinearOpMode {

    double f, b, l, r;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y*0.3,
                            gamepad1.left_stick_x*0.3,
                            gamepad2.right_stick_x*0.3
                    )
            );

            if (gamepad1.x)
                drive.turn(Math.toRadians(-90));
            else if (gamepad1.b)
                drive.turn(Math.toRadians(90));

            if (gamepad1.dpad_up)
                f=0.3;
            if(gamepad1.dpad_down)
                b=0.3;
            if (gamepad1.dpad_left)
                l=0.3;
            if(gamepad1.dpad_right)
                r=0.3;
            l=r=f=b=0;
        }

    }
}
