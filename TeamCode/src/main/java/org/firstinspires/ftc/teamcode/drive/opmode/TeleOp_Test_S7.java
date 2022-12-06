package org.firstinspires.ftc.teamcode.drive.opmode;

import android.text.method.Touch;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class TeleOp_Test_S7 extends LinearOpMode {
    double liftSensivity=0.07, angleCot=0.5, movementSensitivity=0.6, grip=0, powerUmar=0, angleUmar, resistanceSensivity=0.04;
    double mls(double x) //modul de liniaritate si sens
    {
        return x*x*x*movementSensitivity;
    }
    double liftPower (double x) //ecuatia miscarii ascendente a bratului
    {
        x=x/10;
        double y;
        if (x<=8) {
            y = -0.0027 * x * x * x * x + 0.0422 * x * x * x - 0.1883 * x * x + 10;
        }
        else
            y=0;
        return y*liftSensivity;
    }
    double resistivePower(double x) //ecuatia anti-graviationala
    {
        x=x/10;
        double y;
        if (x>1)
            y=-0.0124*x*x*x*x+0.3152*x*x*x-2.5885*x*x+6.496*x+5;
        else
            y=0;
            return y*resistanceSensivity;
    }
    double AngleInAscendingMotion (double x) //ecuatia dependentei pozitiei cotului in functie de unghiul umarului in urcare
    {
        if (x >= 90)
            return 0.675;
        else
            return 0.3;
    }
    double AngleInDescendingMotion (double x) //ecuatia dependentei pozitiei cotului in functie de unghiul umarului in coborare
    {
        if (x >= 100)
            return 0.675;
        else if (x>=40)
            return 0.3;
        else
            return 0.6;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor umard = hardwareMap.dcMotor.get("umarDreapta");
        DcMotor umars= hardwareMap.dcMotor.get("umarStanga");
        DcMotor holder = hardwareMap.dcMotor.get("motorCentral");
        Servo cots = hardwareMap.get(Servo.class, "cotStanga");
        Servo cotd = hardwareMap.get(Servo.class, "cotDreapta");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        umars.setDirection(DcMotorSimple.Direction.REVERSE);
        holder.setDirection(DcMotorSimple.Direction.REVERSE);
        cots.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested()) {
            //movement
            drive.setWeightedDrivePower(
                    new Pose2d(
                            mls(gamepad1.left_stick_y),
                            mls(gamepad1.left_stick_x),
                            mls(gamepad2.right_stick_x)
                    )
            );

            angleUmar=(double)holder.getCurrentPosition()*360/8192;

            if (gamepad2.right_bumper)
            {
                liftSensivity+=0.005;
                Thread.sleep(500);
            }
            if (gamepad2.left_bumper)
            {
                liftSensivity-=0.005;
                Thread.sleep(500);
            }


            if (gamepad2.right_trigger>0)
            {
                resistanceSensivity+=0.005;
                Thread.sleep(500);
            }
            if (gamepad2.left_trigger>0)
            {
                resistanceSensivity-=0.005;
                Thread.sleep(500);
            }

            //control gripper
            if (gamepad2.y) // pick
                grip=0;
            if (gamepad2.x) // drop
                grip=0.3;

            //lift
            if(gamepad2.left_stick_y<-0.2)
            {
                powerUmar=liftPower(angleUmar);
                angleCot=AngleInAscendingMotion(angleUmar);
            }
            else if(gamepad2.left_stick_y>0.2)
            {
                powerUmar=resistivePower(angleUmar);
                angleCot=AngleInDescendingMotion(angleUmar);
            }
            else
                powerUmar=0;


            //control si atribuire
            umard.setPower(powerUmar);
            umars.setPower(powerUmar);
            holder.setPower(powerUmar);
            cotd.setPosition(angleCot);
            cots.setPosition(angleCot);
            gripper.setPosition(grip);

            telemetry.addData("Cot:", angleCot);
            telemetry.addData("Umar:", angleUmar);
            telemetry.addData("Sensivitate Brat In Urcare:", liftSensivity);
            telemetry.addData("Sensivitate miscare ascendenta", liftSensivity);
            telemetry.update();
        }

    }
}
