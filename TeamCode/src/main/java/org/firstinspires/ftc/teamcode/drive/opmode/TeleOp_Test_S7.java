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
    double liftSensivity=0.075, angleCot=0.5, movementSensitivity=0.6, grip=0, powerUmar=0, angleUmar, resistanceSensivity=0.04, i=1;
    double mls(double x) //modul de liniaritate si sens
    {
        return x*x*x*movementSensitivity;
    }
    double liftPower (double x) //ecuatia miscarii ascendente a bratului
    {
        x=x/10;
        if (x<=10) {
            double y = -0.0027 * x * x * x * x + 0.0422 * x * x * x - 0.1883 * x * x + 10;
            return y*liftSensivity;
        }
        else
            return 0;
    }
    double resistivePower(double x) //ecuatia anti-graviationala
    {
        x=x/10;
        double y=-0.0005*x*x*x*x+0.0038*x*x*x-0.3961*x*x+2.7288*x+5;
        if (x>2.5)
            return y*resistanceSensivity;
        else
            return 0;
    }
    double AngleInAscendingMotion (double x) //ecuatia dependentei pozitiei cotului in functie de unghiul umarului
    {
        if (x >= 85)
            return 0.71;
        else if (x>=50)
            return 0.5;
        else
            return 0.3;
    }
    double AngleInDescendingMotion (double x) //ecuatia dependentei pozitiei cotului in functie de unghiul umarului
    {
        if (x>100)
            return 0.71;
        else
            return 0.35;
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
        cots.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested()) {
            //movement


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

            if (gamepad2.dpad_right)
            {
                angleCot+=0.05;
                Thread.sleep(500);
            }
            if (gamepad2.dpad_left)
            {
                angleCot-=0.05;
                Thread.sleep(500);
            }
            //control gripper
            if (gamepad2.y) // pick
            {
                if (grip==0.3)
                grip=0;
                else
                    grip=0.3;
                Thread.sleep(500);
            }


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
            if(gamepad2.b)
                angleCot=0.52;
            if(gamepad2.a)
            {
                if(i==1)
                    i=-1;
                else
                    i=1;
                Thread.sleep(500);
            }
            //atribuire pozitie absoluta coate


            //control si atribuire
            cotd.setPosition(angleCot);
            cots.setPosition(angleCot);
            umard.setPower(powerUmar*i);
            umars.setPower(powerUmar*i);
            holder.setPower(-powerUmar*i);
            gripper.setPosition(grip);

            telemetry.addData("Cot:", angleCot);
            telemetry.addData("Umar:", angleUmar);
            telemetry.addData("liftS", liftSensivity);
            telemetry.addData("antiG", resistanceSensivity);
            telemetry.addData("Sensivitate Brat:", liftSensivity);
            telemetry.update();
        }

    }
}