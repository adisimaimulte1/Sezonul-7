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
    double mls(double x) //modul de liniaritate si sens
    {
        return x*x*x;
    }
    double liftSensivity=0.06;
    @Override
    public void runOpMode() throws InterruptedException {
        double sensivity=0.6, position=0.3, movementSensitivity=0.6, grip=0, liftP=0;
        boolean u1=true, u2=true, s1=true, s2=true;
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
            drive.setWeightedDrivePower(
                    new Pose2d(
                            mls(gamepad1.left_stick_y) * movementSensitivity,
                            mls(gamepad1.left_stick_x) * movementSensitivity,
                            mls(gamepad1.right_stick_x) * movementSensitivity
                    )
            );

            //one push
            //pozitie antebrat
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
            /*if (gamepad2.dpad_up)
            {
                position+=0.05;
                Thread.sleep(500);
            }
            if (gamepad2.dpad_down)
            {
                position-=0.05;
                Thread.sleep(500);
            }*/
            telemetry.addData("Pozitie servo cot:", position);
            telemetry.addData("Unghi servo cot:", position*270);

            //control gripper
            if (gamepad2.y) // pick
                grip=0;
            if (gamepad2.x) // drop
                grip=0.3;
            //lift
            if(gamepad2.left_stick_y<-0.2)
                liftP=liftPower((double)holder.getCurrentPosition()*360/8192);

            else
                liftP=0;
                position=liftAngle((double)holder.getCurrentPosition()*360/8192);
            umard.setPower(liftP);
            umars.setPower(liftP);
            holder.setPower(-liftP);
            telemetry.addData("y ", gamepad2.left_stick_y);
            telemetry.addData("power ", liftPower((double)holder.getCurrentPosition()*360/8192));
            //control si atribuire
            cotd.setPosition(position);
            cots.setPosition(position);
            gripper.setPosition(grip);


            //indicatii
            telemetry.addData("encoder ", (double)holder.getCurrentPosition()*360/8192);
            telemetry.update();
        }

    }
    double liftPower (double x)
    {
        x=x/10;
        double y=-0.0008*x*x*x*x-0.0065*x*x*x+0.1272*x*x-0.5397*x+10;
        if (x<=10.5)
            return y*liftSensivity;
        else
            return 0;
    }
    double liftAngle (double x)
    {
        if (x<=55){
            return 0.25;
        }
        else if (x>55 && x<120){
            return (x-55)*0.007+0.27;
        }
        else { return 0.675;}

    }
}
