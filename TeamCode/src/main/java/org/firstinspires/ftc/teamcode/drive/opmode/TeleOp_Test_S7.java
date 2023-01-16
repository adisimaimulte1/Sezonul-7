/*package org.firstinspires.ftc.teamcode.drive.opmode;

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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "drive")

public class TeleOp_Test_S7 extends LinearOpMode {
    double mls(double x) //modul de liniaritate si sens
    {
        return x*x*x;
    }
    double liftSensivity=0.075;
    double lowerSensitivity=0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        double sensivity=0.6, position=0.3, movementSensitivity=-0.6, grip=0, liftP=0, lowerP=0;
        boolean u1=true, u2=true, s1=true, s2=true;
        DcMotor slider_stanga = hardwareMap.dcMotor.get("slider_stanga");
        DcMotor slider_dreapta = hardwareMap.dcMotor.get("slider_dreapta");
        Servo cot_stanga = hardwareMap.get(Servo.class, "cot_stanga");
        Servo cot_dreapta = hardwareMap.get(Servo.class, "cot_dreapta");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        slider_dreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        cot_dreapta.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                lowerSensitivity+=0.005;
                Thread.sleep(500);
            }
            if (gamepad2.left_bumper)
            {
                lowerSensitivity-=0.005;
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
            /*telemetry.addData("Pozitie servo cot:", position);
            telemetry.addData("Unghi servo cot:", position*270);

            //control gripper
            if (gamepad2.y) // pick
                grip=0;
            if (gamepad2.x) // drop
                grip=0.3;
            //lift
            if(gamepad2.left_stick_y<-0.2)
                liftP = liftPower((double) holder.getCurrentPosition() * 360 / 8192);
            else if(gamepad2.left_stick_y>0.2)
            {
                liftP=antiG((double) holder.getCurrentPosition()*360/8192);
            }
            else
                liftP = 0;
                umard.setPower(liftP);
                umars.setPower(liftP);
                holder.setPower(-liftP);
            //lower



            position=liftAngle((double)holder.getCurrentPosition()*360/8192);

            telemetry.addData("y ", gamepad2.left_stick_y);
            telemetry.addData("power ", liftPower((double)holder.getCurrentPosition()*360/8192));
            telemetry.addData("power2 ", antiG((double)holder.getCurrentPosition()*360/8192));

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
        double y=-0.0026*x*x*x*x+0.028*x*x*x-0.039*x*x-0.3753*x+10;
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
        else if (x>55 && x<110){
            return (x-55)*0.0073+0.27;
        }
        else { return 0.675;}

    }
    double antiG (double x)
    {
        x=x/10;
        double y=-0.0066*x*x*x*x+0.1992*x*x*x-1.9275*x*x+5.5915*x+5;
        if (x>=0)
            return y*lowerSensitivity;
        else
            return 0;
    }

}
*/