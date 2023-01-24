package OpenCv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class AutonomusLeft extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    int Left=17;
    int Middle=18;
    int Right=19;
    int Random;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    private PIDController controller;

    public static double p = 0.0010782000149, i = 0, d = 0.000101064053001252;
    public static double f = 0.00098986;
    public static int targetHigh = 3080, targetMid = 1920, targetLow = 770, target = 200, ante_target = 200;

    private final double ticks_in_degree = 70 / 18.0;

    @Override
    public void runOpMode()
    {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor LeftMotor = hardwareMap.dcMotor.get("slider_stanga");
        DcMotor RightMotor = hardwareMap.dcMotor.get("slider_dreapta");
        Servo cot_stanga = hardwareMap.get(Servo.class, "cot_stanga");
        Servo cot_dreapta = hardwareMap.get(Servo.class, "cot_dreapta");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo level = hardwareMap.get(Servo.class, "level");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cot_dreapta.setDirection(Servo.Direction.REVERSE);

        Pose2d startPose = new Pose2d(41.75, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        gripper.setPosition(0.58);
        cot_stanga.setPosition(0.80);
        cot_dreapta.setPosition(0.80);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose) //pozitionare plasare con preload
                .strafeTo(new Vector2d(2.5,60))
                .strafeTo(new Vector2d(2.5,44.7))
                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    target=targetHigh;
                    controller.setPID(p, i, d);
                    int armPos = RightMotor.getCurrentPosition();
                    double pid = controller.calculate(armPos, target);
                    double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                    double power = pid + ff;
                    LeftMotor.setPower(power);
                    RightMotor.setPower(power);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    gripper.setPosition(0.67);
                })
                .strafeTo(new Vector2d(2.5,50))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    target=15;
                    controller.setPID(p, i, d);
                    int armPos = RightMotor.getCurrentPosition();
                    double pid = controller.calculate(armPos, target);
                    double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                    double power = pid + ff;
                    LeftMotor.setPower(power);
                    RightMotor.setPower(power);
                })
                .strafeTo(new Vector2d(1.5, 37))
                .strafeTo(new Vector2d(11, 37))
                .build();

        Trajectory zona1 = drive.trajectoryBuilder(traj.end()) //parcare zona 1
                .strafeTo(new Vector2d(60, 37))
                .build();

        Trajectory zona2 = drive.trajectoryBuilder(traj.end())
                .strafeTo(new Vector2d(33, 37))
                .build();

        /*Trajectory zona1 = drive.trajectoryBuilder(traj.end())
                .strafeTo(new Vector2d(45,18))
                .build();*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id==Middle || tag.id==Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        if(tagOfInterest==null || tagOfInterest.id==Left) {
            Random = 1;
            telemetry.addLine("Randomizare: 1");
        }else if(tagOfInterest.id==Middle)
        {
            Random=2;
            telemetry.addLine("Randomizare: 2");
        }else if(tagOfInterest.id==Right)
        {
            Random=3;
            telemetry.addLine("Randomizare: 3");
        }
        drive.followTrajectorySequence(traj);
       if (Random==2)
            drive.followTrajectory(zona2);
        else if (Random==1) drive.followTrajectory(zona1);
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}