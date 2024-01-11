

package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.ArrayList;


@Autonomous
public class ScottsBotsBlue extends LinearOpMode {
    OpenCvCamera camera;

    static final double FEET_PER_METER = 3.28084;

    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;
    // AprilTagDetection tagOfInterest = null;
    DcMotor leftWinch;
    DcMotor rightWinch;

    DcMotor intake;

    DcMotor shooter;

    Servo hips;
    Servo arch;
    Servo frontLeg;
    Servo backLeg;

    Servo sneakyLink; // facing robot (sneaky link suuuus)
    Servo sneakyRink;
    MecanumDrive drive;
    int zone = 0;


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode () throws InterruptedException {
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");
        hips = hardwareMap.servo.get("hips");
        arch = hardwareMap.servo.get("arch");
        frontLeg = hardwareMap.servo.get("frontLeg");
        backLeg = hardwareMap.servo.get("backLeg");
        sneakyLink = hardwareMap.servo.get("sneakyLink");
        sneakyRink = hardwareMap.servo.get("sneakyRink");
        drive = new MecanumDrive(hardwareMap, new Pose2d(12.00, 63, Math.toRadians(90)));
        TeamPropDetector.startPropDetection(hardwareMap, telemetry);
        drive.setTelemetry(telemetry);


        rightWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWinch.setDirection((DcMotorSimple.Direction.FORWARD));


        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightWinch.setTargetPosition(0);
        leftWinch.setTargetPosition(0);
//        chainBar.setTargetPosition(ARM_0);
        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hips.setPosition(.22);
        arch.setPosition(.69);
        backLeg.setPosition(.77);  // .9 is closed pos
        frontLeg.setPosition(.52); //  .6 is closed pos
        sneakyLink.setPosition(1); //  up from 0
        sneakyRink.setPosition(0); // down from 0


        waitForStart();
        zone = TeamPropDetector.getBluePropZone();
        TeamPropDetector.endPropDetection();
        //Actions.runBlocking(drive.actionBuilder(new Pose2d(12.00, 63, Math.toRadians(90))).strafeTo(new Vector2d(12,30)).build());
        //Drive to SPIke Mark and get from stack
        if (zone == 1) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(15, 44.00))
                    .strafeToLinearHeading(new Vector2d(15.83, 44.43), Math.toRadians(150))
                    .strafeToConstantHeading(new Vector2d(20, 44.33))
                    //.strafeToConstantHeading(new Vector2d(10, 44.33))
                    //.strafeToConstantHeading(new Vector2d(13,36))

                    .build());
        } else if (zone == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                    .strafeTo(new Vector2d(13,35))
                    .strafeTo(new Vector2d(28,40))
                    .strafeTo(new Vector2d(28,14))
                    .turnTo(Math.toRadians(180))
                    .build());
            sneakyLink.setPosition(.75); //.1 is rest pos
            sneakyRink.setPosition(.25); // 01 is rest pos
            sleep(200);
            intake.setPower(-1);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
            .strafeTo(new Vector2d(25,14 ))
                    .build());
            sleep(1500);
            sneakyLink.setPosition(1); // 1 is rest pos
            sneakyRink.setPosition(0); // 0 is rest pos
            sleep(300);
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);


        } else {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(15, 38), Math.toRadians(70))
                    .strafeToConstantHeading(new Vector2d(13,46))
                    .build());
        }
        drive.updatePoseEstimate();
        telemetry.addLine("Pose" + drive.pose.position);
        telemetry.addLine("HEading" + Math.toDegrees(drive.pose.heading.log()));
        telemetry.update();


        //Drive to Backdrop

        if (zone == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(14, 44))
                    .strafeToLinearHeading(new Vector2d(40, 44), Math.toRadians(180))

                    .build());
        } else if (zone == 2) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(75,14))
                    .strafeTo(new Vector2d(120,-8))
                    .strafeTo(new Vector2d(126,-10))
                    .build());


        } else Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(37, 38), Math.toRadians(180))
                .build());
        drive.updatePoseEstimate();
        telemetry.addLine("Pose" + drive.pose.position);
        telemetry.addLine("HEading" + Math.toDegrees(drive.pose.heading.log()));
        telemetry.update();
        while(!isStopRequested()){

        }

        //PARK
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(40, 64))
                .strafeTo(new Vector2d(48, 64))
                .build());

    }









}


