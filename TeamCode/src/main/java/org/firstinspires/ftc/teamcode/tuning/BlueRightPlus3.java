package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
public class BlueRightPlus3 extends LinearOpMode {
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
        drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 63, Math.toRadians(90)));
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

        hips.setPosition(.22); // og is .22
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
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36.00, 63.00, Math.toRadians(90.00)))
                    .strafeTo(new Vector2d(-40.8, 55.78))
                    .strafeToLinearHeading(new Vector2d(-25.5, 38.84), Math.toRadians(120.00))
                    .strafeToLinearHeading(new Vector2d(-50.24, 50.35), Math.toRadians(180.00))
                    .splineToLinearHeading(new Pose2d(-58.5, 37.7, Math.toRadians(180.00)), Math.toRadians(222.00)) //stack
                    .build());
            arch.setPosition(.69);
            sneakyLink.setPosition(.85); //weirdo position la la lala la
            sneakyRink.setPosition(.15);
            intake.setPower(-1);
            sleep(850);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToConstantHeading(new Vector2d(-7.71, 62.86), Math.toRadians(0.00))
                            .afterDisp(10,(a)->{ //20
                                intake.setPower(1);
                                return false;
                            })
                            .afterDisp(15,(a)->{
                                backLeg.setPosition(.81);
                                frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
                                sleep(200);
                                rightWinch.setTargetPosition(-1400);
                                rightWinch.setPower(1);
                                leftWinch.setTargetPosition(-1400);
                                leftWinch.setPower(1);
                                sleep(850);
                                arch.setPosition(.363);
                                return false;
                            })
                            .splineToConstantHeading(new Vector2d(54.75,29.7),Math.toRadians(45))
                            .build());
            intake.setPower(0);
//            sleep(300);
//            hips.setPosition(.18);
            backLeg.setPosition(.76);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.46);
            sleep(50);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(50.5, 35.7))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(150);
            hips.setPosition(.18);
            sleep(200);
            arch.setPosition(.69);
            sleep(50);
            rightWinch.setTargetPosition(-20);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-20);
            leftWinch.setPower(1);
            hips.setPosition(.27);
            sleep(200);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sleep(200);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(21.23,10.55),Math.toRadians(191.12))
                    .splineToConstantHeading(new Vector2d(-55.75,13.8),Math.toRadians(180))
                    .build());
            sneakyLink.setPosition(.7); //weirdo position la la lala la
            sneakyRink.setPosition(.3);
            intake.setPower(-1);
            sleep(300);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .afterDisp(4,(a)->{ //20
                        intake.setPower(1);
                        return false;
                    })
                    .splineToConstantHeading(new Vector2d(-7.71, 62.86), Math.toRadians(0.00))
                    .afterDisp(1,(a)->{ //10
                        backLeg.setPosition(.81);
                        frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
                        sleep(200);
                        rightWinch.setTargetPosition(-1400);
                        rightWinch.setPower(1);
                        leftWinch.setTargetPosition(-1400);
                        leftWinch.setPower(1);
                        sleep(850);
                        arch.setPosition(.363);
                        return false;
                    })
                    .splineToConstantHeading(new Vector2d(55.5,29.7),Math.toRadians(45))
                    .build());
            intake.setPower(0);
            backLeg.setPosition(.81);
            frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
            sleep(200);
            arch.setPosition(.363);
            sleep(200);
            backLeg.setPosition(.76);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.46);
            sleep(250);
//
        } else if (zone == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36.00, 63.00, Math.toRadians(90.00)))
                    .strafeToConstantHeading(new Vector2d(-36.10, 32.77))
                    .strafeToLinearHeading(new Vector2d(-58.5, 37.7), Math.toRadians(180.00))
                    .build());
            hips.setPosition(.27);
            sneakyLink.setPosition(.85); //weirdo position la la lala la
            sneakyRink.setPosition(.15);
            intake.setPower(-1);
            sleep(850);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(-55,37.7))
                                    .build());
                        sneakyLink.setPosition(.6); //weirdo position la la lala la
                        sneakyRink.setPosition(.4);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(-43.7,60))
                    .afterDisp(12,(a)->{ //20
                        intake.setPower(1);
                        return false;
                    })
                    .strafeToConstantHeading(new Vector2d(45.75,58.8))
                    .afterDisp(12,(a)->{
                        backLeg.setPosition(.81);
                        frontLeg.setPosition(.61);
//            sleep(300);
                        sleep(200);
                        rightWinch.setTargetPosition(-850);
                        rightWinch.setPower(1);
                        leftWinch.setTargetPosition(-850);
                        leftWinch.setPower(1);
                        sleep(850);
                        arch.setPosition(.363);
                        sleep(200);
                        hips.setPosition(.95);
                        return false;

                    })
                    .strafeToConstantHeading(new Vector2d(54.75,29.5))
                            .build());
            intake.setPower(0);
//            sleep(300);
//           hips.setPosition(.18);
            backLeg.setPosition(.76);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.46);
            sleep(50);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(50.5, 29.5))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(150);
            hips.setPosition(.18);
            sleep(500);
            arch.setPosition(.69);
            sleep(50);
            rightWinch.setTargetPosition(-20);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-20);
            leftWinch.setPower(1);
            hips.setPosition(.27);
            sleep(200);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sleep(200);
            sneakyLink.setPosition(.8); //weirdo position la la lala la
            sneakyRink.setPosition(.2);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(24.34, 57.83), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-60.52, 34.71), Math.toRadians(225.00))
                    .build());
            sneakyLink.setPosition(.7); //weirdo position la la lala la
            sneakyRink.setPosition(.3);
            intake.setPower(-1);
            sleep(400);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(-37.7,56.6))
                    .afterDisp(12,(a)->{ //20
                        intake.setPower(1);
                        return false;
                    })
                    .strafeToConstantHeading(new Vector2d(45.75,56))
                    .afterDisp(10,(a)->{
                        backLeg.setPosition(.81);
                        frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
                        sleep(200);
                        rightWinch.setTargetPosition(-1400);
                        rightWinch.setPower(1);
                        leftWinch.setTargetPosition(-1400);
                        leftWinch.setPower(1);
                        sleep(850);
                        arch.setPosition(.363);
                        return false;
                    })
                    .strafeToConstantHeading(new Vector2d(54.75,35.7))
                    .build());
            intake.setPower(0);
            backLeg.setPosition(.81);
            frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
            sleep(200);
            arch.setPosition(.363);
            sleep(200);
            backLeg.setPosition(.76);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.46);
            sleep(150);


        } else {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-36.00, 63.00, Math.toRadians(90.00)))
                    .strafeTo(new Vector2d(-35.29, 41.78))
                    .strafeToLinearHeading(new Vector2d(-25.55, 31.84), Math.toRadians(110.00))
                    .strafeToLinearHeading(new Vector2d(-46.24, 54.35), Math.toRadians(180.00))
                    .build());
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(new Pose2d(-58.62, 35.70, Math.toRadians(180.00)), Math.toRadians(222.00))
                    .build());
            arch.setPosition(.69);
            sneakyLink.setPosition(.85); //weirdo position la la lala la
            sneakyRink.setPosition(.15);
            intake.setPower(-1);
            sleep(2750);;
            sleep(300);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .afterDisp(2,(a)->{ //20
                        intake.setPower(1);
                        return false;
                    })
                            .splineToConstantHeading(new Vector2d(-7.71, 62.86), Math.toRadians(0.00))
                            .afterDisp(1,(a)->{
                                backLeg.setPosition(.81);
                                frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
                                sleep(200);
                                rightWinch.setTargetPosition(-1400);
                                rightWinch.setPower(1);
                                leftWinch.setTargetPosition(-1400);
                                leftWinch.setPower(1);
                                sleep(850);
                                arch.setPosition(.363);
                                return false;
                            })
                            .splineToConstantHeading(new Vector2d(54.75,29.7),Math.toRadians(45))
                            .build());
            intake.setPower(0);
//            sleep(300);
//            hips.setPosition(.18);
            backLeg.setPosition(.76);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.46);
            sleep(50);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(50.5, 35.7))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(150);
            hips.setPosition(.18);
            sleep(200);
            arch.setPosition(.69);
            sleep(50);
            rightWinch.setTargetPosition(-20);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-20);
            leftWinch.setPower(1);
            hips.setPosition(.27);
            sleep(200);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sleep(200);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(21.23,10.55),Math.toRadians(191.12))
                    .splineToConstantHeading(new Vector2d(-55.75,13.8),Math.toRadians(180))
                    .build());
            sneakyLink.setPosition(.7); //weirdo position la la lala la
            sneakyRink.setPosition(.3);
            intake.setPower(-1);
            sleep(300);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(-36.91, 11.56))
                    .afterDisp(15,(a)->{ //10
                        backLeg.setPosition(.81);
                        frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
                        sleep(200);
                        rightWinch.setTargetPosition(-1400);
                        rightWinch.setPower(1);
                        leftWinch.setTargetPosition(-1400);
                        leftWinch.setPower(1);
                        sleep(850);
                        arch.setPosition(.363);
                        return false;
                    })
                    .splineToConstantHeading(new Vector2d(55.5,29.7),Math.toRadians(45))
                    .build());
            intake.setPower(0);
            backLeg.setPosition(.81);
            frontLeg.setPosition(.61);
//            sleep(300);
//            hips.setPosition(.18); hips hips hips hips
            sleep(200);
            arch.setPosition(.363);
            sleep(200);
            backLeg.setPosition(.76);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.46);
            sleep(250);


        }
        drive.updatePoseEstimate();
        telemetry.addLine("Pose" + drive.pose.position);
        telemetry.addLine("HEading" + Math.toDegrees(drive.pose.heading.log()));
        telemetry.update();


        //Drive to Backdrop

        if (zone == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(54.96, 62.87))
                    .build());
        } else if (zone == 2) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(47, 47),Math.toRadians(270))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(50);
            hips.setPosition(.18);
            sleep(200);
            arch.setPosition(.69);
            sleep(250);
            rightWinch.setTargetPosition(-20);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-20);
            leftWinch.setPower(1);
            hips.setPosition(.27);
            sleep(300);
            // hips.setPosition(.27);
//            sleep(300);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sleep(200);


//            Actions.runBlocking(drive.actionBuilder(drive.pose) start
//                    .strafeTo( new Vector2d(28,48))
//                    .build());
//            intake.setPower(0);
//
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//            .strafeToConstantHeading(new Vector2d(25,26))
//                    .strafeToLinearHeading(new Vector2d(100,26), Math.toRadians(180))
//                    .build());
//
//            backLeg.setPosition(.9);
//            frontLeg.setPosition(.7);
//            sleep(400);
//            rightWinch.setTargetPosition(-800);
//            rightWinch.setPower(.8);
//            leftWinch.setTargetPosition(-800);
//            leftWinch.setPower(.8);
//            arch.setPosition(.363);
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(new Vector2d(133,50),Math.toRadians(180))
//                    .build());
//            backLeg.setPosition(.7);  // .9 is closed pos
//            sleep(200);
//            frontLeg.setPosition(.46);
//            sleep(200);
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToConstantHeading(new Vector2d(130,49.5))
//                    .build());
//

        } else
            sleep(200);

        drive.updatePoseEstimate();
        telemetry.addLine("Pose" + drive.pose.position);
        telemetry.addLine("HEading" + Math.toDegrees(drive.pose.heading.log()));
        telemetry.update();
        while(!isStopRequested()){

        }

        if(zone == 1){
            sleep(200);
        }
        else if( zone == 2){

            sleep(300);
        }
        else

            //PARK
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(54.35, 61.05))
                    .turnTo(Math.toRadians(270))
                    .build());

    }










}