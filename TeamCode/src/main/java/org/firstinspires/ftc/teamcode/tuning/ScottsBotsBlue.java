

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

        hips.setPosition(.22); // og is .22
        arch.setPosition(.69);
        backLeg.setPosition(.77);  // .9 is closed pos
        frontLeg.setPosition(.52); //  .6 is closed pos
        sneakyLink.setPosition(1); //  up from 0
        sneakyRink.setPosition(0); // down from 0


        waitForStart();
        zone = TeamPropDetector.getBluePropZone();
        //zone=1;
        TeamPropDetector.endPropDetection();
        //Actions.runBlocking(drive.actionBuilder(new Pose2d(12.00, 63, Math.toRadians(90))).strafeTo(new Vector2d(12,30)).build());
        //Drive to SPIke Mark and get from stack
        if (zone == 1) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, 65, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(18, 35), Math.toRadians(125))
                    .strafeToLinearHeading(new Vector2d(18,50) , Math.toRadians(180))
                    .build());
            sneakyLink.setPosition(.73); //weirdo position la la lala la
            sneakyRink.setPosition(.22);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(-1.5,48 ))// increase x gets farther away from stack
                    .build());
            intake.setPower(-1);
            sleep(1000);



            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo( new Vector2d(18,23))
                    .strafeToLinearHeading(new Vector2d(73,23), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(105,39.5),Math.toRadians(180))
                    .build());
            intake.setPower(1);
            sleep(400);
            intake.setPower(0);
            sleep((100));
            backLeg.setPosition(.9);
            frontLeg.setPosition(.7);
            sleep(400);
            rightWinch.setTargetPosition(-1100);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-1100);
            leftWinch.setPower(1);
            sleep(100);
            arch.setPosition(.363);
            sleep(100);
            hips.setPosition(.95);
            sleep(400);

                  //  .strafeToLinearHeading(new Vector2d(8, 30), Math.toRadians(180))
                    //.strafeToConstantHeading(new Vector2d(10, 44.33))
                    //.strafeToConstantHeading(new Vector2d(13,36))


        } else if (zone == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, 65, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(13,35))
                    .strafeToConstantHeading(new Vector2d(28,42))
                            .turnTo(Math.toRadians(180))
                    .build());
            sneakyLink.setPosition(.73); //weirdo position la la lala la
            sneakyRink.setPosition(.22);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(24.5,48 ))// increase x gets farther away from stack
                    .build());
            //sneakyLink.setPosition(.5);
            //sneakyRink.setPosition(.5);
 //           sleep(350);
            intake.setPower(-1);
            sleep(400);
//            intake.setPower(-1);
//            sleep(400);

//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//            .strafeTo( new Vector2d(28,48))
//                    .build());
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo( new Vector2d(28,48))
                    .strafeToConstantHeading(new Vector2d(40,24))
                    .strafeToLinearHeading(new Vector2d(100,24),Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(132,49),Math.toRadians(180))
                    .afterTime(2, new Action(){

                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            intake.setPower(0);


                            return false;
                        }
                    }).afterTime(3, new Action(){

                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                           // intake.setPower(-1);
                            sleep((100));
                            intake.setPower(0);
                            backLeg.setPosition(.9);
                            frontLeg.setPosition(.7);
                            sleep(400);
                            rightWinch.setTargetPosition(-800);
                            rightWinch.setPower(1);
                            leftWinch.setTargetPosition(-800);
                            leftWinch.setPower(1);
                            sleep(100);
                            arch.setPosition(.363);
                            sleep(100);
                            hips.setPosition(.564);
                            sleep(100);
                            return false;
                        }
                    })
                    .build());
            //intake.setPower(0);


        } else {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(20, 38))
                    // .strafeToLinearHeading(new Vector2d(14, 38), Math.toRadians(70))
                    // increase x gets farther away from stack
                    .build());
        }
        drive.updatePoseEstimate();
        telemetry.addLine("Pose" + drive.pose.position);
        telemetry.addLine("HEading" + Math.toDegrees(drive.pose.heading.log()));
        telemetry.update();


        //Drive to Backdrop

        if (zone == 1) {
            backLeg.setPosition(.7);  // .9 is closed pos
            sleep(200);
            frontLeg.setPosition(.46);
            sleep(200);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(98,37))
                    .turnTo(Math.toRadians(270))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(50);
            hips.setPosition(.18);
            sleep(400);
            arch.setPosition(.69);
            sleep(50);
            rightWinch.setTargetPosition(-10);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-10);
            leftWinch.setPower(1);
            hips.setPosition(.27);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sneakyLink.setPosition(.58); //weirdo position la la lala la
            sneakyRink.setPosition(.42);

        } else if (zone == 2) {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToConstantHeading(new Vector2d(25,24))
//                    .strafeToLinearHeading(new Vector2d(100,24), Math.toRadians(180))
//                    .build());

//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(new Vector2d(133,48.5),Math.toRadians(180))
//                    .build());
            backLeg.setPosition(.7);  // .9 is closed pos
            sleep(200);
            frontLeg.setPosition(.46);
            sleep(200);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(130,47.5))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(50);
            hips.setPosition(.18);
            sleep(200);
            arch.setPosition(.69);
            sleep(50);
            rightWinch.setTargetPosition(-10);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-10);
            leftWinch.setPower(1);
            hips.setPosition(.27);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sneakyLink.setPosition(.58); //weirdo position la la lala la
            sneakyRink.setPosition(.42);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(100,25.5), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(40,25.5), Math.toRadians(180))
                    .build());

//            sneakyLink.setPosition(.72); //weirdo position la la lala la
//            sneakyRink.setPosition(.28);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(22,49),Math.toRadians(180))//look here
                  //  .strafeToLinearHeading(new Vector2d(20,50),Math.toRadians(180))
                    .build());
            intake.setPower(-1);
            sleep(300);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    //.strafeTo( new Vector2d(28,48))
                    .strafeToConstantHeading(new Vector2d(40,25))
                    .strafeToLinearHeading(new Vector2d(100,25), Math.toRadians(180)).strafeToLinearHeading(new Vector2d(131,46.5),Math.toRadians(180)).afterTime(.1, new Action(){
//i hate this
                        // please help me
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            intake.setPower(1);
                            return false;
                        }
                    }).afterTime(.5, new Action(){

                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            intake.setPower(-1);
                            sleep((100));
                            intake.setPower(0);
                            backLeg.setPosition(.9);
                            frontLeg.setPosition(.7);
                            sleep(100);
                            intake.setPower(0);
                            rightWinch.setTargetPosition(-1000);
                            rightWinch.setPower(1);
                            leftWinch.setTargetPosition(-1000);
                            leftWinch.setPower(1);
                            sleep(100);
                            arch.setPosition(.363);
                            sleep(100);
                          //  hips.setPosition(.564);
                            sleep(100);
                            return false;
                        }
                    })
                    .build());


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

        } else Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(37, 28))
                .turnTo(Math.toRadians(180))
                                .strafeToConstantHeading(new Vector2d(28, 28))
                //.strafeToLinearHeading(new Vector2d(37, 36), Math.toRadians(180))
                .build());
        drive.updatePoseEstimate();
        telemetry.addLine("Pose" + drive.pose.position);
        telemetry.addLine("HEading" + Math.toDegrees(drive.pose.heading.log()));
        telemetry.update();


        if(zone == 1){
            Actions.runBlocking(drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(95,36))
            .build());
        }
        else if( zone == 2){
            backLeg.setPosition(.7);  // .9 is closed pos
            sleep(200);
            frontLeg.setPosition(.46);
            sleep(200);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(130,49.5))
                    .build());
            backLeg.setPosition(.85);  // .9 is closed pos
            sleep(50);
            frontLeg.setPosition(.6);
            sleep(50);
              hips.setPosition(.18);
             sleep(200);
            arch.setPosition(.69);
            sleep(50);
            rightWinch.setTargetPosition(-20);
            rightWinch.setPower(1);
            leftWinch.setTargetPosition(-20);
            leftWinch.setPower(1);
             hips.setPosition(.27);
            backLeg.setPosition(.76);  // .9 is closed pos
            frontLeg.setPosition(.52);
            sneakyLink.setPosition(.68); //weirdo position la la lala la
            sneakyRink.setPosition(.32);
            sleep(300);
        }
        else{

 }
//
//        //PARK
     Actions.runBlocking(drive.actionBuilder(drive.pose)
//                .strafeTo(new Vector2d(123, 40))
////                .turnTo(Math.toRadians(270))
////                .strafeTo(new Vector2d(125, 35))
            .build());
//
     }
//










}


