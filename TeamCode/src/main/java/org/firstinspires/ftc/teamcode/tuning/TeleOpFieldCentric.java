package org.firstinspires.ftc.teamcode.tuning;


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


import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class TeleOpFieldCentric extends LinearOpMode{


    DcMotor leftWinch;
    DcMotor rightWinch;

    DcMotor shooter;

    Servo hips;
    Servo arch;
    Servo frontLeg;
    Servo backLeg;

    Servo sneakyLink; // facing robot
    Servo sneakyRink;



    public static final double fullFlip180 = .95;
    public static final double horizantal = .564;
    public static final double rest = .18;
    public static final double susPixelDrop = .357;

    public static final double slapperDeposit = .363;
    public static final double slapperUpForClimb = 1;
    public static final double slapperRest=.717;



    DcMotor intake;
    boolean prevDUP;


    Thread driveThread = null;
    volatile boolean volStop = false;
    volatile MecanumDrive drive;

    public void sliderRunTo(double leftPos, double rightPos) {
        leftWinch.setTargetPosition((int) -leftPos);
        leftWinch.setPower(0.8);
        rightWinch.setTargetPosition((int) -rightPos);
        rightWinch.setPower(0.8);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");
        hips = hardwareMap.servo.get("hips");
        arch = hardwareMap.servo.get("arch");
        frontLeg = hardwareMap.servo.get("frontLeg");
        backLeg = hardwareMap.servo.get("backLeg");
        sneakyLink =hardwareMap.servo.get("sneakyLink");
        sneakyRink = hardwareMap.servo.get("sneakyRink");



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




        hips.setPosition(.27);
        arch.setPosition(.972);
        backLeg.setPosition(.77);  // .9 is closed pos
        frontLeg.setPosition(.52); //  .6 is closed pos
        sneakyLink.setPosition(1); //  up from 0
        sneakyRink.setPosition(0); // down from 0

        double leftSlider = 0;
        double rightSlider = 0;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();
        driveThread = new Thread(this::runDriveLoop);

        driveThread.start();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            double sliderState = gamepad2.right_trigger-gamepad2.left_trigger;

            if(leftSlider < 0 || rightSlider < 0){
                leftSlider = 0;
                rightSlider = 0;
            }

            if (gamepad2.a) {
                backLeg.setPosition(.85);  // .9 is closed pos
                sleep(50);
                frontLeg.setPosition(.6);
                sleep(50);
                hips.setPosition(.18);
                sleep(200);
                arch.setPosition(.771);
                sleep(50);
                leftSlider = 7;
                rightSlider = 7;
                sleep(200);
                hips.setPosition(.27);
                sleep(50);
                backLeg.setPosition(.76);  // .9 is closed pos
                sleep(50);
                frontLeg.setPosition(.52);

// test

            }
//            if (gamepad2.y) {
//                arch.setPosition(.363);
//            }
            telemetry.addLine("IMU: " + Math.toDegrees(drive.getHeading()));
            if(gamepad2.x){
                shooter.setPower(-1);
            }
            else{
                shooter.setPower(0);
            }

            if(gamepad1.right_bumper){
                sneakyLink.setPosition(.62);
                sneakyRink.setPosition(.38);
            }
            if(gamepad1.dpad_down){
                sneakyLink.setPosition(.7);
                sneakyRink.setPosition(.3);
            }
            if(gamepad1.left_bumper){
                sneakyLink.setPosition(1);
                sneakyRink.setPosition(0);
            }
            if(gamepad2.right_bumper){
                backLeg.setPosition(.9);
                frontLeg.setPosition(.7);
                sleep(30);
                leftSlider =750;
                rightSlider = 750;
                sleep(50);
                arch.setPosition(.771);
//                sleep(200);
//                hips.setPosition(.564);

            }
            if(gamepad2.left_bumper){
                backLeg.setPosition(.62);  // .9 is closed pos
                frontLeg.setPosition(.4);
            }
            if(gamepad2.dpad_down){
                hips.setPosition(.564);  // hor
            }
            if(gamepad2.dpad_right){
                hips.setPosition(.357); // diag
            }
            if(gamepad2.dpad_left){
                hips.setPosition(.95);
            }
            if(gamepad2.y){
                rightSlider = 3000;
                leftSlider = 3000;
            }
            if(gamepad2.left_stick_button){
                leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightWinch.setTargetPosition(0);
                leftWinch.setTargetPosition(0);
//        chainBar.setTargetPosition(ARM_0);
                rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_right){
                sneakyLink.setPosition(.79); //weirdo position la la lala la
                sneakyRink.setPosition(.21);
            }


//            if(gamepad2.dpad_up){
//                leftWinch.setTargetPosition(-2000);
//                rightWinch.setTargetPosition(-2000);
//                leftWinch.setPower(0.3);
//                rightWinch.setPower(0.3);
//            }



            if(leftWinch.getCurrentPosition() < -3100 || rightWinch.getCurrentPosition() < -3100) {
                sliderRunTo(-3100, -3100);
            }
//
             if (Math.abs(sliderState) > .2){
                if(leftSlider != rightSlider){
                    leftSlider = Math.max(leftSlider,rightSlider);
                }
                rightSlider = leftSlider;
//                leftSlider -= sliderState * 5;
//                rightSlider = leftSlider;
                 leftSlider += sliderState*28;
                 rightSlider += sliderState*28;


            }
            sliderRunTo(leftSlider, rightSlider);


            double intakeMotorPower = gamepad1.right_trigger-gamepad1.left_trigger;

            intake.setPower(-intakeMotorPower);




//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));




            telemetry.addData("Rslider", rightWinch.getCurrentPosition());
            telemetry.addData("Lslider" ,leftWinch.getCurrentPosition());
            telemetry.addData("Lsliderstate" ,leftSlider);
            telemetry.addData("Rsliderstate" ,rightSlider);


            telemetry.update();
            // Update everything. Odometry. Etc.

        }
        volStop = true;
        driveThread.join();








    }


    public void runDriveLoop(){

        while(!volStop){
            drive.driveFieldCentric(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
            if(gamepad1.dpad_up && gamepad1.dpad_up!=prevDUP){
                drive.resetHeading();
            }
            prevDUP = gamepad1.dpad_up;

            drive.updatePoseEstimate();
        }


    }



}
