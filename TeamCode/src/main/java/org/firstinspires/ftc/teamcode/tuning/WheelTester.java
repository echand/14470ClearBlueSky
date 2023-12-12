package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp
public class WheelTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            drive.leftFront.setPower(gamepad1.left_stick_y);
            drive.leftBack.setPower(gamepad1.left_stick_x);
            drive.rightFront.setPower(gamepad1.right_stick_y);
            drive.rightBack.setPower(gamepad1.right_stick_x);
        }



    }
}
