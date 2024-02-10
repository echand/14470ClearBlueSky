package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous
public class turntest extends LinearOpMode {
    MecanumDrive drive;
    @Override
    public void runOpMode () throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12.00, -63, Math.toRadians(270)));
        drive.setTelemetry(telemetry);

        waitForStart();
        Actions.runBlocking(drive.actionBuilder(new Pose2d(12.00, -63, Math.toRadians(270)))
                .turn(Math.toRadians(180))
                .build());


    }









}