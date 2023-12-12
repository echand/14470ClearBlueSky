package org.firstinspires.ftc.teamcode.tuning;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class SampleTestVision extends LinearOpMode{
    int zone;
    public void runOpMode() throws InterruptedException {
        TeamPropDetector.startPropDetection(hardwareMap, telemetry);
        waitForStart();
        zone = TeamPropDetector.getRedPropZone();
        TeamPropDetector.endPropDetection();
    }
}