package org.firstinspires.ftc.teamcode.tuning;



import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "VeerDiagnostic", group = "helper")
public class VeerDiagnostic extends LinearOpMode {

    int curServo = 0;
    Servo cur = null;
    double curServoPosition = 0;
    private boolean prevDUP;
    private boolean prevDDOWN;


    @Override
    public void runOpMode() throws InterruptedException {

        cur = hardwareMap.servo.get("arch");


        telemetry.addLine(" Position: " + round(cur.getPosition()));
        telemetry.update();
        waitForStart();


        while(!isStopRequested() && opModeIsActive()){


            telemetry.addLine("---Position: " + cur.getPosition());

            prevDUP = gamepad1.dpad_up;
            prevDDOWN = gamepad1.dpad_down;

            //MOVING SERVO
            curServoPosition += gamepad1.left_stick_y*.0005;
            curServoPosition = MathUtils.clamp(curServoPosition, 0, 1);
            cur.setPosition(curServoPosition);
            telemetry.addLine("TARGET: " + curServoPosition);
            telemetry.update();

            }

        }

    private double round(double t){
        return ((int)t*100)/100.0;

    }





}