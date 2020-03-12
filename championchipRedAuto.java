package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CurvePoint;
import org.firstinspires.ftc.teamcode.RobotMovement;

import java.util.ArrayList;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "RED")
@Disabled

public class championchipRedAuto extends LinearOpMode {
    private RobotMovement robotMovement = new RobotMovement();
    ArrayList<CurvePoint> pathCurrent = new ArrayList();
    ArrayList<CurvePoint> path = new ArrayList();
    int level = 0;
    private Thread verticalSlide = new verticalSlide();


    public void runOpMode(){
        robotMovement.robot.init(hardwareMap,false);
        path.add(new CurvePoint(0,0,0.2,0.5,0.5,Math.toRadians(90),1.0));
        path.add(new CurvePoint(10,10,0.2,0.5,0.5,Math.toRadians(90),1.0));
        //path.add(new CurvePoint(40,22,0.7,0.5,0.5,Math.toRadians(90),1.0));
        //path.add(new CurvePoint(20,12,0.7,0.5,0.5,Math.toRadians(90),1.0));

        //path.add(new CurvePoint(20,80,0.7,0.5,0.5,Math.toRadians(180),1.0));
        //path.add(new CurvePoint(40,80,0.7,0.5,0.5,Math.toRadians(180),1.0));

        telemetry.addData("finish","wating to start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            try {
                robotMovement.robot.updatePosition(true);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            telemetry.addData("opMose","is active");
            telemetry.addData("bot X =>", robotMovement.robot.robotPosition.x);
            telemetry.addData("bot Y =>", robotMovement.robot.robotPosition.y);

            telemetry.addData("error",robotMovement.error);
            telemetry.addData("steer",robotMovement.steer);
            telemetry.addData("pathCurrent.size()",pathCurrent.size());

            robotMovement.followCurve(path);
            telemetry.addData("followMe:", "X => " + robotMovement.followMe.x);
            telemetry.addData("followMe:", "Y => " + robotMovement.followMe.y);

            telemetry.update();

        }
    }

    private class verticalSlide extends Thread{
        verticalSlide() {
            this.setName("verticalSlide");
        }

        @Override
        public void run() {
            try {
                while(!isInterrupted() && opModeIsActive()) {
                    robotMovement.robot.AutoVerticalSlide(level, true);
                    Thread.sleep(50);
                }
            } catch (Exception ignored) {
            }
        }
    }

    private class updateEncoderPosition extends Thread{
        updateEncoderPosition() {
            this.setName("updateEncoderPosition");
        }

        @Override
        public void run() {
            try {
                while(!isInterrupted() && opModeIsActive()) {
                    Thread.sleep(50);
                }
            } catch (Exception ignored) {
            }
        }
    }
}
