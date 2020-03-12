package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.normal;
import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;
import static org.firstinspires.ftc.teamcode.skyStoneVision.stonePosition.right;

@Autonomous(name = "Autonomous red champion 2 Cube", group = "apollo")
@Disabled

public  class AutoBlueChampion2Cube extends autonum{
    private Thread outPutCube = new outPutCube();
    private Thread timer = new timer();
    private ElapsedTime runtime = new ElapsedTime();
    skyStoneVision.stonePosition vu;
    enum stonePositionNewMoves{
        LEFT,
        CENTER,
        RIGHT
    }
    double driveAngle = 0;

    private InternalCameraExample openCV  = new InternalCameraExample();
    public void runOpMode(){
        robot.init(hardwareMap, false);
        robot.led.setPattern(COLOR_WAVES_OCEAN_PALETTE);
        robot.led.displayPattern();
        telemetry.addData("imu is working ==> ", robot.imuIsWorking);

        telemetry.update();

        openCV.initOpenCV(hardwareMap);

        openCV.proces(70);
        waitForStart();

        if(!robot.imuIsWorking){
            while (opModeIsActive()){}
        }
        runtime.reset();
        timer.start();
        vu =  openCV.getPosition();

        robot.backClaw.setPosition(backClosePos);
        robot.verticalElevator.setPower(1);
        sleep(950);
        robot.verticalElevator.setPower(-1);
        sleep(600);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(frontOpenPos);
        switch (vu){
            case right:
                robot.leftCollector.setPower(-0.8);
                robot.rightCollector.setPower(1);
                driveAngle = -10;
                newRight();
                break;
            case left:
                robot.leftCollector.setPower(-1);
                robot.rightCollector.setPower(0.8);
                driveAngle = 10;
                newLeft();
                break;
            case center:
                robot.leftCollector.setPower(-1);
                robot.rightCollector.setPower(1);
                newCenter();
                break;
        }
        catchCube();

        gyroTurn(0.4,90);
        sleep(50);
        robot.setCatchersA();

        pidDriveHighSpeed(0.6,-62,88,normal,1);
        switch (vu){
            case left:
                pidDriveHighSpeed(0.6,-10,88,normal,1);
                break;
            case center:
                pidDriveHighSpeed(0.6,-5,88,normal,1);
                break;
        }
        sleep(50);
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        gyroTurn(0.4,165);
        sleep(50);
        outPutCube.start();

        Log.d("apollo", "before catch : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "before catch : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));

        if(vu == right){
            pidDriveHighSpeed(0.25,-6.5,robot.GetGyroAngle(),normal,1);
        }else{
            pidDriveHighSpeed(0.25,-6,robot.GetGyroAngle(),normal,1);

        }
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));

        Log.d("apollo", "after catch : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "after catch : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        Log.d("apollo", "after catch : " + "angle - " + robot.GetGyroAngle());

        robot.setCatchers(0);
        sleep(1000);
        pidTurn(1 , 160);
        pidDriveHighSpeed(1,6, 150,normal,1);
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        gyroTurn(0.8 , 90);
        robot.setCatchers(1);
        pidDrive(0.40,-45, 90,normal,1);
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        pidDriveHighSpeed(0.8,15, 92,normal,1);
        pidDriveHighSpeed(0.8,25,90,normal,1);
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d("apollo", "re : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "re : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        sleep(500);
        robot.setCatchers(0);
        catchCube2();



    }

    public void newRight(){
        pidDrive(0.3,40,driveAngle,normal,1);

    }
    public void newCenter(){
        pidDrive(0.3,37,2,normal,1);

    }
    public void catchCube2(){
        robot.frontClaw.setPosition(frontOpenPos);
        robot.leftCollector.setPower(-0.8);
        robot.rightCollector.setPower(1);
        switch (vu){

            default:
                pidDriveHighSpeed(0.6,15,95,normal,1);
                break;
        }
        int moved = (int) pidDriveByCube(0.4,35,normal);
        sleep(1000);
        robot.frontClaw.setPosition(frontClosePos);
        sleep(700);
        robot.leftCollector.setPower(1);
        robot.rightCollector.setPower(-1);
        sleep(300);
        robot.leftCollector.setPower(0);
        robot.rightCollector.setPower(0);
        pidDriveHighSpeed(0.6,moved - 4,35,normal,1);
        gyroTurn(0.4,90);
        pidDriveHighSpeed(0.8,-10,90,normal,1);


    }
    public void catchCube(){
        double moved = pidDriveByCube(0.2,driveAngle,normal);
        sleep(800);
        robot.frontClaw.setPosition(frontClosePos);
        robot.setCollectMotorsPower(0);
        Log.d("apollo", "moved : " + String.valueOf(moved));
        Log.d("apollo", "moved : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "moved : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        pidDriveHighSpeed(0.2,(int)(moved+10),0,normal,1);

    }
    public void newLeft(){
        pidDrive(0.3,40,driveAngle,normal,1);
    }
    public void seconedPart(){
        robot.initImu();
        robot.setCollectMotorsPower(1);
        pidDrive(0.8, 65 , -15 ,normal,1);
        sleep(600);
        pidDrive(0.8,-65,0, normal,1);
        outPutCube.start();
        pidDrive(0.8,-100,0,normal,1);
        sleep(300);
        pidDrive(0.9,80,0,normal,1);
    }
    private class outPutCube extends Thread {
        int position = 0;

        outPutCube() { setName("catchers"); }

        @Override
        public void run() {
            try {
                robot.backClaw.setPosition(backOpenPos);
                robot.frontClaw.setPosition(frontOpenPos);
                robot.verticalElevator.setPower(1);
                sleep(300);
                robot.verticalElevator.setPower(-1);
                sleep(300);
                robot.verticalElevator.setPower(0);

                robot.backClaw.setPosition(backClosePos);
                robot.frontClaw.setPosition(frontClosePos);
                robot.horizontalElevator.setPower(1);
                robot.verticalElevator.setPower(0);
                sleep(3400);
                robot.verticalElevator.setPower(0);
                robot.horizontalElevator.setPower(0);
                robot.backClaw.setPosition(backOpenPos);
                robot.frontClaw.setPosition(frontOpenPos);
                sleep(700);
                robot.frontClaw.setPosition(frontClosePos);

                robot.horizontalElevator.setPower(-1);
                sleep(1150);
                robot.horizontalElevator.setPower(0);
                sleep(400);
                robot.backClaw.setPosition(backClosePos);
                Thread.sleep(50);

            } catch (InterruptedException ignored) {
            }
        }
    }

    private class timer extends Thread {
        int position = 0;

        timer() { setName("timer"); }

        @Override
        public void run() {
            try {
                while (opModeIsActive() && !isInterrupted()) {
                    telemetry.addData("time", runtime.seconds());
                    telemetry.addData("pos  - ",vu);
                    telemetry.addData("gyro  - ",robot.GetGyroAngle());
                    telemetry.update();
                    sleep(50);
                }

            } catch (InterruptedException ignored) {
            }
        }
    }
}
