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

@Autonomous(name = "Autonomous blue romania", group = "apollo")
public  class AutoBlueRomania extends autonum{
    private Thread timer = new timer();
    private Thread outPutCube = new outPutCube();
    private Thread openCollection = new openCollection();
    private Thread position = new position();
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

        openCV.proces( 0);
        waitForStart();
//        outPutCube.start();
//        delay(30000);
        if(!robot.imuIsWorking){
            while (opModeIsActive()){}
        }
        runtime.reset();
        timer.start();
        vu =  openCV.getPosition();
        openCollection.start();
        delay(600);
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

        gyroTurn(0.4,-88);
        robot.setCatchersA();
        double offset = -90 - robot.GetGyroAngle();
        Log.d("angleError", String.valueOf(offset));
        pidDriveHighSpeed(0.6,-60,-90 + (offset * 1.5),normal,1);
        switch (vu){
            case right:
                pidDriveHighSpeed(0.6,-10,-90,normal,1);
                break;
            case center:
                pidDriveHighSpeed(0.6,-5,-90,normal,1);
                break;
        }
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));

        gyroTurn(0.4,180);


        Log.d("apollo", "before catch : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "before catch : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));

        pidDriveLowSpeed(0.25,-(Xmovement-10),robot.GetGyroAngle(),normal,1);

        outPutCube.start();

        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));

        Log.d("apollo", "after catch : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "after catch : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        Log.d("apollo", "after catch : " + "angle - " + robot.GetGyroAngle());

        robot.setCatchers(0);
        delay(500);
        gyroTurn(1 , -160);
        pidDriveHighSpeed(1,15, -150,normal,1);
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        gyroTurn(0.8 , -90);
        robot.setCatchers(1);
        pidDrive(0.4,-45, -90,normal,1);

        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        pidDriveHighSpeed(0.8,23, -92.5,normal,1);
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d(" robot x", String.valueOf((robot.rightCollector.getCurrentPosition() * (2048 * 4) / (2 * Math.PI))));
        Log.d("apollo", "re : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "re : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        delay(500);
        gyroTurn(0.7,-90);
//        while (opModeIsActive()){
//            telemetry.addData("DISTANCE - ",Ymovement);
//            telemetry.update();
//        }
        switch (vu){
            case center:
                pidDriveHighSpeed(0.8,46.5, -80,normal,1);
                break;
            case right:
                pidDriveHighSpeed(0.8,53, -80,normal,1);
                break;
            case left:
                pidDriveHighSpeed(0.8,40, -80,normal,1);
                break;

        }

        robot.leftCollector.setPower(-1);
        robot.rightCollector.setPower(1);
        gyroTurn(0.9,-45);
        driveAngle = -45;
        robot.frontClaw.setPosition(frontOpenPos);
        robot.backClaw.setPosition(backClosePos);
        catchCube2();
        gyroTurn(0.9,-90);
        pidDriveHighSpeed(1,-50,-90,normal,1);
        outPutCube.start();
        pidDrive(0.4,-60,-90,normal,1);
        pidDriveHighSpeed(1,35,-90,normal,1);
    }

    public void newRight(){
        pidDrive(0.4,40,driveAngle,normal,1);

    }
    public void newCenter(){
        pidDrive(0.4,37,2,normal,1);

    }
    public void catchCube(){
        double moved = pidDriveByCube(0.25,driveAngle,normal);
        delay(800);
        robot.setCollectMotorsPower(0);
        Log.d("apollo", "moved : " + String.valueOf(moved));
        Log.d("apollo", "moved : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "moved : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        pidDriveLowSpeed(0.2,(int)(moved),0,normal,1);

    }

    public void catchCube2(){
        double moved = pidDriveByCube(0.17,driveAngle,normal);
        delay(800);
        robot.setCollectMotorsPower(0);
        Log.d("apollo", "moved : " + String.valueOf(moved));
        Log.d("apollo", "moved : " + "left - " + String.valueOf(robot.driveLeftBack.getCurrentPosition()));
        Log.d("apollo", "moved : " + "right - " + String.valueOf(robot.driveRightBack.getCurrentPosition()));
        pidDriveHighSpeed(0.2,(int)(moved),-90,normal,1);

    }
    public void newLeft(){
        pidDrive(0.4,40,driveAngle,normal,1);
    }

    void outCube() throws InterruptedException {
        robot.backClaw.setPosition(backOpenPos);
        robot.frontClaw.setPosition(frontClosePos);
        delay(1800);
        robot.horizontalElevator.setPower(1);
        delay(1800);
        robot.horizontalElevator.setPower(0);
        delay(500);
        robot.horizontalElevator.setPower(-1);
        delay(1400);
        robot.horizontalElevator.setPower(0);
        delay(400);
        robot.frontClaw.setPosition(frontClosePos);

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
                    delay(50);
                }

            } catch (Exception ignored) {
            }
        }
    }
    private class position extends Thread {
        int position = 0;

        position() { setName("position"); }

        @Override
        public void run() {
            try {
                while (opModeIsActive() && !isInterrupted()) {
                    robot.updatePosition(true);
                }

            } catch (InterruptedException ignored) {
            }
        }
    }
    private class outPutCube extends Thread {
        int position = 0;

        outPutCube() { setName("catchers"); }

        @Override
        public void run() {
            try {
                outCube();
            } catch (InterruptedException ignored) {
            }
        }
    }

    private class openCollection extends Thread {
        int position = 0;

        openCollection() { setName("openCollection"); }

        @Override
        public void run() {
            try {
                robot.backClaw.setPosition(backClosePos);
                robot.verticalElevator.setPower(1);
                delay(950);
                robot.verticalElevator.setPower(-1);
                delay(400);
                robot.verticalElevator.setPower(0);
                robot.frontClaw.setPosition(frontOpenPos);
            } catch (Exception ignored) {
            }
        }
    }
}
