package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
@Disabled
public class GyroOperator extends LinearOpMode {
    static final double HEADING_THRESHOLD = 3;      // As tight as e can make it with an integer gyro
    static final double P_DRIVE_COEFF = 0.01;     // Larger is more responsive, but also less stable
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public double Xmovement = 0;
    public double Ymovement = 0;

    enum DriveMode {
        normal,
        left,
        right;
    }

    // liot test vofuria eithout robot
    Hardware robot = new Hardware();


    @Override
    public void runOpMode() {
    }



    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turnInput.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turnInput power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setDriveMotorsPower(leftSpeed, Hardware.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(rightSpeed, Hardware.DRIVE_MOTOR_TYPES.RIGHT);


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turnInput LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetGyroAngle();
        while (opModeIsActive() && robotError > 180) robotError -= 360;
        while (opModeIsActive() && robotError <= -180) robotError += 360;
        return robotError;
    }
    public void delay(double millis){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && millis > runtime.milliseconds()){
        }
    }
    public double getDriveSpeed(double target,double speed){

        double cor = abs(robot.getYEncoder()) / abs(target);//getDriveSpeed(this.robot.leftCollector,speed);
        cor = 1 - cor;
        cor *= 2;
        if(cor < 0.1){
            cor = 0.1;
        }

        return cor;

    }
    public void gyroTurn (  double speed, double angle) {
        robot.driveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Log.d("apollo","turn : "+ "start - " + robot.GetGyroAngle());
        Log.d("apollo","turn : "+ "to - " + angle);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.update();
        }
        Log.d("apollo","turn : "+ "end - " + robot.GetGyroAngle());
    }
    public void gyroTurnFaound (  double speed, double angle) {
        robot.driveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Log.d("apollo","turn : "+ "start - " + robot.GetGyroAngle());
        Log.d("apollo","turn : "+ "to - " + angle);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
//            int prev = (int) robot.GetGyroAngle();
//            for(int i = 0; i < 70; i++){
//                if(prev == robot.GetGyroAngle()){
//                }
//            }
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        Log.d("apollo","turn : "+ "end - " + robot.GetGyroAngle());
    }
    public void pidTurn(double speed,double angle){
        Log.d("turn","to - " + angle);
        Log.d("turn","start - " + robot.GetGyroAngle());
        PIDController pid = new PIDController(0.02,0,0);
        pid.setSetpoint(angle);
        pid.setOutputRange(-speed, speed);
        pid.setInputRange(-180, 180);
        pid.enable();
        double power;
        double error = angle - robot.GetGyroAngle();
        while(opModeIsActive() && abs(error) > 2){
            power = pid.performPID(robot.GetGyroAngle());
            power *= -error/abs(error);
            robot.setDriveMotorsPower(-power, Hardware.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(power, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
            error = angle - robot.GetGyroAngle();
            telemetry.addData("eror", error);
            telemetry.update();
        }
        Log.d("turn","end - " + robot.GetGyroAngle());
        robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);

            telemetry.addData("eror", error);
            telemetry.update();

        Log.d("YAIR","1");
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void pidDrive(double speed, double distance,double angle,DriveMode mode,int dir){ //put all the void in mahberet
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double steer;
        double error;
        double leftSpeed;
        double rightSpeed;

        PIDController pidDrive;
        PIDController[] pidDriveEncoder = new PIDController[2];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            this.robot.driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.robot.driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.robot.driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            boolean leftOnTarget = false;
            boolean rightOnTarget = false;
            pidDrive = new PIDController(0.1,0.03,0);

            pidDriveEncoder[0] = new PIDController(0.1,0.03,0);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (abs(distance) * COUNTS_PER_INCH);
            newLeftTarget = this.robot.driveLeftBack.getCurrentPosition() + moveCounts;
            newRightTarget = this.robot.driveRightBack.getCurrentPosition() + moveCounts;

            pidDrive.setSetpoint(angle);
            pidDrive.setOutputRange(0, speed);
            pidDrive.setInputRange(-180, 180);
            pidDrive.enable();
            Log.d("YAIR","1");


            pidDriveEncoder[0].setSetpoint(newLeftTarget);
            pidDriveEncoder[0].setOutputRange(-speed, speed);
            pidDriveEncoder[0].setInputRange(0,newLeftTarget);
            pidDriveEncoder[0].enable();
            Log.d("YAIR","2");

            pidDriveEncoder[1] = pidDriveEncoder[0];
            pidDriveEncoder[1].setInputRange(0,newRightTarget);
            pidDriveEncoder[1].setSetpoint(newRightTarget);
            Log.d("YAIR","3");



            //set targets
            this.robot.driveLeftBack.setTargetPosition(newLeftTarget);
            this.robot.driveRightBack.setTargetPosition(newRightTarget);
            Log.d("YAIR","4");


            if (motorTarget(this.robot.driveLeftBack) || motorTarget(this.robot.driveRightBack)){
                if (opModeIsActive()){
                    Log.d("YAIR","13");
                }
            }
            Log.d("apollo","drive angle - " + "to : " + angle);
            Log.d("apollo","drive angle - " + "start : " + robot.GetGyroAngle());
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !rightOnTarget && !leftOnTarget) {
                rightOnTarget = (motorTarget(this.robot.driveRightBack));
                leftOnTarget = (motorTarget(this.robot.driveLeftBack));
                Log.d("YAIR","5");

                error = getError(angle);
                steer = getSteer(error, 0.02);
                leftSpeed = (distance/abs(distance)) * pidDriveEncoder[0].performPID(abs(robot.driveLeftBack.getCurrentPosition()));
                rightSpeed = (distance/abs(distance)) * pidDriveEncoder[1].performPID(abs(robot.driveRightBack.getCurrentPosition()));

                Log.d("YAIR","6");


                switch (mode) {
                    case left:
                        robot.driveLeftFront.setPower(-rightSpeed - steer);
                        robot.driveLeftBack.setPower(leftSpeed - steer);

                        robot.driveRightFront.setPower(rightSpeed + steer);
                        robot.driveRightBack.setPower(-leftSpeed + steer);
                        telemetry.addData("mode",mode);
                        break;
                    case right:
                        robot.driveLeftFront.setPower(leftSpeed - steer);
                        robot.driveLeftBack.setPower(-rightSpeed - steer);

                        robot.driveRightFront.setPower(-leftSpeed + steer);
                        robot.driveRightBack.setPower(rightSpeed + steer);
                        break;

                    default:

                        this.robot.setDriveMotorsPower(leftSpeed - steer, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                        this.robot.setDriveMotorsPower(rightSpeed + steer, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                        break;

                }
                Log.d("YAIR","7");

                // Display drive status for the driver.
                telemetry.addData("Err/St", steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", this.robot.driveLeftBack.getCurrentPosition(),
                        this.robot.driveRightBack.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.addData("leftOnTarget =>", leftOnTarget);
                telemetry.addData("rightOnTarget =>", rightOnTarget);
                telemetry.update();
            }

            // Stop all motion;
            Log.d("YAIR","8");

            this.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);
            Log.d("apollo","drive angle - " + "end : " + robot.GetGyroAngle());

            pidDriveEncoder[0].disable();
            pidDriveEncoder[0].disable();

        }
        Log.d("YAIR","9");
    }


    public void pidDriveHighSpeed(double s ,double distance, double angle, DriveMode mode, double dir){ //put all the void in mahberet
        int newTarget;
        double speed = 1;
        double steer;
        double error;
        double leftSpeed;
        double rightSpeed;

        PIDController pidDrive;
        PIDController pidDriveEncoder = new PIDController(0.2,0,0.01);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            this.robot.leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.robot.leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            boolean OnTarget = false;
            pidDrive = new PIDController(0.2,0.1,0);

            // Determine new target position, and pass to motor controller
            Log.d("e","start : " +  String.valueOf(robot.getYEncoder()));
            newTarget = (int) (this.robot.getYEncoder() + abs(distance));

            pidDrive.setSetpoint(angle);
            pidDrive.setOutputRange(0, speed);
            pidDrive.setInputRange(-180, 180);
            pidDrive.enable();
            Log.d("YAIR","1");


            pidDriveEncoder.setSetpoint((newTarget * robot.CuiToIn));
            pidDriveEncoder.setOutputRange(0, speed);
            pidDriveEncoder.setInputRange(0,(newTarget * robot.CuiToIn));
            pidDriveEncoder.enable();



            //set targets
            Log.d("YAIR","4");

            this.robot.leftCollector.setTargetPosition((int) (newTarget * robot.CuiToIn));
            if (motorTarget(this.robot.leftCollector)){
                if (opModeIsActive()){
                    Log.d("YAIR","13");
                }
            }
            Log.d("apollo","drive angle - " + "to : " + angle);
            Log.d("apollo","drive angle - " + "start : " + robot.GetGyroAngle());
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !OnTarget) {
                OnTarget = (motorTarget(this.robot.leftCollector));
                Log.d("YAIR","5");
                double cor = getDriveSpeed(newTarget,speed);
                error = getError(angle);
                steer = getSteer(error, 0.15 * speed * cor);
                leftSpeed = cor * (distance/abs(distance)) * pidDriveEncoder.performPID(abs(robot.getYEncoder()));
                rightSpeed = cor * (distance/abs(distance)) * pidDriveEncoder.performPID(abs(robot.getYEncoder()));

                Log.d("YAIR","6");


                switch (mode) {
                    case left:
                        robot.driveLeftFront.setPower(-rightSpeed - steer);
                        robot.driveLeftBack.setPower(leftSpeed - steer);

                        robot.driveRightFront.setPower(rightSpeed + steer);
                        robot.driveRightBack.setPower(-leftSpeed + steer);
                        telemetry.addData("mode",mode);
                        break;
                    case right:
                        robot.driveLeftFront.setPower(leftSpeed - steer);
                        robot.driveLeftBack.setPower(-rightSpeed - steer);

                        robot.driveRightFront.setPower(-leftSpeed + steer);
                        robot.driveRightBack.setPower(rightSpeed + steer);
                        break;

                    default:

                        this.robot.setDriveMotorsPower(leftSpeed - steer, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                        this.robot.setDriveMotorsPower(rightSpeed + steer, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                        break;

                }
                Log.d("YAIR","7");

                // Display drive status for the driver.
//                telemetry.addData("Err/St", steer);
//
//                telemetry.addData("Actual", "%7d:%7d", this.robot.driveLeftBack.getCurrentPosition(),
//                        this.robot.driveRightBack.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.addData("OnTarget =>", OnTarget);
                telemetry.addData("TARGET - ",newTarget);
                telemetry.addData("SPEED - ",leftSpeed);
                telemetry.addData("COR - ",cor);
                telemetry.addData("TARGET TICK - ",robot.leftCollector.getTargetPosition());
                telemetry.addData("CURRENT TICK - ",robot.leftCollector.getCurrentPosition());
                telemetry.addData("CURRENT - ",robot.leftCollector.getCurrentPosition() / robot.CuiToIn);
                telemetry.update();
            }

            // Stop all motion;
            Log.d("YAIR","8");
            if(abs(angle) < 25){
                Xmovement += robot.leftCollector.getCurrentPosition() / robot.CuiToIn;
            }else{
                Ymovement += robot.leftCollector.getCurrentPosition() / robot.CuiToIn;
            }
            this.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);
            Log.d("movement", String.valueOf(Xmovement));
            Log.d("apollo","drive angle - " + "end : " + robot.GetGyroAngle());

            pidDriveEncoder.disable();

        }
        Log.d("YAIR","9");
    }

    public void pidDriveLowSpeed(double s,double distance,double angle,DriveMode mode,double dir){ //put all the void in mahberet
        int newTarget;
        double speed = 0.1;
        double steer;
        double error;
        double leftSpeed;
        double rightSpeed;

        PIDController pidDrive;
        PIDController pidDriveEncoder = new PIDController(0.2,0,0.01);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            this.robot.leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.robot.leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            boolean OnTarget = false;
            pidDrive = new PIDController(0.2,0.1,0);

            // Determine new target position, and pass to motor controller
            Log.d("e","start : " +  String.valueOf(robot.getYEncoder()));
            newTarget = (int) (this.robot.getYEncoder() + abs(distance));

            pidDrive.setSetpoint(angle);
            pidDrive.setOutputRange(0, speed);
            pidDrive.setInputRange(-180, 180);
            pidDrive.enable();
            Log.d("YAIR","1");


            pidDriveEncoder.setSetpoint((newTarget * robot.CuiToIn));
            pidDriveEncoder.setOutputRange(0, speed);
            pidDriveEncoder.setInputRange(0,(newTarget * robot.CuiToIn));
            pidDriveEncoder.enable();



            //set targets
            Log.d("YAIR","4");

            this.robot.leftCollector.setTargetPosition((int) (newTarget * robot.CuiToIn));
            if (motorTarget(this.robot.leftCollector)){
                if (opModeIsActive()){
                    Log.d("YAIR","13");
                }
            }
            Log.d("apollo","drive angle - " + "to : " + angle);
            Log.d("apollo","drive angle - " + "start : " + robot.GetGyroAngle());
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !OnTarget) {
                OnTarget = (motorTarget(this.robot.leftCollector));
                Log.d("YAIR","5");
                double cor = 2.9 * getDriveSpeed(newTarget,speed);
                error = getError(angle);
                steer = getSteer(error, 0.08 * speed * cor);
                leftSpeed = cor * (distance/abs(distance)) * pidDriveEncoder.performPID(abs(robot.getYEncoder()));
                rightSpeed = cor * (distance/abs(distance)) * pidDriveEncoder.performPID(abs(robot.getYEncoder()));

                Log.d("YAIR","6");


                switch (mode) {
                    case left:
                        robot.driveLeftFront.setPower(-rightSpeed - steer);
                        robot.driveLeftBack.setPower(leftSpeed - steer);

                        robot.driveRightFront.setPower(rightSpeed + steer);
                        robot.driveRightBack.setPower(-leftSpeed + steer);
                        telemetry.addData("mode",mode);
                        break;
                    case right:
                        robot.driveLeftFront.setPower(leftSpeed - steer);
                        robot.driveLeftBack.setPower(-rightSpeed - steer);

                        robot.driveRightFront.setPower(-leftSpeed + steer);
                        robot.driveRightBack.setPower(rightSpeed + steer);
                        break;

                    default:

                        this.robot.setDriveMotorsPower(leftSpeed - steer, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                        this.robot.setDriveMotorsPower(rightSpeed + steer, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                        break;

                }
                Log.d("YAIR","7");

                // Display drive status for the driver.
//                telemetry.addData("Err/St", steer);
//
//                telemetry.addData("Actual", "%7d:%7d", this.robot.driveLeftBack.getCurrentPosition(),
//                        this.robot.driveRightBack.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.addData("OnTarget =>", OnTarget);
                telemetry.addData("TARGET - ",newTarget);
                telemetry.addData("SPEED - ",leftSpeed);
                telemetry.addData("COR - ",cor);
                telemetry.addData("TARGET TICK - ",robot.leftCollector.getTargetPosition());
                telemetry.addData("CURRENT TICK - ",robot.leftCollector.getCurrentPosition());
                telemetry.addData("CURRENT - ",robot.leftCollector.getCurrentPosition() / robot.CuiToIn);
                telemetry.update();
            }

            // Stop all motion;
            Log.d("YAIR","8");

            this.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);
            Log.d("apollo","drive angle - " + "end : " + robot.GetGyroAngle());
            if(abs(angle) < 25){
                Xmovement += robot.leftCollector.getCurrentPosition() / robot.CuiToIn;
            }else{
                Ymovement += robot.verticalElevator.getCurrentPosition() / robot.CuiToIn;
            }
            pidDriveEncoder.disable();

        }
        Log.d("YAIR","9");
    }

    public double pidDriveByCube(double speed,double angle,DriveMode mode) { //put all the void in mahberet

        double steer;
        double leftSpeed;
        double rightSpeed;
        int newTarget;

        PIDController pidDrive;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            this.robot.driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.robot.driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.robot.driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            this.robot.leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.robot.leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pidDrive = new PIDController(.025, 0, 0);


            pidDrive.setSetpoint(angle);
            pidDrive.setOutputRange(0, speed);
            pidDrive.setInputRange(-180, 180);
            pidDrive.enable();


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !robot.hasStone()) {
                steer = pidDrive.performPID(robot.GetGyroAngle());
                leftSpeed = speed;
                rightSpeed = speed;


                switch (mode) {
                    case left:
                        robot.driveLeftFront.setPower(-rightSpeed - steer);
                        robot.driveLeftBack.setPower(leftSpeed - steer);

                        robot.driveRightFront.setPower(rightSpeed + steer);
                        robot.driveRightBack.setPower(-leftSpeed + steer);
                        telemetry.addData("mode", mode);
                        break;
                    case right:
                        robot.driveLeftFront.setPower(leftSpeed - steer);
                        robot.driveLeftBack.setPower(-rightSpeed - steer);

                        robot.driveRightFront.setPower(-leftSpeed + steer);
                        robot.driveRightBack.setPower(rightSpeed + steer);
                        break;

                    default:
                        this.robot.setDriveMotorsPower(leftSpeed - steer, Hardware.DRIVE_MOTOR_TYPES.LEFT);
                        this.robot.setDriveMotorsPower(rightSpeed + steer, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                        break;

                }
                Log.d("YAIR", "7");

                // Display drive status for the driver.
                telemetry.addData("Err/St", steer);
                telemetry.addData("Actual", "%7d:%7d", this.robot.driveLeftBack.getCurrentPosition(),
                        this.robot.driveRightBack.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;

            this.robot.setDriveMotorsPower(0, Hardware.DRIVE_MOTOR_TYPES.ALL);


        }
        return robot.getYEncoder();
    }

    public boolean motorTarget(DcMotor Motor) {
            if (abs(Motor.getCurrentPosition()) < abs(Motor.getTargetPosition())) {
                return false;
            } else {
                return true;
            }

    }

    public void driveByAngle(double speed,double driveAngle){

    }
}


