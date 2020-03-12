
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.MathFunctions.moveX;
import static org.firstinspires.ftc.teamcode.MathFunctions.moveY;

@Disabled
public class Hardware {
    DcMotor driveLeftBack = null;
    DcMotor driveRightBack = null;
    DcMotor driveLeftFront = null;
    DcMotor driveRightFront = null;

    DcMotor rightCollector = null;
    DcMotor leftCollector = null;

    Servo frontClaw = null;
    Servo backClaw = null;

    Servo capStone = null;
    private Servo rightCatcher = null;
    private Servo leftCatcher = null;
    double angle;

    DcMotor verticalElevator = null;
    DcMotor horizontalElevator = null;
    RevBlinkinLedDriver led = new RevBlinkinLedDriver();

    BNO055IMU imu;
    NormalizedColorSensor colorSensor;

    public static double frontClosePos = 1;
    public static double frontOpenPos =  0.25;

    public static double backClosePos =0;
    public static double  backOpenPos = 0.9;

    private static final double countsPer20gearmotor = 1120;
    static final double DRIVE_GEAR_REDUCTION = 2.4;
    private static final double COUNTS_PER_INCH_FOR_VERTICAL_SLIDE = (countsPer20gearmotor / DRIVE_GEAR_REDUCTION) / (Math.PI);
    private int[] levels = {0,6,10,15,18,23};
    boolean verticalMotorOnTarget = true;
    boolean imuIsWorking = false;

    Point previousEncoderPosition = new Point();
    Point move = new Point();
    Point robotPosition = new Point();
    Point currentEncoderPosition = new Point();
    Point initEncoderPosition;

    private double s4tPPR = 3200;
    private double s4tGear = 3;
    private double s4tWheelDiameter = 2;

    public double CuiToIn = (2048 * 4) / (s4tWheelDiameter * Math.PI);
    private double motorToIn = (537.6) / (4 * Math.PI);


    Point point() {
        return robotPosition;
    }

    public enum DRIVE_MOTOR_TYPES {
        LEFT,
        RIGHT,
        FRONT,
        BACK,
        SIDE_WAYS,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT,
        ALL
    }


    float GetGyroAngle(){

        Orientation angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap,boolean teleop) {
        /* local OpMode members. */
        led.initialize(ahwMap);
        imu = ahwMap.get(BNO055IMU.class, "imu 1");
        imuIsWorking = initImu();
        if (!imuIsWorking){
            imu = ahwMap.get(BNO055IMU.class , " imu");
            imuIsWorking = initImu();
        }
        colorSensor = ahwMap.get(NormalizedColorSensor.class, "cs");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        driveLeftBack = ahwMap.get(DcMotor.class, "dlb");
        driveRightBack = ahwMap.get(DcMotor.class, "drb");
        driveRightFront = ahwMap.get(DcMotor.class, "drf");
        driveLeftFront = ahwMap.get(DcMotor.class, "dlf");

        rightCollector = ahwMap.get(DcMotor.class, "rc");
        leftCollector = ahwMap.get(DcMotor.class, "lc");

        verticalElevator = ahwMap.get(DcMotor.class, "eu");
        horizontalElevator = ahwMap.get(DcMotor.class, "es");

        rightCatcher = ahwMap.get(Servo.class, "rca");
        leftCatcher = ahwMap.get(Servo.class, "lca");

        capStone = ahwMap.get(Servo.class , "ca");

        frontClaw = ahwMap.get(Servo.class,"fc");
        backClaw = ahwMap.get(Servo.class,"bc" );


        driveRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);



        verticalElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        driveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(!teleop) {


            driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            driveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        verticalElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        driveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initEncoderPosition = new Point(0, 0);
        previousEncoderPosition = initEncoderPosition;
        capStone.setPosition(0);
        led.setPattern(COLOR_WAVES_LAVA_PALETTE);

    }


    void setDriveMotorsPower(double power, DRIVE_MOTOR_TYPES driverMotorType){
        switch (driverMotorType){
            case LEFT:

                driveLeftFront.setPower(power);
                driveLeftBack.setPower(power);
                break;
            case RIGHT:

                driveRightBack.setPower(power);
                driveRightFront.setPower(power);
                break;
            case SIDE_WAYS:

                driveRightBack.setPower(power);
                driveRightFront.setPower(-power);
                driveLeftFront.setPower(power);
                driveLeftBack.setPower(-power);
                break;
            case DIAGONAL_LEFT:
                driveRightFront.setPower(power);
                driveLeftBack.setPower(power);
                break;
            case DIAGONAL_RIGHT:

                driveLeftFront.setPower(power);
                driveRightBack.setPower(power);

                break;
            case BACK:
                driveLeftBack.setPower(power);
                driveRightBack.setPower(power);
                break;

            case FRONT:
                driveLeftFront.setPower(power);
                driveRightFront.setPower(power);
                break;
            case ALL:
            default:
                //  driveLeftFront.setVelocity(power);
                //  driveLeftBack.setVelocity(power);
                //  driveRightFront.setVelocity(power);
//               driveRightBack.setVelocity(power);

                driveLeftFront.setPower(power);
                driveLeftBack.setPower(power);
                driveRightBack.setPower(power);
                driveRightFront.setPower(power);
                break;
        }
    }

    void setCollectMotorsPower(double power){
        rightCollector.setPower(power);
        leftCollector.setPower(-power);
    }

    void setCatchers(double position){
        if(position == 1){
            leftCatcher.setPosition(0.9);
            rightCatcher.setPosition(0.3);
        }else{
            leftCatcher.setPosition(0.6);
            rightCatcher.setPosition(0);
        }
        try {
            sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }
    void setCatchersA(){
        leftCatcher.setPosition(0.6);
        rightCatcher.setPosition(0.6);
        try {
            sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
    public double getX(){
        return point().x;
    }
    public double getY()
    {
        return point().y;
    }
    public boolean initImu(){
        boolean sucses;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        sucses = imu.initialize(parameters);
        return sucses;


    }
    public double getYEncoder(){
        return leftCollector.getCurrentPosition() / CuiToIn;
    }
    public double getXEncoder(){ return verticalElevator.getCurrentPosition() / CuiToIn; }

    public void updatePosition(boolean blue) throws InterruptedException {


        currentEncoderPosition = new Point(abs(getXEncoder()), abs(getYEncoder()));

        move.y = currentEncoderPosition.x - previousEncoderPosition.x;
        move.x = currentEncoderPosition.y - previousEncoderPosition.y;

        angle = Math.toRadians(GetGyroAngle());
        robotPosition.x += (cos(angle) * move.x) + (sin(angle) * move.y);
        robotPosition.y += (cos(angle) * move.y) + (sin(angle) * move.x);

/*
       // move.y *= -1;
        double hypotenuse = Math.hypot(move.x, move.y);
        if(move.x != 0){
            hypotenuse *= (move.x/abs(move.x));
        }if(move.y != 0){
            hypotenuse *= (move.y/abs(move.y));
        }
        if(blue){
            robotPosition.x -= (moveX(finalAngle, hypotenuse) + initEncoderPosition.x);
        }
        else{
            robotPosition.x += (moveX(finalAngle, hypotenuse) + initEncoderPosition.x);
        }

        robotPosition.y -=(moveY(finalAngle, hypotenuse) + initEncoderPosition.y);
*/
        previousEncoderPosition = currentEncoderPosition;
    }

    public void AutoVerticalSlide(int level, boolean setTarget){
        if (setTarget) {
            verticalMotorOnTarget = false;
            verticalElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            verticalElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalElevator.setTargetPosition((int) COUNTS_PER_INCH_FOR_VERTICAL_SLIDE * levels[level]);
            verticalElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (verticalElevator.isBusy() && !verticalMotorOnTarget) {
            verticalElevator.setPower(0.9);
        } else {
            verticalMotorOnTarget = true;
            verticalElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verticalElevator.setPower(0);
        }
    }
    public float[] getColorHsv(){
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /** Use telemetry to display feedback on the driver station. We show the conversion
         * of the colors to hue, saturation and value, and display the the normalized values
         * as returned from the sensor.
         * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues;
    }
    public boolean hasStone(){
        if(getColorHsv()[0] > 35){
            return true;
        }else{
            return false;
        }
    }
}


