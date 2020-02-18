package org.firstinspires.ftc.teamcode;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.ContentResolver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.IntentSender;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.database.DatabaseErrorHandler;
import android.database.sqlite.SQLiteDatabase;
import android.graphics.Bitmap;
import android.graphics.drawable.Drawable;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.UserHandle;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.view.Display;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.DriveTrain;
import org.firstinspires.ftc.teamcode.Autonomous.DriveTrainVel;
import org.firstinspires.ftc.teamcode.Hardware.*;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class Robot {
    public HardwareMap hardwareMap;
    public DriveTrainVel driveTrain;
    public Intake intake;
    public FoundationArms foundationArms;
    public Lift lift;
    public Servos s;
    public Sensors c;
    public IMU imu;
    public RobotMotors robotMotors;


    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AfmBbcz/////AAAAGbLGg++zzk4MiOrcPTc3t9xQj3QHfISJprebOgt5JJ4+83xtFO+ApGlI3GVY/aMgCpoGEIzaJse9sXiYDiLYpJQlGDX765tWJUrqM+pzqLxVXjWA1J6c968/YqYq74Vq5emNxGHj5SF3HP3m43Iq/YYgkSdMv4BR+RThPPnIIzrbAjEAHHtMgH7vVh036+bcw9UqBfSdD/IBqrKpJLERn5+Qi/4Q4EoReCC0CTDfZ+LcY0rUur0QZRkMpxx/9s4eCgIU+qfOcSlBvjoX7QAQ2MImUME1y5yJiyaWueamnhRBOwERGBuDKyGp4eBWp4i3esJcplrWYovjzPg9fL7Thy8v9KnrHy22PUFAYY+1vjKp";
    BNO055IMU gyro;
    public Orientation angles;


    double previousSpeed;
    double previousXInches;
    double previousYInches;
    double previousTime;

    boolean gsFirstRun = true;
    ElapsedTime speedTimer = new ElapsedTime();

    public Robot() {

    }

    public void init(HardwareMap ahwMap) throws InterruptedException {
        hardwareMap = ahwMap;
        initRobot();
    }


    public void initRobot() {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor intakeWheelL = hardwareMap.dcMotor.get("intakeWheelL");
        DcMotor intakeWheelR = hardwareMap.dcMotor.get("intakeWheelR");
        DcMotor liftMotorL = hardwareMap.dcMotor.get("liftMotorL");
        DcMotor liftMotorR = hardwareMap.dcMotor.get("liftMotorR");
        Servo foundationArmL = hardwareMap.servo.get("foundationArmL");
        Servo foundationArmR = hardwareMap.servo.get("foundationArmR");
        Servo claw = hardwareMap.servo.get("claw");
        CRServo horizontal = hardwareMap.crservo.get("horizontal");
        Servo flip = hardwareMap.servo.get("flip");
        Servo cap = hardwareMap.servo.get("cap");
        Servo kick = hardwareMap.servo.get("kick");
        ColorSensor color = hardwareMap.colorSensor.get("colorSensor");


        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(imuParameters);

        imu = new IMU(gyro);



        driveTrain = new DriveTrainVel(frontLeft, frontRight, backLeft, backRight);
        intake = new Intake(intakeWheelL, intakeWheelR);
        foundationArms = new FoundationArms(foundationArmL, foundationArmR);
        lift = new Lift(liftMotorL, liftMotorR);
        robotMotors = new RobotMotors(frontLeft,frontRight,backLeft,backRight);

    }
    public void waitForTick(long periodMs) throws InterruptedException {
        long remaining = periodMs - (long) speedTimer.milliseconds();

        if (remaining > 0)
            Thread.sleep(remaining);

        speedTimer.reset();
    }
}

//    public void driveToPoint(double xInches, double yInches, double heading, double speedModifier) {
//
//
//                telemetry.addLine("5439-starting driveToPoint"
//                        + " X:" + xInches
//                        + " Y:" + yInches
//                        + " Heading:" + heading
//                        + " SPM:" + speedModifier);
//        );
//
//        double wheel_encoder_ticks = 2400;
//        double wheel_diameter = 2.3622;  // size of wheels
//        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);
//
//        double cubackRightentXInches;
//        double cubackRightentYInches;
//        double startXPos = odometers.getXPos();
//        double startYPos = odometers.getYPos();
//
//        double frontLeftPower;
//        double frontRightPower;
//        double backLeftPower;
//        double backRightPower;
//
//        double cubackRightentSpeed;
//
//        double maxWheelPower;
//        double wheelPower = .15; //Minimum speed we start at
//
//        gsFirstRun = true;
//
//        ElapsedTime timeoutTimer = new ElapsedTime();
//
//        cubackRightentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
//        cubackRightentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
//        double distanceToX = xInches - cubackRightentXInches;
//        double distanceToY = yInches - cubackRightentYInches;
//        cubackRightentSpeed = getSpeed(cubackRightentXInches, cubackRightentYInches);
//
//        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance
//
//        telemetry.addLine(
//                "5439 driveToPoint"
//                        + " XPos:" + Double.toString(cubackRightentXInches)
//                        + " YPos:" + Double.toString(cubackRightentYInches)
//                        + " Wheel Power:" + Double.toString(wheelPower)
//                        + " Distance remaining:" + Double.toString(distanceRemaining)
//        );
//
//        while ((distanceRemaining > 1 || cubackRightentSpeed > 3) && opModeIsActive() && timeoutTimer.seconds() < .75) {
//
//            Lift.runLift();
//
//            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 25) / 100;
//
//            double speedIncrease = .15;
//
//            wheelPower += speedIncrease;
//            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
//                wheelPower = maxWheelPower;
//            }
//
//            double angleRadians;
//            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI / 4;
//
//            double adjustment = headingAdjustment2(heading);
//
//            frontLeftPower = wheelPower * Math.cos(angleRadians) - adjustment;
//            frontRightPower = wheelPower * Math.sin(angleRadians) + adjustment;
//            backLeftPower = wheelPower * Math.sin(angleRadians) - adjustment;
//            backRightPower = wheelPower * Math.cos(angleRadians) + adjustment;
//
//            driveTrain.setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
//
//            telemetry.addData("XPos: ", cubackRightentXInches);
//            telemetry.addData("YPos: ", cubackRightentYInches);
//            telemetry.addData("CubackRightent Speed:", cubackRightentSpeed);
//            telemetry.addData("Wheel Power: ", wheelPower);
//            telemetry.addData("distanceToX: ", distanceToX);
//            telemetry.addData("distanceToY: ", distanceToY);
//            telemetry.addData("Distance remaining: ", distanceRemaining);
//            telemetry.addData("andleradians: ", angleRadians);
//            telemetry.addData("angleradianDegrees: ", Math.toDegrees(angleRadians));
//            telemetry.update();
//
//            cubackRightentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
//            cubackRightentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
//            distanceToX = xInches - cubackRightentXInches;
//            distanceToY = yInches - cubackRightentYInches;
//
//            cubackRightentSpeed = getSpeed(cubackRightentXInches, cubackRightentYInches);
//
//            if (Math.abs(cubackRightentSpeed) > .5) {
//                timeoutTimer.reset();
//            }
//            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance
//
//            telemetry.addLine(
//                    "5439 driveToPoint"
//                            + " XPos:" + cubackRightentXInches
//                            + " YPos:" + cubackRightentYInches
//                            + " distanceToX:" + distanceToX
//                            + " distanceToY:" + distanceToY
//                            + " Wheel Power:" + wheelPower
//                            + " Distance remaining:" + distanceRemaining
//                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
//                            + " cubackRightentSpeed:" + cubackRightentSpeed
//                            + " ajustment:" + adjustment
//                            + " cubackRightent heading:" + imu.getCubackRightentHeading()
//            );
//        }
//        driveTrain.setMotorPower(0, 0, 0, 0);
//    }
