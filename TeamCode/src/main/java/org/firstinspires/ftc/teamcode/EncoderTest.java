package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Autonomous.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.FoundationArms;
import android.media.MediaPlayer;
import java.lang.Runnable;

import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "EncoderTest", group = "Autonomous")
public class EncoderTest extends LinearOpMode {
    private DriveTrain driveTrain;

    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);
        executor.schedule(new Runnable() {
            public void run() {
                telemetry.addData("FL Velocity: ", driveTrain.getWheels().get(0).getVelocity());
                telemetry.addData("FR Velocity: ", driveTrain.getWheels().get(1).getVelocity());
                telemetry.addData("BL Velocity: ", driveTrain.getWheels().get(2).getVelocity());
                telemetry.addData("BR Velocity: ", driveTrain.getWheels().get(3).getVelocity());
                telemetry.update();
            }
        }, 100, TimeUnit.MILLISECONDS);

        driveTrain.moveForward(96, 1);

        // robot.driveTrain.moveForward(24,.4);
        // robot.driveTrain.moveBackward(24,.4);


//
//        final ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);
//        executor.schedule(new Runnable() {
//            @Override
//            public void run() {
//                robot.tfod.deactivate();
//            }
//        }, 29, TimeUnit.SECONDS);
//
////        robot.turnDegrees(90, 'l', 0.5, hardware); GAMER MOMENTS 2020
////        Thread.sleep(250); GAMER MOMENTS 2020
//        robot.moveForward((int)(275 * fricRatio), 0.6, hardware);
//        Thread.sleep(100);
//        robot.turnDegrees((int)(64 * fricRatio), 'r', 0.7, hardware);
//        Thread.sleep(100);
//
//        robot.tfod.activate();
//        robot.setMotorPower(0.225, 1, 1, 1, 1, hardware); // Make sure you account for friction and change
//        if (robot.tfod != null) {
//            boolean skyStoneFound = false;                                            // motor power if necessary
//            while (!skyStoneFound) {
//                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("Found ", updatedRecognitions.size());
//                    for (Recognition recognition : updatedRecognitions) {
//                        if (recognition.getLabel().equals(AutonomousTools.LABEL_SKYSTONE)) {
//                            telemetry.addData("SKYSTONE FOUND!", ' ');
//                            skyStoneFound = true;
//                            robot.setMotorPower(0, hardware);
//                        }
//                    }
//                }
//                telemetry.update();
//            }
//            robot.moveForward((int)(fricRatio * 500), -.6, hardware);
//            robot.turnDegrees((int)(fricRatio * 40), 'l', 0.7, hardware);
//            hardware.intakeWheelL.setPower(0.7);
//            hardware.intakeWheelR.setPower(0.7);
//            robot.moveForward((int)(fricRatio * 900), .6, hardware);
//            //By now we should have the skystone in our robot GAMER MOMENTS 2020
//            hardware.intakeWheelL.setPower(0);
//            hardware.intakeWheelR.setPower(0);
//
//            robot.moveForward((int)(fricRatio * 480), -.6, hardware);
//            Thread.sleep(100);
//            robot.turnDegrees((int)(fricRatio * 85), 'l', 0.7, hardware);
//            //By now we should be on the second GAMER MOMENTS 2020
//            Thread.sleep(100);
//            robot.moveForward((int)(fricRatio * 350), .6, hardware);
//
//            executor.schedule(new Runnable() {
//                @Override
//                public void run() {
//                    robot.setMotorPower(0, hardware);
//                }
//            }, 5, TimeUnit.SECONDS);
//
//            robot.setMotorPower(0.225, hardware);
//            while (hardware.wheels.get(0).getPower() != 0) {
//                if (hardware.color.blue() > 3 * hardware.color.red()) {
//                    robot.setMotorPower(0, hardware);
//                }
//            }
//            robot.moveForward((int)(fricRatio * 500), .5, hardware);
//            robot.moveForward((int)(fricRatio * 500), -.5, hardware);
//        }
    }
}





