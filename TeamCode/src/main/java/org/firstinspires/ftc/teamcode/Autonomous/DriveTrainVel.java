package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.RobotMotors;
import org.firstinspires.ftc.teamcode.Hardware.RobotMotorsAuto;

public class DriveTrainVel extends RobotMotorsAuto {
    private final double MAX_VELOCITY = 20;

    public DriveTrainVel(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);

        final double PIDF_F = 223 / MAX_VELOCITY; //32767
        final double PIDF_P = 0.1 * PIDF_F;
        final double PIDF_I = 0.1 * PIDF_P;
        final double PIDF_D = 0;
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PIDF_P,PIDF_I,PIDF_D,PIDF_F);
//        for (int i = 0; i < wheels.size(); i++) {
//            wheels.get(i).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        }
    }

    @Override
    public void move(int inches, double power, int[] dir) {
        if (!checkMotors()) {
            return;
        }

        int dest = getTicks(inches);

        for (int i = 0; i < wheels.size(); i++) {
            ((DcMotorEx)wheels.get(i)).setVelocity(MAX_VELOCITY * power * dir[i]);
        }

        boolean isBusy = true;
        while (isBusy) {
            for (int i = 0; i < wheels.size(); i++) {
                if (Math.abs(wheels.get(i).getCurrentPosition()) > dest) {
                    isBusy = false;
                }
            }
        }
    }

    private boolean checkMotors() {
        boolean instanceOf = true;
        for (int i = 0; i < wheels.size(); i++) {
            if (wheels.get(i) instanceof DcMotorEx) {
                instanceOf = false;
                break;
            }
        }

        return instanceOf;
    }
    public void moveForward(int inches, double power) {
        direction = MoveStyle.FORWARD;
        move(inches, power, getDirs(direction));
    }

    public void moveBackward(int inches, double power) {
        direction = MoveStyle.BACKWARD;
        move(inches, power, getDirs(direction));
    }

    public void turnLeft(int inches, double power) {
        direction = MoveStyle.TURN_LEFT;
        move(inches, power, getDirs(direction));
    }

    public void turnRight(int inches, double power) {
        direction = MoveStyle.TURN_RIGHT;
        move(inches, power, getDirs(direction));
    }

    public void strafeLeft(int inches, double power) {
        direction = MoveStyle.LEFT;
        move(inches, power, getDirs(direction));
    }

    public void strafeRight(int inches, double power) {
        direction = MoveStyle.BACKWARD;
        move(inches, power, getDirs(direction));
    }
}
