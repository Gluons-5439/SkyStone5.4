package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

/**
 * Drive Train Class
 */

public class RobotMotors {
    private DcMotorEx frontRight;    // Hub 3 Slot 0
    private DcMotorEx frontLeft;     // Hub 2 Slot 0
    private DcMotorEx backRight;     // Hub 3 Slot 1
    private DcMotorEx backLeft;      // Hub 2 Slot 1
    protected ArrayList<DcMotorEx> wheels = new ArrayList<>();

    protected MoveStyle direction;

    public enum MoveStyle {
        FORWARD, BACKWARD, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT
    }

    public RobotMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        frontLeft = (DcMotorEx) fl;
        frontRight = (DcMotorEx) fr;
        backLeft = (DcMotorEx) bl;
        backRight = (DcMotorEx) br;
        
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheels.add(frontLeft);
        wheels.add(frontRight);
        wheels.add(backLeft);
        wheels.add(backRight);

        setMotorPower();
    }

    public ArrayList<DcMotorEx> getWheels() {
        return wheels;
    }

    public void setMotorPower(double flpower, double frpower, double blpower, double brpower){
        frontLeft.setPower(flpower);
        frontRight.setPower(frpower);
        backLeft.setPower(blpower);
        backRight.setPower(brpower);
    }
    
    protected void setMotorPower() {
        for (int i = 0; i < wheels.size(); i++) {
            wheels.get(i).setPower(0);
            wheels.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    protected boolean motorsAreBusy() {
        return frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy();
    }

    public void turnOffEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnOnEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorPower();
    }
}