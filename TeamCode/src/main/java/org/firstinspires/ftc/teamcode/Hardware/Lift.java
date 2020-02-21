package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Lift {

    public DcMotorEx liftMotorL;     // Hub 2 Slot 2 GAMER MOMENTS 2020
    public DcMotorEx liftMotorR;     // Hub 3 Slot 3 GAMER MOMENTS 2020

    private static final double ENCODER_TICKS = 753.2; //NOT CORRECT - NEED FOR 1150 RPM
    private static final double WHEEL_RADIUS = 2;   // NOT CORRECT - NEED RADIUS OF SPOOL
    private static final int GEAR_RATIO = 26; //WORM GEAR
    private final double MAX_VELOCITY = 20; //NOT CORRECT - Need to Test to Find
    private static final double IN_PER_REV = 2 * Math.PI * WHEEL_RADIUS;
    private static double TICKS_PER_REV = ENCODER_TICKS * GEAR_RATIO;
    private static double TICKS_PER_IN = TICKS_PER_REV / IN_PER_REV;



    public Lift(DcMotor left, DcMotor right)
    {

        final double PIDF_F = 1150 / MAX_VELOCITY; //32767
        final double PIDF_P = 0.1 * PIDF_F;
        final double PIDF_I = 0.1 * PIDF_P;
        final double PIDF_D = 0;
       // PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PIDF_P,PIDF_I,PIDF_D,PIDF_F);

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection(DcMotor.Direction.FORWARD);

        liftMotorL = (DcMotorEx) left;
        liftMotorR = (DcMotorEx) right;

      //  liftMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
      //  liftMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
    }



    public void turnOnEncoders()
    {
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /**
     * Checks if the encoders are on
     * @return
     * true = They ARE On
     * false = They AREN'T On
     */
    public boolean getEncoderStatus()
    {
        return (liftMotorL.getMode() == DcMotor.RunMode.RUN_USING_ENCODER && liftMotorR.getMode() == DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rise()
    {
        liftMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorL.setPower(1);
        liftMotorR.setPower(1);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lower()
    {
        liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorL.setPower(1);
        liftMotorR.setPower(1);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stop()
    {
        liftMotorR.setPower(0);
        liftMotorL.setPower(0);
    }

    /**
     * Lifts the block to a target position regardless of current position.
     * Doesn't reset encoders unless at position 0
     * @param inches = desired number of inches for lift to rise
     */
    public void runToPosition(int inches)
    {
        if(!getEncoderStatus()) {
            turnOnEncoders();
        }

        liftMotorR.setTargetPosition((int)(inches*TICKS_PER_IN)); //insert calculations for this number GAMER MOMENTS 2020
        liftMotorL.setTargetPosition((int)(inches*TICKS_PER_IN)); //insert calculations for this number GAMER MOMENTS 2020

        if(liftMotorL.getTargetPosition() <= 0 && liftMotorR.getTargetPosition() <= 0)
            returnToZero();
        else if(liftMotorL.getCurrentPosition() < liftMotorL.getTargetPosition() && liftMotorR.getCurrentPosition() < liftMotorR.getTargetPosition())
            rise();
        else
            lower();

    }

    /**
     * Returns lift closed position
     */
    public void returnToZero()
    {
        liftMotorR.setTargetPosition(0);
        liftMotorL.setTargetPosition(0);
        lower();
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


}
