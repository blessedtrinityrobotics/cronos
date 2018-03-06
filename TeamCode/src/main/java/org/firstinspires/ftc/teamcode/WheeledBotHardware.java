package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Base class for the op-modes for the Clock robot (FTC team 9785 / Cronos).
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any  fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 */
public class WheeledBotHardware extends OpMode {

    final double LEFT_OPEN_WIDE_POSITION  =  0.1;
    final double LEFT_OPEN_POSITION       =  0.4;
    final double LEFT_CLOSED_POSITION     =  0.6;
    final double LEFT_PUSH_POSITION       =  0.9;
    final double RIGHT_OPEN_WIDE_POSITION =  0.9;
    final double RIGHT_OPEN_POSITION      =  0.6;
    final double RIGHT_CLOSED_POSITION    =  0.4;
    final double RIGHT_PUSH_POSITION      =  0.9;
    //final double RELIC_OPEN               = 0.1;
    //final double RELIC_CLOSE              = 0.3;

    final double BALANCE_UP            = 0.9;
    final double BALANCE_DOWN          = 0.2;

    final double JOULE_UP            = 0.225;
    final double JOULE_DOWN          = 0.82;

    final float  WHEEL_DIAMETER      = 4.0f / 4.11f;

    DcMotor leftRearMotor;
    DcMotor leftFrontMotor;
    DcMotor rightRearMotor;
    DcMotor rightFrontMotor;
    DcMotor elvMotor;
    Servo topLeftGrip;
    Servo topRightGrip;
    Servo bottomLeftGrip;
    Servo bottomRightGrip;
    Servo balance;
    Servo joule;
    Servo relic;

    BNO055IMU imu;
    ColorSensor colorSensor;
    DcMotor LinSlideUpDown = null;
    DcMotor LinSlideMotor = null;

    TouchSensor armTouch;
    TouchSensor beaconTouch;
    OpticalDistanceSensor opticalDistanceSensor;

    private int prevLeftRearStep;
    private int prevLeftFrontStep;
    private int prevRightRearStep;
    private int prevRightFrontStep;
    
    /**
     * Absolute position of the robot in the x-axis.
     */
    public double positionX;
    /**
     * Absolute position of the robot in the y-axis.
     */
    public double positionY;

    /**
     * Absolute heading (z axis / up) of the robot in radians.
     */
    public double heading;

    /**
     * Indicates that location encoders is under reset.
     */
    public boolean onArmReset;


    @Override
    public void init() {

        StringBuilder sb = new StringBuilder();

        sb.append("lr_drive: ");
        try {
            leftRearMotor = hardwareMap.dcMotor.get("left_drive1");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("lf_drive: ");
        try {
            leftFrontMotor = hardwareMap.dcMotor.get("left_drive0");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("rr_drive: ");
        try {
            rightRearMotor = hardwareMap.dcMotor.get("right_drive3");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("rf_drive: ");
        try {
            rightFrontMotor = hardwareMap.dcMotor.get("right_drive2");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("elv_drive: ");
        try {
            elvMotor = hardwareMap.dcMotor.get("elevator");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("arm_touch: ");
        try {
            armTouch = hardwareMap.touchSensor.get("arm_touch");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("beacon_touch: ");
        try {
            beaconTouch = hardwareMap.touchSensor.get("beacon_touch");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("tl_grip: ");
        try {
            topLeftGrip = hardwareMap.servo.get("tl_grip");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("tr_grip: ");
        try {
            topRightGrip = hardwareMap.servo.get("tr_grip");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("bl_grip: ");
        try {
            bottomLeftGrip = hardwareMap.servo.get("bl_grip");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("br_grip: ");
        try {
            bottomRightGrip = hardwareMap.servo.get("br_grip");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("balance: ");
        try {
            balance = hardwareMap.servo.get("balance");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("joule: ");
        try {
            joule = hardwareMap.servo.get("joule_arm");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("relic: ");
        try {
            relic = hardwareMap.servo.get("relic");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("imu: ");
        try {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("color ");
        try {
            colorSensor = hardwareMap.colorSensor.get("joule_color");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("sliup ");
        try {
            LinSlideUpDown = hardwareMap.get(DcMotor.class, "LinSlideUpDown");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("slimot ");
        try {
            LinSlideMotor = hardwareMap.get(DcMotor.class, "LinSlideMotor");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }

//        sb.append("optical ");
//        try {
//            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("optical");
//            sb.append("OK ");
//        } catch (Exception ex) {
//            sb.append("ERR ");
//        }

        //Reverse the right-side motors
        if (rightRearMotor != null)
            rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFrontMotor != null)
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Elevator direction
        if (elvMotor != null)
            elvMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set gripper to close
        //closeGripper();

        //Prepare drive
        resetDriveEncoders();

        //Prepare arm
        resetElevatorEncoders();

        //Report status
        telemetry.addData("Status", sb.toString());
    }

    @Override
    public void loop () {
        /***
         // on arm reset, keep changing drive mode until ready
         DcMotor.RunMode mode = elvMotor.getMode();
         if ( onArmReset ) {
         // force a reset until we detect a position==0
         if ( mode != DcMotor.RunMode.STOP_AND_RESET_ENCODER || elvMotor.getCurrentPosition() != 0)
         elvMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         // signal reset done when ready
         if ( mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER && elvMotor.getCurrentPosition() == 0) {
         onArmReset = false;
         }
         }
         else if (mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
         // force a power mode until we detect it
         elvMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
         ***/

//        // signal reset done when ready
//        DcMotor.RunMode mode = elvMotor.getMode();
//        if ( mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER && elvMotor.getCurrentPosition() == 0) {
//            elvMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        else if (mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
//            // force a power mode until we detect it
//            elvMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }


        if ( imu != null ) {
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            //Position pos = imu.getPosition();
            //positionX = pos.x;
            //positionY = pos.y;
        }

        // update absolution position
        updatePosition();
    }


    /**
     * Set the gripper to open position.
     */
    void openWideGripper() {
        if (topLeftGrip != null)
            topLeftGrip.setPosition(LEFT_OPEN_WIDE_POSITION);
        if (topRightGrip != null)
            topRightGrip.setPosition(RIGHT_OPEN_WIDE_POSITION);
        if (bottomLeftGrip != null)
            bottomLeftGrip.setPosition(LEFT_OPEN_WIDE_POSITION);
        if (bottomRightGrip != null)
            bottomRightGrip.setPosition(RIGHT_OPEN_WIDE_POSITION);
    }

    /**
     * Set the gripper to open position.
     */
    void openGripper() {
        if (topLeftGrip != null)
            topLeftGrip.setPosition(LEFT_OPEN_POSITION);
        if (topRightGrip != null)
            topRightGrip.setPosition(RIGHT_OPEN_POSITION);
        if (bottomLeftGrip != null)
            bottomLeftGrip.setPosition(LEFT_OPEN_POSITION);
        if (bottomRightGrip != null)
            bottomRightGrip.setPosition(RIGHT_OPEN_POSITION);
    }

    /**
     * Set the gripper to close position.
     */
    void closeGripper() {
        if (topLeftGrip != null)
            topLeftGrip.setPosition(LEFT_CLOSED_POSITION);
        if (topRightGrip != null)
            topRightGrip.setPosition(RIGHT_CLOSED_POSITION);
        if (bottomLeftGrip != null)
            bottomLeftGrip.setPosition(LEFT_CLOSED_POSITION);
        if (bottomRightGrip != null)
            bottomRightGrip.setPosition(RIGHT_CLOSED_POSITION);
    }

    /**
     * Set the right gripper to push position.
     */
    void pushRightGripper() {
        if (topRightGrip != null)
            topRightGrip.setPosition(RIGHT_PUSH_POSITION);
        if (bottomRightGrip != null)
            bottomRightGrip.setPosition(RIGHT_PUSH_POSITION);
    }

    /**
     * Set the left gripper to push position.
     */
    void pushLeftGripper() {
        if (topLeftGrip != null)
            topLeftGrip.setPosition(LEFT_PUSH_POSITION);
        if (bottomLeftGrip != null)
            bottomLeftGrip.setPosition(LEFT_PUSH_POSITION);
    }

    /*void RelicDown() {
        if (relic != null)
            relic.setPosition(RELIC_CLOSE);
    }

    void RelicUp() {
        if (relic != null)
            relic.setPosition(RELIC_OPEN);
    }
    */
    /**
     * Raise the balance
     */
    void balanceUp()
    {
        if ( balance != null)
            balance.setPosition(BALANCE_UP);
    }

    /**
     * Lower the balance.
     */
    void balanceDown()
    {
        if ( balance != null)
            balance.setPosition(BALANCE_DOWN);
    }

    /**
     * Raise the joule arm.
     */
    void jouleArmUp()
    {
        if ( joule != null)
            joule.setPosition(JOULE_UP);
    }

    /**
     * Lower the joul arm.
     */
    void jouleArmDown()
    {
        if ( joule != null)
            joule.setPosition(JOULE_DOWN);
    }



    /**
     * Move the arm with the specified power: positive value raises the arm.
     *
     * @param power the power level: positive (up)/negative (down)
     */
    void moveElevator(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1, 1);

        if (elvMotor != null ) {
            // set motor to the correct mode (if necessary)
            if ( elvMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                elvMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            boolean atLowerLimit = armTouch != null && armTouch.isPressed();
            boolean atUpperLimit = false; //elvMotor.getCurrentPosition() > 2000;

            if (atLowerLimit && power < 0 && !onArmReset) {
                stopElevator();
                resetElevatorEncoders();

                // we are done
                return;
            }

            // do not keep raising the arm if at upper limit
            if (atUpperLimit && power > 0)
                power = 0;

            // do not keep lowering the arm if at lower limit
            if (atLowerLimit && power < 0)
                power = 0;

            // ok send the power level
            elvMotor.setPower(power);
        }
    }

    /**
     * Move the arm with the specified power: positive value raises the arm.
     *
     * @param target the power level: positive (up)/negative (down)
     */
    void moveElevatorToPos(int target) {
        //Clip the power values so that it only goes from -1 to 1
        //power = Range.clip(power, -1, 1);

        if (elvMotor != null ) {
            // set motor to the correct mode (if necessary)
            if (elvMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                elvMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // get current position
            int pos = elvMotor.getCurrentPosition();

            // set where we want to move
            elvMotor.setTargetPosition(target);

            // set the speed
            double sign = Math.signum(target - pos);
            elvMotor.setPower(sign * 0.5);

        }
    }

    /**
     * Move the robot a specified distance in the forward axis.
     * @param forward   the distance to move forward in inches
     * @param power     the move power to use
     */
    void moveRobotForwardToPos(float forward, double power) {
        // compute the number of ticks each robot has to move

        int fwd = (int)(280f * forward / (Math.PI * WHEEL_DIAMETER));
        power = Range.clip(power, -1, 1);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (leftRearMotor != null)
            leftRearMotor.setTargetPosition(fwd);
        if (leftFrontMotor != null)
            leftFrontMotor.setTargetPosition(fwd);
        if (rightRearMotor != null)
            rightRearMotor.setTargetPosition(fwd);
        if (rightFrontMotor != null)
            rightFrontMotor.setTargetPosition(fwd);

        if (leftRearMotor != null)
            leftRearMotor.setPower(power);
        if (leftFrontMotor != null)
            leftFrontMotor.setPower(power);
        if (rightRearMotor != null)
            rightRearMotor.setPower(power);
        if (rightFrontMotor != null)
            rightFrontMotor.setPower(power);
    }

    /**
     * Stop arm movement.
     */
    void stopElevator() {
        if (elvMotor != null ) {
            if (elvMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                elvMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elvMotor.setPower(0);
        }
    }

    /**
     * Reset arm motor encoders.
     */
    void resetElevatorEncoders() {
        if (elvMotor != null) {
            // stop motor
            elvMotor.setPower(0);

            // send command
            elvMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // we are now in reset mode
            onArmReset = true;
        }
    }

    /**
     * Set the arm motor to the specified mode.
     *
     * @param mode the motor run mode
     */
    void setElevatorMode(DcMotor.RunMode mode) {
        if (elvMotor != null)
            elvMotor.setMode(mode);
    }

    /**
     * Reset drive motor encoders.
     */
    void resetDriveEncoders() {
        if (leftRearMotor != null)
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (leftFrontMotor != null)
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightRearMotor != null)
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightFrontMotor != null)
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset relative variables
        prevLeftRearStep = 0;
        prevLeftFrontStep = 0;
        prevRightRearStep = 0;
        prevRightFrontStep = 0;

        // reset absolute variables
        positionX = 0.0;
        positionY = 0.0;
        heading = 0.0;
    }

    /**
     * Use the motor encoders and the imu sensor to update the absolute position of the robot.
     */
    private void updatePosition() {
        double distance = 0;
        int motors = 0;
        int leftRearStep = 0;
        int leftFrontStep = 0;
        int rightRearStep = 0;
        int rightFrontStep = 0;
        if (leftRearMotor != null) {
            leftRearStep = leftRearMotor.getCurrentPosition();
            distance += leftRearStep - prevLeftRearStep;
            motors += 1;
        }
        if (leftFrontMotor != null) {
            leftFrontStep = leftFrontMotor.getCurrentPosition();
            distance += leftFrontStep - prevLeftFrontStep;
            motors += 1;
        }
        if (rightRearMotor != null) {
            rightRearStep = rightRearMotor.getCurrentPosition();
            distance += rightRearStep - prevRightRearStep;
            motors += 1;
        }
        if (rightFrontMotor != null) {
            rightFrontStep = rightFrontMotor.getCurrentPosition();
            distance += rightFrontStep - prevRightFrontStep;
            motors += 1;
        }

        if (motors > 0 && !Double.isNaN(heading)) {
            // compute average of distance iin pulses
            distance = distance / motors;

            // convert pulses to inches
            distance = distance / 280.0 * Math.PI * WHEEL_DIAMETER;

            // read angle
            double rad = Math.toRadians(heading);

            //telemetry.addData("raw", String.format("%d %.0f", raw, distance));

            // compute displacement
            double dx = distance * Math.sin(rad);
            double dy = distance * Math.cos(rad);

            // update position
            positionX = positionX + dx;
            positionY = positionY + dy;
        }

        // prepare for next reading
        prevLeftRearStep = leftRearStep;
        prevLeftFrontStep = leftFrontStep;
        prevRightRearStep = rightRearStep;
        prevRightFrontStep = rightFrontStep;

        //telemetry.addData("front", String.format("%d %d", -leftFrontStep, -rightFrontStep));
        //telemetry.addData("rear", String.format("%d %d", -leftRearStep, -rightRearStep));
    }

    /**
     * Set the drive motors to the specified mode.
     *
     * @param mode the motor run mode
     */
    void setDriveMode(DcMotor.RunMode mode) {
        if (leftRearMotor != null)
            leftRearMotor.setMode(mode);
        if (leftFrontMotor != null)
            leftFrontMotor.setMode(mode);
        if (rightRearMotor != null)
            rightRearMotor.setMode(mode);
        if (rightFrontMotor != null)
            rightFrontMotor.setMode(mode);
    }

    /**
     * Set the power for both drive motors using the specified value.
     *
     * @param power the power level
     */
    void setDrivePower(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1, 1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = false;//opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && power > 0;

        if (!stop) {
            setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (leftRearMotor != null)
                leftRearMotor.setPower(power);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(power);
            if (rightRearMotor != null)
                rightRearMotor.setPower(power);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(power);
        }
    }

    /**
     * Set the power for both drive motors using the specified values.
     *
     * @param leftPower  the power level of the left drive motor
     * @param rightPower the power level of the right drive motor
     */
    void setDrivePower(double leftPower, double rightPower) {
        //Clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = false;//opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && (leftPower > 0 || rightPower > 0);

        if (!stop) {
            setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (leftRearMotor != null)
                leftRearMotor.setPower(leftPower);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(leftPower);
            if (rightRearMotor != null)
                rightRearMotor.setPower(rightPower);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(rightPower);
        }
    }

    /**
     * Set the power for both drive motors using the specified values.
     *
     * @param forward  the power level of the forward motion (+forward)
     * @param right the power level of the right motion (+right)
     * @param turn  the power level of the turn motion (+clockwise)
     */
    void setDrivePower(double forward, double right, double turn) {

        double leftFrontPower  = forward - right + turn;
        double leftRearPower   = forward + right + turn;
        double rightFrontPower = forward + right - turn;
        double rightRearPower  = forward - right - turn;

        //Clip the power values so that it only goes from -1 to 1
        leftFrontPower  = Range.clip(leftFrontPower, -1, 1);
        leftRearPower   = Range.clip(leftRearPower, -1, 1);
        rightFrontPower = Range.clip(rightFrontPower, -1, 1);
        rightRearPower  = Range.clip(rightRearPower, -1, 1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = false;//opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && (leftPower > 0 || rightPower > 0);

        if (!stop) {
            setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (leftRearMotor != null)
                leftRearMotor.setPower(leftRearPower);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(leftFrontPower);
            if (rightRearMotor != null)
                rightRearMotor.setPower(rightRearPower);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(rightFrontPower);
        }
    }

    /**
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double value) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) Math.round(value * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (value < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    /**
     * Normalizes an angle in radians between the range of [reference, reference + 2 PI).
     *
     * @param angle     the angle in radians
     * @param reference the reference in radians
     * @return the normalized angle in radians
     */
    public static double NormalizeAngle(double angle, double reference) {
        final double TWO_PI = 2.0 * Math.PI;
        double result = (angle - reference) % TWO_PI;
        return result < 0.0 ? TWO_PI + reference + result : reference + result;
    }
 }