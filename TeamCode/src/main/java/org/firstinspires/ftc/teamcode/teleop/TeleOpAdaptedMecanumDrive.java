package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_BASE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class TeleOpAdaptedMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    public enum Height{
        HIGH,
        MEDIUM,
        LOW
    }

    public HardwareTeleOp.Height robot_height = HardwareTeleOp.Height.LOW;

    //define servos
    public Servo gripper = null;
    public Servo servo_brat_sus = null;
    public Servo servo_brat_jos = null;
    public DcMotor lift_left = null;
    public DcMotor lift_right = null;

    public boolean rrmode = true;

    public boolean chassis_test = true;
    public boolean lift_test = true;
    public boolean gripper_test = true;
    public boolean servo_brat_test = true;
    public boolean all_in = false;
    public boolean initialisation = true;

    public double angle = 0;
    public double distance = 0;

    double COUNTS_PER_CM = 537.7 / (9.6 * 3.1415);

    public TeleOpAdaptedMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    }

    public void initialize(HardwareMap hardwareMap){
        if(chassis_test) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            //        rightFront.setDirection(DcMotor.Direction.REVERSE);
            //        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);
            }

            if (RUN_USING_ENCODER) {
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(lift_test || all_in) {
            lift_left = hardwareMap.get(DcMotor.class, "lift_left");
            lift_left.setDirection(DcMotor.Direction.FORWARD);
            lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_left.setPower(0);

            lift_right = hardwareMap.get(DcMotor.class, "lift_right");
            lift_right.setDirection(DcMotor.Direction.FORWARD);
            lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_right.setPower(0);

            lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gripper_test || all_in) {
            //initialize servos
            gripper = hardwareMap.get(Servo.class, "gripper");
            gripper.setDirection(Servo.Direction.FORWARD);
        }
        if(servo_brat_test || all_in) {
            servo_brat_jos = hardwareMap.get(Servo.class, "servo_brat_jos");
            servo_brat_sus = hardwareMap.get(Servo.class, "servo_brat_sus");

            servo_brat_jos.setDirection(Servo.Direction.FORWARD);
            servo_brat_sus.setDirection(Servo.Direction.FORWARD);
//            pickup_fata();
        }
        initialisation = false;
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }
    public void runToPositionCHASSIS(){
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowerCHASSIS(double power){

        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(power);
        leftFront.setPower(power);
    }
    public void setPowerZeroCHASSIS(){
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopAndResetEncodersCHASSIS(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //functii pentru celelalte motoare

    public void closeGripper(){
        gripper.setPosition(1);
    }
    public void openGripper(){
        gripper.setPosition(0);
    }
    public void lift_pos_down(int pos){
        pos = -pos;
        lift_left.setTargetPosition(pos);
        lift_right.setTargetPosition(pos);

        if(lift_left.getCurrentPosition() <= pos-10){
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);
        }
        else{
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
    }
    public void lift_pos_up(int pos) {
        pos = -pos;
        lift_left.setTargetPosition(pos);
        lift_right.setTargetPosition(pos);

        if (lift_left.getCurrentPosition() >= pos + 10 && (robot_height.equals(HardwareTeleOp.Height.LOW) || robot_height.equals(HardwareTeleOp.Height.MEDIUM))) {
            lift_right.setPower(1);
            lift_left.setPower(1);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
    }
    public void moveForward(double power,double distance) {

//        stopAndResetEncodersCHASSIS();

        int target =  (int)(distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(-target);
        leftRear.setTargetPosition(-target);
        rightRear.setTargetPosition(-target);

        runToPositionCHASSIS();

        setPowerCHASSIS(power);

    }
    public void moveReverse(double power,double distance) {

//        stopAndResetEncodersCHASSIS();

        int target =  (int)(distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();

        setPowerCHASSIS(power);

    }
    public void moveRight(double power,double distance) {

//        stopAndResetEncodersCHASSIS();

        int target = -(int) (distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(-target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();

        setPowerCHASSIS(power);

    }

    public void rotateRight(double power, double distance){

        stopAndResetEncodersCHASSIS();

        int target = (int)(distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(-target);
        leftRear.setTargetPosition(-target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();
        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();

    }

    public void liftDown(){
        lift_pos_down(0);
    }
    public void rrswitch(boolean s){rrmode = s;}
    public void pickup_fata(){
        if(initialisation) {
            closeGripper();
        }else {
            liftDown();
            openGripper();
            servo_brat_jos.setPosition(0);
            servo_brat_sus.setPosition(0.4777);
        }
        robot_height = HardwareTeleOp.Height.LOW;
    }
    public void cone_up_high_fata(){
        if(robot_height.equals(HardwareTeleOp.Height.MEDIUM)){
            lift_pos_up(700);
            robot_height = HardwareTeleOp.Height.HIGH;
        }else if(robot_height.equals(HardwareTeleOp.Height.LOW)){
            lift_pos_up(700);
            robot_height = HardwareTeleOp.Height.HIGH;
        }
        servo_brat_jos.setPosition(0.48);
        servo_brat_sus.setPosition(0.62);
    }
    public void cone_up_mid_fata(){
        if(robot_height.equals(HardwareTeleOp.Height.LOW))
            lift_pos_up(390);
        robot_height = HardwareTeleOp.Height.MEDIUM;
        servo_brat_jos.setPosition(0.695);
        servo_brat_sus.setPosition(0.97);
    }
    public void cone_up_high_spate(){
        lift_pos_up(605);
        servo_brat_jos.setPosition(1);
        servo_brat_sus.setPosition(1);
    }

    public void cone_up_low_fata(){
        liftDown();
        robot_height = HardwareTeleOp.Height.LOW;
        servo_brat_jos.setPosition(0.69);
        servo_brat_sus.setPosition(1);
    }
//    public void cone_up_spate(){
//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.6);
//    }
}
