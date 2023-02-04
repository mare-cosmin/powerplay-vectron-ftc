package org.firstinspires.ftc.teamcode.custom.teleop;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
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

    public Height robot_height = Height.LOW;

    //define servos
    public Servo gripper = null;
    public Servo servo_brat_sus = null;
    public Servo servo_brat_jos = null;
    public Servo servo_rotation = null;
    public DcMotor lift_exp = null;
    public DcMotor lift_ctrl = null;

    public boolean rrmode = true;

    public boolean chassis_test = true;
    public boolean lift_test = true;
    public boolean gripper_test = true;
    public boolean servo_brat_test = true;
    public boolean all_in = false;
    public boolean initialisation = true;
    public boolean backwards = false;

    private final double servo_rot_up = 0.2;
    private final double servo_rot_down = 0.85;

    double COUNTS_PER_CM = 537.7 / (9.6 * 3.1415);

    public TeleOpAdaptedMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
    }

    public void initialize(HardwareMap hardwareMap){
        if(chassis_test) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

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
            lift_exp = hardwareMap.get(DcMotor.class, "lift_exp");
            lift_exp.setDirection(DcMotor.Direction.FORWARD);
            lift_exp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift_exp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_exp.setPower(0);

            lift_ctrl = hardwareMap.get(DcMotor.class, "lift_ctrl");
            lift_ctrl.setDirection(DcMotor.Direction.FORWARD);
            lift_ctrl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift_ctrl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_ctrl.setPower(0);

            lift_exp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift_ctrl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(gripper_test || all_in) {
            //initialize servos
            gripper = hardwareMap.get(Servo.class, "gripper");
            gripper.setDirection(Servo.Direction.FORWARD);
        }
        if(servo_brat_test || all_in) {
            servo_brat_jos = hardwareMap.get(Servo.class, "servo_brat_jos");
            servo_brat_sus = hardwareMap.get(Servo.class, "servo_brat_sus");
            servo_rotation = hardwareMap.get(Servo.class, "servo_rotation");

            servo_rotation.setDirection(Servo.Direction.FORWARD);
            servo_brat_jos.setDirection(Servo.Direction.FORWARD);
            servo_brat_sus.setDirection(Servo.Direction.FORWARD);
//            pickup_fata();
        }
        initialisation = false;
    }


    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
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
        gripper.setPosition(0.5);
    }
    public void openGripper(){
        gripper.setPosition(0.2);
    }
    public void lift_pos_down(int pos){
        lift_exp.setTargetPosition(-pos);
        lift_ctrl.setTargetPosition(pos);

        if(lift_exp.getCurrentPosition() * lift_ctrl.getCurrentPosition() == 0){
            return;
        }

        if(lift_exp.getCurrentPosition() <= pos+80){
            lift_exp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_ctrl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_exp.setPower(0.3);
            lift_ctrl.setPower(0.3);
        }
        else{
            lift_exp.setPower(0);
            lift_ctrl.setPower(0);
        }
    }
    public void lift_pos_up(int pos) {
        lift_exp.setTargetPosition(-pos);
        lift_ctrl.setTargetPosition(pos);

        if (robot_height.equals(Height.LOW) || robot_height.equals(Height.MEDIUM)) {
            lift_ctrl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_exp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_ctrl.setPower(1);
            lift_exp.setPower(1);
        } else {
            lift_exp.setPower(0);
            lift_ctrl.setPower(0);
        }
    }
    public void moveForward(double power,double distance) {

//        stopAndResetEncodersCHASSIS();

        int target =  (int)(distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();

        setPowerCHASSIS(power);

    }
    public void moveReverse(double power,double distance) {

//        stopAndResetEncodersCHASSIS();

        int target =  (int)(distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(-target);
        leftRear.setTargetPosition(-target);
        rightRear.setTargetPosition(-target);

        runToPositionCHASSIS();

        setPowerCHASSIS(power);

    }
    public void moveRight(double power,double distance) {

        stopAndResetEncodersCHASSIS();

        int target = -(int) (distance * COUNTS_PER_CM);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(-(target + 150));
        leftRear.setTargetPosition(target);
        rightRear.setTargetPosition(-target);

        runToPositionCHASSIS();

        setPowerCHASSIS(power);

    }

    public void liftDown(){
        lift_pos_down(0);
    }

    public void rrswitch(boolean s){rrmode = s;}

    public void auto_position(){
        servo_brat_jos.setPosition(0.08);
        servo_brat_sus.setPosition(0.45);
        robot_height = Height.LOW;
    }

    public void pickup_fata() {
        liftDown();
        openGripper();
        servo_rotation.setPosition(servo_rot_down);

        servo_brat_sus.setPosition(0.54);
        servo_brat_jos.setPosition(0.04);
        robot_height = Height.LOW;
    }

    public void cone_down_low_fata(){
        liftDown();
        robot_height = Height.LOW;
        servo_rotation.setPosition(servo_rot_down);
        servo_brat_sus.setPosition(0.48);
        servo_brat_jos.setPosition(0.04);

    }

    public void cone_up_low_fata(){
        liftDown();
        robot_height = Height.LOW;

        servo_rotation.setPosition(servo_rot_down);
        servo_brat_jos.setPosition(0.60);
        servo_brat_sus.setPosition(1);
    } //jos mai jos

    public void cone_up_mid_fata(){
        if(robot_height.equals(Height.LOW)) {
            lift_pos_up(350);
            robot_height = Height.MEDIUM;
        }
        servo_rotation.setPosition(servo_rot_down);
        servo_brat_jos.setPosition(0.62);
        servo_brat_sus.setPosition(0.97);
    } //jos mai jos

    public void cone_up_high_fata(){
        if(robot_height.equals(Height.MEDIUM)){
            lift_pos_up(650);
            robot_height = Height.HIGH;
        }else if(robot_height.equals(Height.LOW)){
            lift_pos_up(650);
            robot_height = Height.HIGH;
        }
        servo_rotation.setPosition(servo_rot_down);
        servo_brat_jos.setPosition(0.42);
        servo_brat_sus.setPosition(0.78);
    }

    public void cone_up_low_spate(){
        liftDown();
        servo_rotation.setPosition(servo_rot_up);
        if(robot_height.equals(Height.LOW))
            lift_pos_up(300);
        servo_brat_jos.setPosition(0.9);
        servo_brat_sus.setPosition(0.74);
        robot_height = Height.LOW;
    }
    public void cone_up_mid_spate(){
        if(robot_height.equals(Height.LOW)) {
            lift_pos_up(550);
            robot_height = Height.MEDIUM;
        }
        servo_rotation.setPosition(servo_rot_up);
        servo_brat_jos.setPosition(0.9);
        servo_brat_sus.setPosition(0.63);
    }

    public void lift_up(){
        lift_pos_up(lift_ctrl.getCurrentPosition()+40);
    }
    public void lift_down(){
        lift_pos_down(lift_ctrl.getCurrentPosition()-40);
    }

    public void cone_up_high_spate(){
        if(robot_height.equals(Height.MEDIUM)){
            lift_pos_up(650);
            robot_height = Height.HIGH;
        }else if(robot_height.equals(Height.LOW)){
            lift_pos_up(650);
            robot_height = Height.HIGH;
        }
        servo_rotation.setPosition(servo_rot_up);
        servo_brat_jos.setPosition(0.97);
        servo_brat_sus.setPosition(0.62);
    }


    public void pickup_stack(int pos){
        switch(pos){
            case 1:
                servo_brat_jos.setPosition(0.25);
                servo_brat_sus.setPosition(0.65);
                break;
            case 2:
                servo_brat_jos.setPosition(0.2);
                servo_brat_sus.setPosition(0.7);
                break;
            case 3:
                servo_brat_jos.setPosition(0.15);
                servo_brat_sus.setPosition(0.70);
                break;
            case 4:
                servo_brat_jos.setPosition(0.1);
                servo_brat_sus.setPosition(0.75);
                break;
        }
        robot_height = Height.LOW;
    }
//    public void cone_up_spate(){
//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.6);
//    }
}
