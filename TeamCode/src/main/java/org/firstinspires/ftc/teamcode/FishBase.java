////package org.firstinspires.ftc.teamcode.FishBase;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//public abstract class FishBase extends LinearOpMode {
//    public static final int FIELD_LENGTH_IN_MILLIMETER = 3590;
//    public static final int SHIPPING_HUB_RADIUS_IN_MILLIMETER = 226;
//    public static final int CAROUSAL_RADIUS_IN_MILLIMETER = 191;
//    public static final int RABBIT_POSITION_MIN_IN_MILLIMETER = 700;
//    public static final int RABBIT_POSITION_MAX_IN_MILLIMETER = 800;
//    public static final int HALF_CHASSIS_WIDTH_IN_MILLIMETER = 114;
//    public static final int HALF_CHASSIS_LENGTH_IN_MILLIMETER = 219;
//    public static final int BLUE_SHIPPING_HUB_X_IN_MILLIMETER = 1187;
//    public static final int BLUE_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
//    public static final int RED_SHIPPING_HUB_X_IN_MILLIMETER = 2375;
//    public static final int RED_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
//    public static final int STORAGE_START_X_IN_MILLIMETER = 630;
//    public static final int STORAGE_START_Y_IN_MILLIMETER = 0;
//    public static final int STORAGE_END_X_IN_MILLIMETER = 1150;
//    public static final int STORAGE_END_Y_IN_MILLIMETER = 630;
//    public static final int STORAGE_CENTER_X_IN_MILLIMETER = 890;
//    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_RIGHT_DISTANCE_SENSOR = 100;
//    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_FRONT_DISTANCE_SENSOR = 100;
//    public static final double ANGLE_RATIO = 1.4;
//    public static final double SLIDE_RATIO = 1.2;
//    public static final double CHASSIS_RADIUS = 203;
//    public static final double COUNTS_PER_ENCODER_REV = 28;
//    public static final double WHEEL_GEAR_RATIO = 19.203208556149733;
//    public static final double ARM_GEAR_RATIO = 139.13824192336588;
//    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
//    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
//    public static final int ARM_MOTOR_SPPED_IN_RPM = 43;
//    public static final double WHEEL_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO / 360;
//    public static final double WHEEL_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO * WHEEL_MOTOR_SPPED_IN_RPM;
//    public static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO / 360;
//    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPPED_IN_RPM;
//    public static final double COUNTS_PER_MILLIMETER = (COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO) / (WHEEL_DIAMETER_MILLIMETTER * Math.PI);
//    //    public DistanceSensor frontDistanceSensor;
////    public DistanceSensor leftDistanceSensor;
////    public DistanceSensor rightDistanceSensor;
//    public DcMotorEx motorFrontLeft;
//    public DcMotorEx motorFrontRight;
//    public DcMotorEx motorBackLeft;
//    public DcMotorEx motorBackRight;
//    //    public void slide(int distanceInMillimeter, double speed) {
////        int direction = 1;
////        int distanceInCounts = (int) (distanceInMillimeter * COUNTS_PER_MILLIMETER);
////        this.setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        this.motorFrontRight.setTargetPosition(distanceInCounts);
////        this.motorFrontLeft.setTargetPosition(distanceInCounts);
////        this.motorBackRight.setTargetPosition(-distanceInCounts);
////        this.motorBackLeft.setTargetPosition(-distanceInCounts);
////        this.setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
////        this.motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
////        this.motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
////        this.motorBackRight.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
////        this.motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
////        while (this.isStillDriving()) {
////            // this.telemetry.addData("Front right motor speed", this.motorFrontRight.getVelocity());
////            // this.telemetry.addData("Front left motor speed", this.motorFrontLeft.getVelocity());
////            // this.telemetry.addData("Back right speed", this.motorBackRight.getVelocity());
////            // this.telemetry.addData("Back left speed", this.motorBackLeft.getVelocity());
////            // this.telemetry.update();
////        }
////    }
//    public void setDrivingMotorMode(DcMotor.RunMode mode) {
//        motorFrontRight.setMode(mode);
//        motorFrontLeft.setMode(mode);
//        motorBackRight.setMode(mode);
//        motorBackLeft.setMode(mode);
//    }
//    public boolean isStillDriving() {
//        return motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy();
//    }
//    protected void driveDistance(int distanceInMilliMeter, double speed) {
//        if(distanceInMilliMeter == 0) return;
//        int direction = distanceInMilliMeter / Math.abs(distanceInMilliMeter);
//        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
//        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setTargetPosition(distanceInCounts);
//        motorFrontLeft.setTargetPosition(-distanceInCounts);
//        motorBackRight.setTargetPosition(distanceInCounts);
//        motorBackLeft.setTargetPosition(-distanceInCounts);
//        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        motorFrontLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        while (isStillDriving()) {
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update();
//        }
//    }
//
//    public void turn(double angle, double speed) {
//        if (angle == 0.0) return;
//        int direction = ((int) (100 * angle)) / Math.abs((int) (100 * angle));
//
//        double distanceInMilliMeter = Math.toRadians(angle) * CHASSIS_RADIUS;
//        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
//        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setTargetPosition(distanceInCounts);
//        motorFrontLeft.setTargetPosition(distanceInCounts);
//        motorBackRight.setTargetPosition(distanceInCounts);
//        motorBackLeft.setTargetPosition(distanceInCounts);
//        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        motorFrontLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
//        while (isStillDriving()) {
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update();
//        }
//    }
//}
