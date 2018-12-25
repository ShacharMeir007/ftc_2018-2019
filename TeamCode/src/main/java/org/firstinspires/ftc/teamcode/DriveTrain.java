package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by shach on 10/25/2018.
 * interface for class{@link DriveTrain_Interface}
 * This Class describes the driveTrain of the robot
 */
public class DriveTrain implements DriveTrain_Interface {
    private ElapsedTime runtime = new ElapsedTime();
    enum Shape {BALL,CUBE}
    enum Place {LEFT,CENTER,RIGHT}
    //Constants
    private final int ENCODERS = 1120;
    private final int DIAMETER = 10;
    private final double ENCODERS_PER_CM = ENCODERS /(DIAMETER*Math.PI);
    private final double ENCODERS_PER_DEGREE = 11.45;

    //Motors Objects
    DcMotor leftMotor = null;//port 0 MOTOR
    DcMotor rightMotor = null;//port 1 MOTOR
    Servo lightServo =null;//port 0 SERVO
    Servo lightServo2 = null;
    // sensor objects
    BNO055IMU imu;
    Orientation angles;

    ColorSensor colorSensor = null;//port 1 I2C
    //PID variables
    private static int p1=0;
    private static int p2=0;
    private static double i=0;
    private static double d=0;
    private int deltaLeftEncoders = 0;
    private int deltaRightEncoders =0;


    public void DriveTrain(){}
    /**************************************************************************************
    *Function name: init
    *Input: HardwareMap of an opMode
    *Output: void
    *Operation: initialize the hardware of the drive train
    ***************************************************************************************/
    @Override
    public void init(HardwareMap hwm){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu =hwm.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        leftMotor = hwm.get(DcMotor.class, "leftMotor");
        rightMotor = hwm.get(DcMotor.class, "rightMotor");
        colorSensor = hwm.get(ColorSensor.class, "colorSensor");
        lightServo = hwm.get(Servo.class,"lightServo");
        lightServo2 = hwm.get(Servo.class,"lightServo2");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lightServo.setPosition(0.75);
        lightServo2.setPosition(0.53);

    }
     /***************************************************************************************
     *Function name: driveForward                                                           *
     *Input: a power value and a distance value                                             *
     *Output: nothing                                                                       *
     *Operation: drive in the given power the distance given                                *
     ****************************************************************************************/
    public void driveForward(int cm, double power) {
        int target =(int) Math.floor (cm * ENCODERS_PER_CM);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(-target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){
            leftMotor.setPower(power);
            rightMotor.setPower(-power);

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void lightSensorUp (){
        lightServo.setPosition(0.75);
        lightServo2.setPosition(0.53);
    }
    public void lightSensorDown (){
        lightServo2.setPosition(0.0);
        lightServo.setPosition(0.0);
    }


    /**************************************************************************************
     *Function name: PID_SpeedCalculator
     *Input: a target power
     *Output: a power value after the PID controller's calculation
     *Operation: measures errors of the encoders values and returns a power
     * value accordingly
     ***************************************************************************************/
    private final double kP =0.003;
    private final double kI =0.001;
    private final double kD=0.001;
    private double PID_SpeedCalculator(double targetPower) {
        if (leftMotor.isBusy()&& rightMotor.isBusy ()) {
            double result;
            deltaLeftEncoders = leftMotor.getCurrentPosition()- deltaLeftEncoders;
            deltaRightEncoders = -rightMotor.getCurrentPosition()- deltaRightEncoders;
            p2 =p1;
            p1 = (deltaRightEncoders) - (deltaLeftEncoders);
            if (Math.abs(leftMotor.getPower())<0.9) {
                i += p1;
            }
            d= p1-p2;
            result =targetPower+ kP*p1 + kI*i + kD*d;
            if (Math.abs(result)<=1) {
                return result;
            }
            else if (result>0){
                return 1;
            }
            else {
                return -1;
            }

        }
        return 0;
    }
    private double PID_SpeedCalculator_TURN(double targetPower) {
        if (leftMotor.isBusy()&& rightMotor.isBusy ()) {
            double result;
            deltaLeftEncoders = leftMotor.getCurrentPosition()- deltaLeftEncoders;
            deltaRightEncoders = rightMotor.getCurrentPosition()- deltaRightEncoders;
            p2 =p1;
            p1 = (deltaRightEncoders) - (deltaLeftEncoders);
            if (Math.abs(leftMotor.getPower())<0.9) {
                i += p1;
            }
            d= p1-p2;
            result =targetPower+ kP*p1 + kI*i + kD*d;
            if (Math.abs(result)<=1) {
                return result;
            }
            else if (result>0){
                return 1;
            }
            else {
                return -1;
            }

        }
        return 0;
    }
    /**************************************************************************************
     *Function name: resetPID
     *Input: nothing
     *Output: void
     *Operation: resets PID variables
     ***************************************************************************************/
    private void resetPID(){
        p1=0;
        p2=0;
        i=0;
        d=0;
        deltaLeftEncoders = 0;
        deltaRightEncoders =0;
    }
    /**************************************************************************************
     *Function name: driveForward_PID
     *Input: a power value and a distance value
     *Output: void
     *Operation: drives forward a given distance in a given power while changing the
     * power of the left motor (using PID)in order to truly drive straight
     ***************************************************************************************/
    void driveForward_PID(double cm, double power) {
        int target =(int) Math.floor (cm * ENCODERS_PER_CM);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(-target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){

            leftMotor.setPower(PID_SpeedCalculator(power));
            rightMotor.setPower(-power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetPID();
    }

    /**************************************************************************************
     *Function name: turnRight_ENCODERS
     *Input: a power value and a degree value
     *Output: void
     *Operation: turn to the right in the given power the given amount of degrees
     ***************************************************************************************/
    void turnLeft_ENCODERS(double power, int degrees) {

        int target =(int) Math.floor (degrees * ENCODERS_PER_DEGREE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){
            leftMotor.setPower(PID_SpeedCalculator(power));
            rightMotor.setPower(power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetPID();
    }

    /**
     *
     * @param power the power for the motors
     * @param degrees the amount of degrees to turn
     */
    void turnRight_ENCODERS(double power, int degrees) {

        int target =(int) Math.floor (degrees * ENCODERS_PER_DEGREE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(-target);
        rightMotor.setTargetPosition(-target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetPID();
}
void turn_Gyro(float degrees,double power){
    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle<degrees){
            while (angles.firstAngle<degrees){
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
        }
        else{
            while (angles.firstAngle>degrees){
                leftMotor.setPower(-power);
                rightMotor.setPower(-power);
            }
        }
}
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
void cubeMission (int i){
    switch (i){
        case 0:
            //driveForward(5, 0.3);
            sleep(100);
            lightSensorUp(); //arm up
            sleep(100);
            driveForward_PID(-5,-0.2);
            turnLeft_ENCODERS(0.3, 95);
            sleep(100);
            driveForward(100, 0.7);

            break;
        case 1:
            sleep(100);
            lightSensorUp(); //arm up
            sleep(100);
            driveForward_PID(-5,-0.2);
            turnLeft_ENCODERS(0.3, 110);
            sleep(100);
            driveForward(100, 0.7);

            break;
        case 2:
            sleep(100);
            lightSensorUp(); //arm up
            sleep(100);
            driveForward_PID(-5,-0.2);
            turnLeft_ENCODERS(0.3, 150);
            sleep(100);
            driveForward(100, 0.7);
            break;
        default:
                break;
    }
}
    /**
     * this function uses the color sensor to determine the shape
     * in front of the color sensor.
     * @param telemetry the telemetry of the op Mode.
     * @return the shape detected.
     */
    Shape ShapeCheck(Telemetry telemetry,ElapsedTime runtime){
        Shape shape =Shape.BALL;
        colorSensor.enableLed(true);
        telemetry.addData("color value: ", colorSensor.argb());
        telemetry.addData("red: ",colorSensor.red());
        telemetry.addData("green: ",colorSensor.green());
        telemetry.addData("blue: ",colorSensor.blue());
        telemetry.addData("alpha: ",colorSensor.alpha());
        telemetry.addData("argb: ",colorSensor.argb());
        runtime.reset();
        while (runtime.seconds()<1)
        if((double)colorSensor.blue()/colorSensor.red()<0.4){
            telemetry.addData("shape: ","Cube");
            telemetry.update();
            shape= Shape.CUBE;
        }
        else {
            telemetry.addData("shape: ","Ball");
            telemetry.update();
            shape=Shape.BALL;
        }
        return shape;
    }


}

