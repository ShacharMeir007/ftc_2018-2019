package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.RunnableFuture;

/**
 * Created by shach on 11/14/2018.
 */

public abstract class OpModeRobot_Iterative extends OpMode {
     Robot Shachar =new Robot();
     public void print(String caption,String value){
          telemetry.addData(caption,value);
          telemetry.update();
     }


     Runnable Remote_Drive_Tank(DcMotor leftMotor, DcMotor rightMotor,double lim){
          double x;
          double y;
          double absoluteValue;


          y = lim*Math.pow(gamepad1.left_stick_y,3);
          x = -lim*Math.pow(gamepad1.left_stick_x,3);
          absoluteValue = Math.abs( Math.max(Math.abs(y+x), Math.abs(x-y)));


          if (absoluteValue>1){
               Shachar.driveTrain.leftMotor.setPower((x-y)/ absoluteValue );
               Shachar.driveTrain.rightMotor.setPower(x+y / absoluteValue );
          } else {
               Shachar.driveTrain.leftMotor.setPower(x-y);
               Shachar.driveTrain.rightMotor.setPower(x+y);
          }
          return null;
     }
}
