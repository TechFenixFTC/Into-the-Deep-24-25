package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Disabled
@TeleOp(name = "Teste horizontal novo ")
public class horizontal extends OpMode{

    DcMotor Horizontal;

    @Override
    public void init(){
        Horizontal =  hardwareMap.get(DcMotorEx.class, "horizontal");
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop(){
        if(gamepad2.left_trigger > 0){
            Horizontal.setPower(gamepad2.left_trigger);

        } else if (gamepad2.right_trigger > 0) {
            Horizontal.setPower(gamepad2.left_trigger * -1 );
        }

        telemetry.addData("horizontal position", Horizontal.getCurrentPosition());
        telemetry.update();
    }
}

