package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalV4;

public class LinearHorizontalInferior extends LinearHorizontalV4 {

    public LinearHorizontalInferior(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.servoLinearHorizontal= hardwareMap.get(Servo.class, HardwareNames.horizontalInferiorServo);
        this.encoder = hardwareMap.get(DcMotorEx.class,HardwareNames.getHorizontalInferior);
    }
    public void monitor(Telemetry telemetry){
        super.monitor(telemetry,"Inferior");
    }
}
