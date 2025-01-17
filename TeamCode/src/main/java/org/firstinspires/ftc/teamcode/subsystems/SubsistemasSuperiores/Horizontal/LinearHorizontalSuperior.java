package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalV4;

public class LinearHorizontalSuperior extends LinearHorizontalV4 {

    public LinearHorizontalSuperior(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.servoLinearHorizontal= hardwareMap.get(Servo.class, HardwareNames.horizontalSuperiorServo);
    }
    public void monitor(Telemetry telemetry){
        super.monitor(telemetry,"Superior");
    }
}
