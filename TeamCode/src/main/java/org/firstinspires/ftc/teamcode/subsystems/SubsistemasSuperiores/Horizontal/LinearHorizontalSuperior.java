package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalV4;

public class LinearHorizontalSuperior extends LinearHorizontalV4 {

    public LinearHorizontalSuperior(HardwareMap hardwareMap) {
        super(hardwareMap, HardwareNames.horizontalSuperiorServo);
        mapStateHorizontal.put(LinearHorizontalStates.EXTENDED,0.634);
        mapStateHorizontal.put(LinearHorizontalStates.RETRACTED,0.121);

    }
    public void monitor(Telemetry telemetry){
        this.monitor(telemetry,"Superior");
    }

    public Action goToRetracted() {return super.goToRetracted();}


    public Action goToExtended() {return  super.goToExtended();}
}
