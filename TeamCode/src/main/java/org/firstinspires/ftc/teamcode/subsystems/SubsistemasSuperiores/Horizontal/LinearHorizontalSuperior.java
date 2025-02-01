package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontal;

public class LinearHorizontalSuperior extends LinearHorizontal {

    public LinearHorizontalSuperior(HardwareMap hardwareMap) {
        super(hardwareMap, HardwareNames.horizontalSuperiorServo);
        mapStateHorizontal.put(LinearHorizontalStates.EXTENDED,0.634);
        mapStateHorizontal.put(LinearHorizontalStates.RETRACTED,0.121);

    }
    public void monitor(Telemetry telemetry){
        this.monitor(telemetry,"Superior");
    }

}
