package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

public class GarraInferior extends GarraV4 {

    public GarraInferior(HardwareMap hardwareMap) {

        super(hardwareMap);

    }


    public void monitor(Telemetry telemetry) {
        this.monitor(telemetry,"Inferior");
    }
}
