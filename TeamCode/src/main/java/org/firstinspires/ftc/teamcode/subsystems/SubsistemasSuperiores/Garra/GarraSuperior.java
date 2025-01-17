package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

public class GarraSuperior extends GarraV4 {

    public GarraSuperior(HardwareMap hardwareMap) {

        super(hardwareMap);

    }

    @Override
    public void monitor(Telemetry telemetry) {
        super.monitor(telemetry,"Superior");
    }
}
