package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz TeleOp BLUE AUDIENCE", group = "A")
public class TeleOpBlueAudience extends TeleOpBlue {
    @Override
    protected StartingSide fixedStartingSide() {
        return StartingSide.AUDIENCE;
    }
}
