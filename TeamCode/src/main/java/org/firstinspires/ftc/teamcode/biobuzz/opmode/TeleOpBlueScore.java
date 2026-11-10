package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz TeleOp BLUE SCORE", group = "A")
public class TeleOpBlueScore extends TeleOpBlue {
    @Override
    protected StartingSide fixedStartingSide() {
        return StartingSide.SCORE;
    }
}
