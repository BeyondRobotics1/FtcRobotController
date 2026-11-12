package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * TriggerGroup group pollen & nectar triggers together
 * so they can be open/close through one method call
 */
public class TriggerGroup {
    Trigger pollenTrigger;
    Trigger nectarTrigger;

    public TriggerGroup(HardwareMap hardwareMap)
    {
        pollenTrigger = new Trigger(hardwareMap, "pollenTrigger",
                Trigger.POLLEN_TRIGGER_OPEN_POSITION,
                Trigger.POLLEN_TRIGGER_CLOSED_POSITION, true);

        nectarTrigger = new Trigger(hardwareMap, "nectarTrigger",
                Trigger.NECTAR_TRIGGER_OPEN_POSITION,
                Trigger.NECTAR_TRIGGER_CLOSED_POSITION, true);

        close();
    }

    public void open()
    {
        pollenTrigger.open();
        nectarTrigger.open();
    }

    public void close()
    {
        pollenTrigger.close();
        nectarTrigger.close();
    }

    public void openPollenTrigger()
    {
        pollenTrigger.open();
    }

    public void openNectarTrigger()
    {
        nectarTrigger.open();
    }

    public void closePollenTrigger()
    {
        pollenTrigger.close();
    }

    public void closeNectarTrigger()
    {
        nectarTrigger.close();
    }

}
