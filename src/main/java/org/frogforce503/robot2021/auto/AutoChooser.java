package org.frogforce503.robot2021.auto;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;

import org.frogforce503.lib.util.FFDashboard;
import org.json.simple.parser.ParseException;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

public class AutoChooser {
    private static final AutoChooser instance = new AutoChooser();
    private final HashMap<String, CommandGroupBase> autoMap = new HashMap<>();
    private final String autoBooleanKey = "autoGo";
    private final String autoNameKey = "autoName";
    CommandGroupBase selectedAuto = null;
    FFDashboard table;

    public static AutoChooser getInstance() {
        return instance;
    }

    public void resetClass() {
        selectedAuto = null;
        autoMap.clear();
        table = new FFDashboard("AutoSelect");
        try {
            populate();
        } catch (ParseException | IOException e) {
            e.printStackTrace();
        }
        addAutoListener();
        table.putBoolean(this.autoBooleanKey, false);
        table.putString(this.autoNameKey, "Drive Off Line");

        System.out.println("QUEUING AUTONOMOUS PATHS...");
        setAndInitAuto(new TestHolonomicPathAuto());
        System.out.println("FINISHED QUEUING... READY TO RUN");
    }

    private void populate() throws ParseException, IOException {

        // autoMap.put("Show Off 11 Ball Auto", new ShowOff11BallAuto().getGroup());
        // autoMap.put("Barrel Racing Path Auto", new BarrelRacingPathAuto());
        // autoMap.put("Slalom Path Auto", new SlalomPathAuto());
        // autoMap.put("Galactic Search Path A Auto", new GalacticSearchAAuto());
        // autoMap.put("Galactic Search Path B Auto", new GalacticSearchBAuto());
        // autoMap.put("Bounce Path Auto", new BouncePathAuto());

        String[] keys = new String[autoMap.size()];
        System.out.println("PRINT KEYS:");
        for (int i = 0; i < keys.length; i++) {
            keys[i] = autoMap.keySet().toArray()[i].toString();
            System.out.println(keys[i]);
        }
        System.out.println(Arrays.toString(keys));
        String autoListKey = "autoList";
        table.putStringArray(autoListKey, keys);
    }

    private void addAutoListener() {
        table.getEntry(autoBooleanKey).addListener(event -> {
            boolean bool = event.getEntry().getBoolean(false);
            if (bool) {
                String autoName = table.getString(autoNameKey, "Teleop Auto").trim();
                setAndInitAuto(autoMap.get(autoName));
                table.putString("Robot Selected Auto", autoName);
                event.getEntry().setBoolean(false);
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void setAndInitAuto(CommandGroupBase auto) {
        this.selectedAuto = auto;
    }

    public void runSelectedAuto() {
        if (this.selectedAuto == null) {
            this.selectedAuto = (new TestHolonomicPathAuto());
        }
        this.selectedAuto.schedule();
    }
}