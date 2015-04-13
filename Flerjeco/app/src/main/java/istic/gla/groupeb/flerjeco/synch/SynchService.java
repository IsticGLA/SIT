package istic.gla.groupeb.flerjeco.synch;

import android.app.IntentService;
import android.content.Intent;
import android.os.Bundle;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;

import java.util.Timer;
import java.util.TimerTask;

import istic.gla.groupeb.flerjeco.ISynchTool;
import istic.gla.groupeb.flerjeco.login.DisplaySynch;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by amhachi on 13/04/15.
 */
public class SynchService extends IntentService {

    private ISynchTool synchTool;

    public SynchService()
    {
        super("synchServices");

    }

    public void setSynchTool(ISynchTool synchTool) {
        this.synchTool = synchTool;
    }

    DisplaySynch displaySynch;
    Messenger messenger;
    Timer t=new Timer();


    @Override
    protected void onHandleIntent(final Intent intent) {
        displaySynch = (DisplaySynch) intent.getExtras().get("displaySynch");

        t.schedule(new TimerTask() {

            @Override
            public void run() {
                // just call the handler every 3 Seconds
                if(displaySynch != null)
                    displaySynch.ctrlDisplay();
                else Log.i("MAMH", " displaySynch == null");

            }
        }, 100,3000);

    }
}
