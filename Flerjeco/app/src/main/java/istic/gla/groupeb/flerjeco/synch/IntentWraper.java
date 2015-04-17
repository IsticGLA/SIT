package istic.gla.groupeb.flerjeco.synch;

import android.content.Context;
import android.content.Intent;

import istic.gla.groupeb.flerjeco.FlerjecoApplication;

/**
 * Created by amhachi on 15/04/15.
 */
public class IntentWraper {

    protected static Intent intent ;
    protected static Context context;

    public static Intent getIntentInstance(){
        if(intent == null) {
            context = FlerjecoApplication.getInstance().getApplicationContext();
            intent = new Intent(context, SynchService.class);
        }
        return  intent;
    }

    public static void startService(String url, DisplaySynch displaySynch){
        getIntentInstance();

        intent.putExtra("displaySynch", displaySynch);
        intent.putExtra("url", url);

        context.startService(intent);
    }

    public static void startService(String url, DisplaySynch displaySynch, DisplaySynchDrone displaySynchDrone){
        getIntentInstance();

        intent.putExtra("displaySynch", displaySynch);
        intent.putExtra("url", url);

        intent.putExtra("displaySynchDrone", displaySynchDrone);

        context.startService(intent);
    }

    public static void stopService(){
        getIntentInstance();
        SynchService.stopTimerTask();
        context.stopService(intent);

        intent.removeExtra("displaySynch");
        intent.removeExtra("url");

        intent.removeExtra("displaySynchDrone");
    }
}
