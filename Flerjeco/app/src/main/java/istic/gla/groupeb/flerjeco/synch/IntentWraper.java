package istic.gla.groupeb.flerjeco.synch;

import android.content.Context;
import android.content.Intent;

import istic.gla.groupeb.flerjeco.MyApp;

/**
 * Created by amhachi on 15/04/15.
 */
public class IntentWraper {

    private static Intent intent ;
    private static Context context;

    public static Intent getIntentInstance(){
        if(intent == null) {
            context = MyApp.getInstance().getApplicationContext();
            intent = new Intent(context, SynchService.class);
        }
        return  intent;
    }

    public static void startService(String url, DisplaySynch displaySynch){
        getIntentInstance();
        context.stopService(intent);

        intent.removeExtra("displaySynch");
        intent.removeExtra("url");

        intent.putExtra("displaySynch", displaySynch);
        intent.putExtra("url", url);

        context.startService(intent);
    }
}
