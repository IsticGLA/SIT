package istic.gla.groupeb.flerjeco.synch;

import android.content.Context;
import android.content.Intent;

/**
 * Created by amhachi on 15/04/15.
 */
public class IntentWraper {

    private static Intent intent ;

    public static Intent getIntentInstance(Context context){
        if(intent == null) {
            intent = new Intent(context, SynchService.class);
        }
        return  intent;
    }
}
