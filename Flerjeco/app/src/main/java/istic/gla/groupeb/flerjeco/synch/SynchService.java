package istic.gla.groupeb.flerjeco.synch;

import android.app.IntentService;
import android.content.Intent;
import android.os.Bundle;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;

import java.util.Timer;
import java.util.TimerTask;

import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by amhachi on 13/04/15.
 */
public class SynchService extends IntentService {

    public SynchService()
    {
        super("myintentservice");
    }


    Messenger messenger;
    Timer t=new Timer();


    @Override
    protected void onHandleIntent(Intent intent) {
        messenger=(Messenger) intent.getExtras().get("handler");

        t.schedule(new TimerTask() {

            @Override
            public void run() {
                // just call the handler every 3 Seconds

                SpringService springService = new SpringService();
                Message msg=Message.obtain();
                Bundle data=new Bundle();
                data.putString("k", ""+springService.getNotify() );
                msg.setData(data);

                try {
                    messenger.send(msg);
                } catch (RemoteException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }, 100,3000);

    }
}
