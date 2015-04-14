package istic.gla.groupeb.flerjeco.synch;

import android.app.IntentService;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Messenger;
import android.util.Log;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import java.util.Timer;
import java.util.TimerTask;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.codis.intervention.InterventionActivity;
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
    Intervention intervention;
    SpringService springService;
    Messenger messenger;
    Timer t=new Timer();


    @Override
    protected void onHandleIntent(final Intent intent) {
        displaySynch = (DisplaySynch) intent.getExtras().get("displaySynch");
        intervention = (Intervention) intent.getExtras().get("intervention");
        final GetNotifyTask getNotifyTask = new GetNotifyTask();

        t.schedule(new TimerTask() {

            @Override
            public void run() {

                Log.i("MAMH", "ID : "+intervention.getId());
                getNotifyTask.execute(intervention);
            }
        }, 100,10000);

    }

    // Backgroud task to post intervention
    private class GetNotifyTask extends AsyncTask<Intervention, Void, String> {

        @Override
        protected String doInBackground(entity.Intervention... params) {
            try {
                Log.i("MAMH", "ID inter : "+params[0]);
                return springService.getNotify(params[0]);
            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);
                return null;
            }

        }

        @Override
        protected void onPostExecute(String resultPost) {
            // just call the handler every 3 Seconds
            if(displaySynch != null && "210".equals(resultPost)) {
                displaySynch.ctrlDisplay();
            }
            else if ("210".equals(resultPost)) { Log.i("MAMH", "SynchService : l'intervention est à jour");}
                else Log.i("MAMH", " displaySynch == nusll");
        }

    }
}
