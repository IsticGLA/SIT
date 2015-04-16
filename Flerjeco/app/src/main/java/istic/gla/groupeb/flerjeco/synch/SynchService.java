package istic.gla.groupeb.flerjeco.synch;

import android.app.IntentService;
import android.content.Intent;
import android.database.sqlite.SQLiteDatabase;
import android.os.AsyncTask;
import android.os.Messenger;
import android.util.Log;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import java.sql.Timestamp;
import java.util.Timer;
import java.util.TimerTask;

import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by amhachi on 13/04/15.
 */
public class SynchService extends IntentService {

    private ISynchTool synchTool;

    public SynchService() {
        super("synchServices");

    }

    public void setSynchTool(ISynchTool synchTool) {
        this.synchTool = synchTool;
    }

    DisplaySynch displaySynch;
    String url;
    Timestamp timestamp = new Timestamp(0);
    //TODO SpringService
    SpringService springService = new SpringService();
    Messenger messenger;
    static Timer t = new Timer();
    static TimerTask timerTask;

    public static void stopTimerTask() {
        timerTask.cancel();
    }


    @Override
    protected void onHandleIntent(final Intent intent) {
        displaySynch = (DisplaySynch) intent.getExtras().get("displaySynch");
        url = (String) intent.getExtras().get("url");

        t.cancel();
        t.purge();
        t = new Timer();

        final GetNotifyTask getNotifyTask = new GetNotifyTask();

        timerTask = new TimerTask() {

            @Override
            public void run() {
                new GetNotifyTask().execute(url);
            }
        };

        t.schedule(timerTask, 100, 5000);

    }

    // Backgroud task to post intervention
    private class GetNotifyTask extends AsyncTask<String, Void, Timestamp> {

        @Override
        protected Timestamp doInBackground(String... params) {
            try {
                //Log.i("MAMH", "ID inter : "+params[0]);
                return springService.getNotify(params[0], timestamp);
            } catch (Exception e) {
                //Log.e("InterventionActivity", e.getMessage(), e);
                return null;
            }

        }

        @Override
        protected void onPostExecute(Timestamp resultPost) {

            // just call the handler every 3 Seconds
            if(resultPost == null){
//                Log.i("MAMH", "resultPost est null ");
            }else
            if(resultPost.equals(timestamp)){
//                Log.i("MAMH", "SynchService : l'intervention est à jour");
            }
            else if (displaySynch != null && timestamp.before(resultPost)) {
                timestamp = resultPost;
                displaySynch.ctrlDisplay();
//                Log.i("MAMH", "SynchService : l'intervention se mis à jour");
            } else if (displaySynch == null)
            {
                //Log.i("MAMH", " displaySynch == null");
            }
        }

    }

    @Override
    public SQLiteDatabase openOrCreateDatabase(String name, int mode, SQLiteDatabase.CursorFactory factory) {
        return super.openOrCreateDatabase(name, mode, factory);
    }
}
