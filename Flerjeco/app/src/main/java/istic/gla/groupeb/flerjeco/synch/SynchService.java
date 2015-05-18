package istic.gla.groupeb.flerjeco.synch;

import android.app.IntentService;
import android.content.Intent;
import android.database.sqlite.SQLiteDatabase;
import android.os.AsyncTask;

import java.sql.Timestamp;
import java.util.Timer;
import java.util.TimerTask;

import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by amhachi on 13/04/15.
 */
public class SynchService extends IntentService {

    private ISynchTool synchTool;

    protected int timer;
    protected int timerDrone;

    public SynchService() {
        super("synchServices");
        timer = 5000;
        timerDrone = 1000;
    }

    public void setSynchTool(ISynchTool synchTool) {
        this.synchTool = synchTool;
    }

    DisplaySynch displaySynch;
    DisplaySynchDrone displaySynchDrone;

    String url;
    Timestamp timestamp = new Timestamp(0);
    SpringService springService = new SpringService();
    static Timer t = new Timer();
    static Timer tDrone = new Timer();
    static TimerTask timerTask;
    static TimerTask timerTaskDrone;

    public static void stopTimerTask() {
        timerTask.cancel();
        if (timerTaskDrone != null) {
            timerTaskDrone.cancel();
            timerTaskDrone = null;
        }
    }


    @Override
    protected void onHandleIntent(final Intent intent) {
        displaySynch = (DisplaySynch) intent.getExtras().get("displaySynch");
        displaySynchDrone = (DisplaySynchDrone) intent.getExtras().get("displaySynchDrone");
        url = (String) intent.getExtras().get("url");

        t.cancel();
        t.purge();
        t = new Timer();

        tDrone.cancel();
        tDrone.purge();
        tDrone = new Timer();

        final GetNotifyTask getNotifyTask = new GetNotifyTask();

        timerTask = new TimerTask() {

            @Override
            public void run() {
                new GetNotifyTask().execute(url);
            }
        };
        t.schedule(timerTask, 100, timer);

        if (displaySynchDrone != null) {
            timerTaskDrone = new TimerTask() {

                @Override
                public void run() {
                    displaySynchDrone.ctrlDisplay();
                }
            };
            tDrone.schedule(timerTaskDrone, 100, timerDrone);
        }

    }

    // Backgroud task to post intervention
    private class GetNotifyTask extends AsyncTask<String, Void, Timestamp> {

        @Override
        protected Timestamp doInBackground(String... params) {
            try {
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
                //Log.i("MAMH", " displaySynchDrone == null");
            }
        }

    }

    @Override
    public SQLiteDatabase openOrCreateDatabase(String name, int mode, SQLiteDatabase.CursorFactory factory) {
        return super.openOrCreateDatabase(name, mode, factory);
    }
}
