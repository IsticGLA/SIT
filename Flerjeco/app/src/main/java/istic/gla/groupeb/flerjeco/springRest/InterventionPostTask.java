package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;


import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by corentin on 16/04/15.
 */
// Backgroud task to post intervention
public class InterventionPostTask extends AsyncTask<Intervention, Void, Intervention> {
    private static final String TAG = InterventionPostTask.class.getSimpleName();
    private int count = 0;
    private Intervention intervention;
    private IInterventionActivity activity;
    private SpringService service = new SpringService();

    public InterventionPostTask(IInterventionActivity activity) {
        this.activity = activity;
    }

    public InterventionPostTask(IInterventionActivity activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected entity.Intervention doInBackground(entity.Intervention... params) {
        intervention = params[0];
        return service.postIntervention(intervention);
    }

    @Override
    protected void onPostExecute(entity.Intervention resultPost) {
        Log.i(TAG, "InterventionPostTask onPostExecute");
        if(resultPost != null) {
            activity.updateIntervention(resultPost);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new InterventionPostTask(activity, count).execute(intervention);
            }
            else {
                activity.updateIntervention(null);
                Toast.makeText(activity.getContext(), R.string.fail_post_interventions, Toast.LENGTH_SHORT).show();
            }
        }
    }
}