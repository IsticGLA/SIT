package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;

/**
 * Represents an asynchronous task used to get interventions
 */
public class GetInterventionTask extends AsyncTask<Void, Void, Boolean> {

    private static final String TAG = GetInterventionTask.class.getSimpleName();
    private int count = 0;
    private Intervention intervention;
    private IInterventionActivity activity;
    private Long id;
    private SpringService service = new SpringService();

    public GetInterventionTask(IInterventionActivity activity, Long id) {
        this.activity = activity;
        this.id = id;
    }

    public GetInterventionTask(IInterventionActivity activity, Long id, int count) {
        this.id = id;
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        intervention = null;
        try {
            Log.i(TAG, "doInBackground, id :" + id);
            intervention = service.getInterventionById(id);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage());
            return false;
        }

        if(intervention ==  null) {
            return false;
        }
        return true;
    }

    @Override
    protected void onPostExecute(final Boolean success) {
        if(success) {
            Log.i(TAG,"onPostExecute Success");
            activity.updateIntervention(intervention);
        }
        else {
            Log.i(TAG,"onPostExecute NOT Success");
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetInterventionTask(activity, id, count).execute();
            }
            else {
                activity.updateIntervention(null);
                Toast.makeText(activity.getContext(), R.string.fail_get_interventions, Toast.LENGTH_SHORT).show();
            }
        }
    }
}