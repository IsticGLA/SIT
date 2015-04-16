package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;

/**
 * Represents an asynchronous task used to get interventions
 */
public class GetAllInterventionsTask extends AsyncTask<Void, Void, Boolean> {

    private static final String TAG = GetAllInterventionsTask.class.getSimpleName();
    private int count = 0;
    private Intervention[] interventionTab;
    private IInterventionsActivity activity;

    public GetAllInterventionsTask(IInterventionsActivity activity) {
        this.activity = activity;
    }

    public GetAllInterventionsTask(IInterventionsActivity activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        SpringService service = new SpringService();
        interventionTab = service.getAllInterventions();
        if(interventionTab ==  null) {
            return false;
        }
        Log.i(TAG, "interventionTab size : " + interventionTab.length);
        Log.i(TAG, "doInBackground end");
        return true;
    }

    @Override
    protected void onPostExecute(final Boolean success) {
        if(success) {
            activity.showProgress(false);
            activity.updateInterventions(interventionTab);
        }
        else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetAllInterventionsTask(activity, count).execute();
            }
            else {
                activity.showProgress(false);
                Toast.makeText(activity.getContext(), R.string.fail_get_interventions, Toast.LENGTH_SHORT).show();
            }
        }
    }
}