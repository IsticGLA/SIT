package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import istic.gla.groupb.nivimoju.entity.IncidentCode;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by corentin on 16/04/15.
 */
public class GetAllIncidentCodeTask extends AsyncTask<Void, Void, IncidentCode[]> {
    private static final String TAG = GetAllIncidentCodeTask.class.getSimpleName();
    private int count = 0;
    private IIncidentCode activity;
    private SpringService service = new SpringService();

    public GetAllIncidentCodeTask(IIncidentCode activity) {
        this.activity = activity;
    }

    public GetAllIncidentCodeTask(IIncidentCode activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected IncidentCode[] doInBackground(Void... params) {
        try {
            return service.codeSinistreClient();
        } catch (Exception e){
            Log.e(TAG, "ERROR " + e);
        }
        return null;
    }

    @Override
    protected void onPostExecute(IncidentCode[] codes) {
        if(codes != null) {
            activity.updateIncidentCodes(codes);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetAllIncidentCodeTask(activity, count).execute();
            }
            else {
                activity.updateIncidentCodes(null);
                Toast.makeText(activity.getContext(), R.string.fail_get_incident_code, Toast.LENGTH_SHORT).show();
            }
        }
    }
}
