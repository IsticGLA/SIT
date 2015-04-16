package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import java.util.HashMap;

import entity.IncidentCode;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by corentin on 16/04/15.
 */
public class GetAllIncidentCodeTask extends AsyncTask<Void, Void, IncidentCode[]> {
    private static final String TAG = GetAllIncidentCodeTask.class.getSimpleName();
    private int count = 0;
    private IIncidentCode activity;

    public GetAllIncidentCodeTask(IIncidentCode activity) {
        this.activity = activity;
    }

    public GetAllIncidentCodeTask(IIncidentCode activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected IncidentCode[] doInBackground(Void... params) {
        SpringService service = new SpringService();
        return service.codeSinistreClient();
    }

    @Override
    protected void onPostExecute(IncidentCode[] codes) {
        if(codes != null) {
            activity.getIncidentCode(codes);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetAllIncidentCodeTask(activity, count).execute();
            }
            else {
                activity.getIncidentCode(null);
                Toast.makeText(activity.getContext(), R.string.fail_get_interventions, Toast.LENGTH_SHORT).show();
            }
        }
    }
}
