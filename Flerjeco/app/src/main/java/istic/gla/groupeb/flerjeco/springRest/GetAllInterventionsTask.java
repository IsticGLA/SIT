package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import org.springframework.http.ResponseEntity;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;

/**
 * Represents an asynchronous task used to get interventions
 */
public class GetAllInterventionsTask extends AsyncTask<Void, Void, ResponseEntity<Intervention[]>> {

    private static final String TAG = GetAllInterventionsTask.class.getSimpleName();
    private int count = 0;
    private IInterventionsActivity activity;
    private SpringService service = new SpringService();

    public GetAllInterventionsTask(IInterventionsActivity activity) {
        this.activity = activity;
    }

    public GetAllInterventionsTask(IInterventionsActivity activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected ResponseEntity<Intervention[]> doInBackground(Void... params) {
        return service.getAllInterventions();
    }

    @Override
    protected void onPostExecute(ResponseEntity<Intervention[]> response) {
        switch(response.getStatusCode()){
            case OK:
                if(response.getBody() != null){
                    Log.v(TAG, "got interventions : " + response.getBody().length);
                    activity.updateInterventions(response.getBody());
                } else{
                    Log.v(TAG, "got 0 interventions ");
                    activity.updateInterventions(new Intervention[0]);
                }

                break;
            default:
                count++;
                if(count < 4) {
                    Log.i(TAG, "Retry: " + count);
                    new GetAllInterventionsTask(activity, count).execute();
                }
                else {
                    activity.updateInterventions(null);
                    Toast.makeText(activity.getContext(), R.string.fail_get_interventions, Toast.LENGTH_SHORT).show();
                }
        }
    }
}