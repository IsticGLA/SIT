package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;

import org.springframework.web.client.HttpStatusCodeException;

import istic.gla.groupb.nivimoju.entity.Intervention;

/**
 * Created by corentin on 20/05/15.
 */
public class UpdateResourceTask extends AsyncTask<String, Void, Intervention> {
    private static final String TAG = UpdateResourceTask.class.getSimpleName();
    private SpringService service = new SpringService();
    private IResourceActivity activity;

    public UpdateResourceTask(IResourceActivity activity) {
        this.activity = activity;
    }

    @Override
    protected Intervention doInBackground(String... params) {
        Log.i(TAG, "doInBackground start");
        try {
            return service.changeResourceState(params);
        } catch (HttpStatusCodeException e) {
            Log.e(TAG, e.getMessage(), e);
        }

        return null;
    }

    @Override
    protected void onPostExecute(Intervention intervention) {
        activity.updateResources(intervention);
        Log.i(TAG, "Request returned");
    }

}