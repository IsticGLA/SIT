package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import istic.gla.groupb.nivimoju.entity.ResourceType;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by jules on 18/05/15.
 */
public class GetResourceTypesTask extends AsyncTask<Void, Void, ResourceType[]> {
    private static final String TAG = GetResourceTypesTask.class.getSimpleName();
    private IResourceTypesActivity activity;
    private SpringService service = new SpringService();
    private int count = 0;

    public GetResourceTypesTask(IResourceTypesActivity activity) {
        this.activity = activity;
    }

    public GetResourceTypesTask(IResourceTypesActivity activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected ResourceType[] doInBackground(Void... params) {
        ResourceType[] resourceTypes;
        try {
            Log.i(TAG, "GetResourceTypes start");
            resourceTypes = service.getResourceTypes();
        } catch (Exception e) {
            Log.e(TAG, e.getMessage());
            return null;
        }
        Log.i(TAG, "GetResourceTypes end");
        return resourceTypes;
    }

    @Override
    protected void onPostExecute(ResourceType[] resourceTypes) {
        if(resourceTypes != null) {
            activity.updateResourceTypes(resourceTypes);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetResourceTypesTask(activity, count).execute();
            }
            else {
                activity.updateResourceTypes(null);
                Toast.makeText(activity.getContext(), R.string.fail_get_resource_type, Toast.LENGTH_SHORT).show();
            }
        }
    }
}
