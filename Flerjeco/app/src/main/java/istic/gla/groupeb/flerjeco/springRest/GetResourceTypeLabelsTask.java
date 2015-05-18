package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import java.util.List;

import entity.ResourceType;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by corentin on 16/04/15.
 * Background task to get Resource Type
 */
public class GetResourceTypeLabelsTask extends AsyncTask<List<Long>, Void, ResourceType[]> {
    private static final String TAG = GetResourceTypeLabelsTask.class.getSimpleName();
    private IResourceTypeLabelsActivity resourceTypeActivity;
    private SpringService service = new SpringService();
    private int count = 0;

    public GetResourceTypeLabelsTask(IResourceTypeLabelsActivity resourceTypeActivity){
        this.resourceTypeActivity = resourceTypeActivity;
    }

    public GetResourceTypeLabelsTask(IResourceTypeLabelsActivity resourceTypeActivity, int count) {
        this.count = count;
        this.resourceTypeActivity = resourceTypeActivity;
    }

    @Override
    protected ResourceType[] doInBackground(List<Long>... params) {
        Log.i(TAG, "GetResourceTypeLabelsTask start");
        List<Long> idResourcesTypes = params[0];
        ResourceType[] resourcesType = new ResourceType[idResourcesTypes.size()];

        for(int i = 0; i < idResourcesTypes.size(); i++){
            ResourceType rt = service.getResourceTypeById(idResourcesTypes.get(i));
            resourcesType[i] = rt;
        }
        Log.i(TAG, "GetResourceTypeLabelsTask end");
        return resourcesType;
    }

    @Override
    protected void onPostExecute(ResourceType[] resultPost) {
        if(resultPost != null) {
            resourceTypeActivity.updateResourceTypeLabels(resultPost);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetResourceTypeLabelsTask(resourceTypeActivity, count).execute();
            }
            else {
                resourceTypeActivity.updateResourceTypeLabels(null);
                Toast.makeText(resourceTypeActivity.getContext(), R.string.fail_get_resource_type, Toast.LENGTH_SHORT).show();
            }
        }
    }
}
