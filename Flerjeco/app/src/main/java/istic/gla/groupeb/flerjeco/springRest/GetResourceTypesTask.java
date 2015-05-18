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
public class GetResourceTypesTask extends AsyncTask<List<Long>, Void, ResourceType[]> {
    private static final String TAG = GetResourceTypesTask.class.getSimpleName();
    private IResourceTypesActivity resourceTypeActivity;
    private SpringService service = new SpringService();
    private int count = 0;

    public GetResourceTypesTask(IResourceTypesActivity resourceTypeActivity){
        this.resourceTypeActivity = resourceTypeActivity;
    }

    public GetResourceTypesTask(IResourceTypesActivity resourceTypeActivity, int count) {
        this.count = count;
        this.resourceTypeActivity = resourceTypeActivity;
    }

    @Override
    protected ResourceType[] doInBackground(List<Long>... params) {
        Log.i(TAG, "GetResourceTypeTask start");
        List<Long> idResourcesTypes = params[0];
        ResourceType[] resourcesType = new ResourceType[idResourcesTypes.size()];

        for(int i = 0; i < idResourcesTypes.size(); i++){
            ResourceType rt = service.getResourceTypeById(idResourcesTypes.get(i));
            resourcesType[i] = rt;
        }
        Log.i(TAG, "GetResourceTypeTask end");
        return resourcesType;
    }

    @Override
    protected void onPostExecute(ResourceType[] resultPost) {
        if(resultPost != null) {
            resourceTypeActivity.updateResourceTypes(resultPost);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetResourceTypesTask(resourceTypeActivity, count).execute();
            }
            else {
                resourceTypeActivity.updateResourceTypes(null);
                Toast.makeText(resourceTypeActivity.getContext(), R.string.fail_get_resource_type, Toast.LENGTH_SHORT).show();
            }
        }
    }
}
