package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

import entity.ResourceType;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by corentin on 16/04/15.
 * Background task to get Resource Type
 */
public class GetResourceTypeTask extends AsyncTask<List<Long>, Void, List<ResourceType>> {
    private static final String TAG = GetResourceTypeTask.class.getSimpleName();
    private IResourceTypeActivity resourceTypeActivity;
    private int count = 0;

    public GetResourceTypeTask(IResourceTypeActivity resourceTypeActivity){
        this.resourceTypeActivity = resourceTypeActivity;
    }

    public GetResourceTypeTask(IResourceTypeActivity resourceTypeActivity, int count) {
        this.count = count;
        this.resourceTypeActivity = resourceTypeActivity;
    }

    @Override
    protected List<ResourceType> doInBackground(List<Long>... params) {
        Log.i(TAG, "GetResourceTypeTask start");
        List<ResourceType> resourcesType = new ArrayList<>();
        List<Long> idResourcesTypes = params[0];
        SpringService service = new SpringService();

        for(Long idRes : idResourcesTypes){
            ResourceType rt = service.getResourceTypeById(idRes);
            resourcesType.add(rt);
        }
        Log.i(TAG, "GetResourceTypeTask end");
        return  resourcesType;
    }

    @Override
    protected void onPostExecute(List<ResourceType> resultPost) {
        if(resultPost != null) {
            resourceTypeActivity.getResourceType(resultPost);
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetResourceTypeTask(resourceTypeActivity, count).execute();
            }
            else {
                resourceTypeActivity.getResourceType(null);
                Toast.makeText(resourceTypeActivity.getContext(), R.string.fail_get_resource_type, Toast.LENGTH_SHORT).show();
            }
        }
    }
}
