package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import istic.gla.groupb.nivimoju.entity.StaticData;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by jules on 16/04/15.
 */
public class GetAllStaticDataTask extends AsyncTask<Void, Void, StaticData[]> {

    private int count = 0;
    private static final String TAG = GetAllStaticDataTask.class.getName();
    private SpringService service = new SpringService();
    private IStaticDataActivity activity;

    public GetAllStaticDataTask(IStaticDataActivity activity) {
        this.activity = activity;
    }

    public GetAllStaticDataTask(IStaticDataActivity activity, int count) {
        this.activity = activity;
        this.count = count;
    }

    @Override
    protected StaticData[] doInBackground(Void... params) {
        return service.getAllStaticDatas();
    }

    protected void onPostExecute(StaticData[] data) {
        if(null != data) {
            activity.setStaticData(data);
            Log.i(TAG, "Static data got");
        } else {
            count++;
            if(count < 4) {
                Log.i(TAG, "Count: " + count);
                new GetAllStaticDataTask(activity, count).execute();
            }
            else {
                Toast.makeText(activity.getContext(), R.string.fail_get_static_data, Toast.LENGTH_SHORT).show();
                Log.i(TAG, "Fail to get static data");
            }
        }
    }
}
