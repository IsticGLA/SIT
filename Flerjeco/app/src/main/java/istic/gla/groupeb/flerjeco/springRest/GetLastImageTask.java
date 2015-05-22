package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import java.sql.Timestamp;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by jules on 22/05/15.
 */
public class GetLastImageTask extends AsyncTask<Object, Void, Image> {

    private static final String TAG = GetLastImageTask.class.getSimpleName();

    private SpringService service = new SpringService();
    private IImageActivity activity;

    public GetLastImageTask(IImageActivity activity) {
        this.activity = activity;
    }

    @Override
    protected Image doInBackground(Object... params) {
        Log.i(TAG, "Ask for last image at:" + params[0] + ", " + params[1] + " for intervention: " + params[2]);
        return service.getLastImage((Double)params[0], (Double)params[1], (Long)params[2], (Timestamp)params[3]);
    }

    @Override
    protected void onPostExecute(Image image) {
        if(null != image) {
            Log.i(TAG, "Got Image");
            activity.addImage(image);
        }
    }
}
