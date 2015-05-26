package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;

import java.sql.Timestamp;
import java.util.List;

import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.entity.Image;

/**
 * Created by jules on 22/05/15.
 */
public class GetLastImageTask extends AsyncTask<Object, Void, Image[]> {

    private static final String TAG = GetLastImageTask.class.getSimpleName();

    private SpringService service = new SpringService();
    private IImageActivity activity;

    public GetLastImageTask(IImageActivity activity) {
        this.activity = activity;
    }

    @Override
    protected Image[] doInBackground(Object... params) {
        Log.i(TAG, "Ask for last images for intervention: " + params[0]);
        return service.getLastImages((Long) params[0], (List<TimestampedPosition>) params[1]);
    }

    @Override
    protected void onPostExecute(Image[] images) {
        if(null != images) {
            Log.i(TAG, "Got Images");
            activity.updateImages(images);
        }
    }
}
