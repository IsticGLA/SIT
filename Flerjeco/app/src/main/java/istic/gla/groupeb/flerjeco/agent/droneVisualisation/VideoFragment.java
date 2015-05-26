package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;
import org.springframework.util.support.Base64;

import java.util.Timer;
import java.util.TimerTask;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.GetLastImageForDroneTask;


public class VideoFragment extends Fragment implements VideoRefresher{

    private final String TAG = VideoFragment.class.getSimpleName();
    private ImageView mDroneVideoImageView;
    private TextView mDroneVideoTitleView;
    private String mDroneLabel;
    private DateTimeFormatter dtf = DateTimeFormat.forPattern("MM/dd/yyyy HH:mm:ss");
    Timer timer;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                                Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        View v = inflater.inflate(R.layout.fragment_drone_video, container,
                false);
        mDroneVideoTitleView = (TextView) v.findViewById(R.id.drone_video_title);
        mDroneVideoImageView = (ImageView) v.findViewById(R.id.drone_video_image);
        Bundle bundle = getArguments();
        mDroneLabel = bundle.getString("droneLabel");
        mDroneVideoTitleView.setText(
                String.format(getString(R.string.drone_video_title), mDroneLabel));
        return v;
    }

    @Override
    public void onResume() {
        super.onResume();
        Log.i(TAG, "starting thread to get images");
        final Handler handler = new Handler();
        timer = new Timer();
        final VideoFragment refresher = this;
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                handler.post(new Runnable() {
                    public void run() {
                        new GetLastImageForDroneTask(refresher, mDroneLabel).execute();
                    }
                });
            }
        };
        timer.schedule(task, 0, 1000); //it executes this every 1000ms

    }

    @Override
    public void onPause() {
        super.onPause();
        timer.cancel();
    }

    /**
     * change l'image à afficher
     * @param image l'image
     */
    @Override
    public void setLastImage(Image image) {
        if(image != null){
            try {
                byte[] imageAsBytes = Base64.decode(image.getBase64Image());
                mDroneVideoImageView.setImageBitmap(BitmapFactory.decodeByteArray(imageAsBytes, 0, imageAsBytes.length));
            } catch (Exception e){
                Log.e(TAG, "erreur a l'affichage d'image", e);
            }
        }
    }
}