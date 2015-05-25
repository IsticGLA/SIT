package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.content.Context;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.util.Pair;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import com.daimajia.slider.library.SliderLayout;
import com.daimajia.slider.library.SliderTypes.BaseSliderView;
import com.daimajia.slider.library.SliderTypes.TextSliderView;
import com.daimajia.slider.library.Tricks.ViewPagerEx;
import com.google.android.gms.maps.model.LatLng;

import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;
import org.springframework.util.StringUtils;
import org.springframework.util.support.Base64;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.GetImagesForInterventionAndPositionTask;
import istic.gla.groupeb.flerjeco.springRest.GetLastImagesForDrone;


public class VideoFragment extends Fragment implements VideoRefresher{

    private final String TAG = VideoFragment.class.getSimpleName();
    private ImageView mDroneVideoImageView;
    private String mDroneLabel;
    private DateTimeFormatter dtf = DateTimeFormat.forPattern("MM/dd/yyyy HH:mm:ss");
    Timer timer;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                                Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        View v = inflater.inflate(R.layout.fragment_drone_video, container,
                false);
        mDroneVideoImageView = (ImageView) v.findViewById(R.id.drone_video_image);
        Bundle bundle = getArguments();
        mDroneLabel = bundle.getString("droneLabel");
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
                        new GetLastImagesForDrone(refresher, mDroneLabel).execute();
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