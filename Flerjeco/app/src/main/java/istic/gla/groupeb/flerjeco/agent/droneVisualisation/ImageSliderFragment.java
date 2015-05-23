package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.content.Context;
import android.support.v4.app.Fragment;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import com.daimajia.slider.library.SliderLayout;
import com.daimajia.slider.library.SliderTypes.BaseSliderView;
import com.daimajia.slider.library.SliderTypes.TextSliderView;
import com.daimajia.slider.library.Tricks.ViewPagerEx;
import com.google.android.gms.maps.model.LatLng;

import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;
import org.springframework.util.support.Base64;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.GetImagesForInterventionAndPositionTask;


public class ImageSliderFragment extends Fragment implements BaseSliderView.OnSliderClickListener, ViewPagerEx.OnPageChangeListener, ImageRefresher{

    private final String TAG = ImageSliderFragment.class.getSimpleName();
    private SliderLayout mDemoSlider;
    private VisualisationActivity activity;
    private View mProgressView;
    private View mDemoSliderIndicator;
    private TextView mImagesEmptyView;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                                Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        View v = inflater.inflate(R.layout.fragment_image_slider, container,
                false);
        mDemoSlider = (SliderLayout) v.findViewById(R.id.image_slider);
        mDemoSliderIndicator = v.findViewById(R.id.image_slider_indicator);
        mProgressView = v.findViewById(R.id.loading_bar);
        mImagesEmptyView = (TextView) v.findViewById(R.id.text_slider_empty);
        return v;
    }

    @Override
    public void onResume() {
        super.onResume();
        activity = (VisualisationActivity) getActivity();
    }

    public void askForImages(LatLng position){
        VisualisationActivity activity = (VisualisationActivity) getActivity();
        Log.i(TAG, "starting thread to get images");
        new GetImagesForInterventionAndPositionTask(this, activity.getIntervention().getId(), position).execute();
    }

    public void setLoading(boolean loading){
        mProgressView.setVisibility(loading ? View.VISIBLE : View.GONE);
        mDemoSlider.setVisibility(loading ? View.GONE : View.VISIBLE);
        mDemoSliderIndicator.setVisibility(loading ? View.GONE : View.VISIBLE);
        mImagesEmptyView.setVisibility(View.GONE);
    }


    /**
     * charge le slider avec les images eet réinitialise à la première image si resetTop est vrai
     * @param images les image
     */
    public void loadImages(List<Image> images){
        Log.i(TAG, "loading images");
        clearCache();
        mDemoSlider.removeAllSliders();
        if(images == null || images.size() == 0){
            mDemoSlider.setVisibility(View.GONE);
            mDemoSliderIndicator.setVisibility(View.GONE);
            mImagesEmptyView.setVisibility(View.VISIBLE);
            return;
        } else{
            mDemoSlider.setVisibility(View.VISIBLE);
            mDemoSliderIndicator.setVisibility(View.VISIBLE);
            mImagesEmptyView.setVisibility(View.GONE);
        }
        //map nom de l'image -> image
        HashMap<String,File> file_maps = new HashMap<>();
        DateTimeFormatter dtf = DateTimeFormat.forPattern("MM/dd/yyyy HH:mm:ss");
        for(Image image : images){
            File imageAsFile = getTempFile(getActivity().getApplicationContext(), image);
            if(imageAsFile != null){
                file_maps.put(dtf.print(image.getTimestamp()), imageAsFile);
            } else {
                Log.e(TAG, "erreur à la création du fichier temporaire pour les images");
            }
        }


        for(Map.Entry<String, File> entry : file_maps.entrySet()){
            TextSliderView textSliderView = new TextSliderView(getActivity());

            // initialize a SliderLayout
            textSliderView
                    .description(entry.getKey())
                    .image(entry.getValue())
                    .setScaleType(BaseSliderView.ScaleType.FitCenterCrop)
                    .setOnSliderClickListener(this);

            //add your extra information
            textSliderView.bundle(new Bundle());
            textSliderView.getBundle()
                    .putString("extra",entry.getKey());

            mDemoSlider.addSlider(textSliderView);
        }
        mDemoSlider.setPresetTransformer(SliderLayout.Transformer.Stack);
        mDemoSlider.setPresetIndicator(SliderLayout.PresetIndicators.Center_Bottom);
        mDemoSlider.stopAutoCycle();
        setLoading(false);
    }

    @Override
    public void onStop() {
        // To prevent a memory leak on rotation, make sure to call stopAutoCycle() on the slider before activity or fragment is destroyed
        mDemoSlider.stopAutoCycle();
        clearCache();
        super.onStop();
    }

    @Override
    public void onSliderClick(BaseSliderView slider) {
        Toast.makeText(this.getActivity(), slider.getBundle().get("extra") + "", Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onPageScrolled(int position, float positionOffset, int positionOffsetPixels) {}

    @Override
    public void onPageSelected(int position) {
        Log.d("Slider Demo", "Page Changed: " + position);
    }

    @Override
    public void onPageScrollStateChanged(int state) {}

    @Override
    public void updateWithImages(List<Image> images) {
        loadImages(images);
    }

    public File getTempFile(Context context, Image image) {
        File file = null;
        try {
            byte[] data = Base64.decode(image.getBase64Image());
            String filename = "sliderimg_" + image.getTimestamp();
            file = File.createTempFile(filename, null, context.getCacheDir());
            FileOutputStream outputStream = new FileOutputStream(file);
            outputStream.write(data);
            outputStream.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        return file;
    }

    private void clearCache(){
        File cacheDir = getActivity().getCacheDir();

        File[] files = cacheDir.listFiles();

        if (files != null) {
            for (File file : files)
                file.delete();
        }
    }

}