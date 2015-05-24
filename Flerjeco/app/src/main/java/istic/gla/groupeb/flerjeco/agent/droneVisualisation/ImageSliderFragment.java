package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.content.Context;
import android.support.v4.app.Fragment;
import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
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

import org.apache.commons.collections4.CollectionUtils;
import org.apache.commons.collections4.ListUtils;
import org.joda.time.DateTime;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.GetImagesForInterventionAndPositionTask;


public class ImageSliderFragment extends Fragment implements BaseSliderView.OnSliderClickListener, ViewPagerEx.OnPageChangeListener, ImageRefresher{

    private final String TAG = ImageSliderFragment.class.getSimpleName();
    private SliderLayout mDemoSlider;
    private View mProgressView;
    private View mDemoSliderIndicator;
    private TextView mImagesEmptyView;
    private Long mInterventionId;
    private LatLng mPosition;

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
        Bundle bundle = getArguments();
        mInterventionId = bundle.getLong("interventionId");
        mPosition = new LatLng(bundle.getDouble("latitude"), bundle.getDouble("longitude"));
        return v;
    }

    @Override
    public void onResume() {
        super.onResume();
        askForImages(mPosition);
    }

    public void askForImages(LatLng position){
        Log.i(TAG, "starting thread to get images");
        setLoading(true);
        new GetImagesForInterventionAndPositionTask(this, mInterventionId, position).execute();
    }

    public void setLoading(boolean loading){
        mProgressView.setVisibility(loading ? View.VISIBLE : View.GONE);
        mDemoSlider.setVisibility(loading ? View.GONE : View.VISIBLE);
        mDemoSliderIndicator.setVisibility(loading ? View.GONE : View.VISIBLE);
        mImagesEmptyView.setVisibility(View.GONE);
    }

    @Override
    public void onStop() {
        // To prevent a memory leak on rotation, make sure to call stopAutoCycle() on the slider before activity or fragment is destroyed
        mDemoSlider.stopAutoCycle();
        clearCache();
        super.onStop();
    }

    @Override
    public void onSliderClick(BaseSliderView slider) {}

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
        Log.i(TAG, "updating with images");
        clearCache();

        Log.i(TAG, (images == null ? 0 : images.size()) + " images to load");

        if(images == null || images.size() == 0){
            mProgressView.setVisibility(View.GONE);
            mDemoSlider.setVisibility(View.GONE);
            mDemoSliderIndicator.setVisibility(View.GONE);
            mImagesEmptyView.setVisibility(View.VISIBLE);
            return;
        } else{
            mDemoSlider.setVisibility(View.VISIBLE);
            mDemoSliderIndicator.setVisibility(View.VISIBLE);
            mImagesEmptyView.setVisibility(View.GONE);
        }

        Collections.sort(images, new Comparator<Image>() {
            @Override
            public int compare(Image image, Image t1) {
                return ((Long) image.getTimestamp()).compareTo(t1.getTimestamp());
            }
        });
        List<Pair<String, File>> fileList = new ArrayList<>();
        DateTimeFormatter dtf = DateTimeFormat.forPattern("MM/dd/yyyy HH:mm:ss");
        for(Image image : images){
            Log.d(TAG, "creating temporary image file");
            File imageAsFile = getTempFile(getActivity().getApplicationContext(), image);
            if(imageAsFile != null){
                fileList.add(new Pair<>(dtf.print(image.getTimestamp()), imageAsFile));
            } else {
                Log.e(TAG, "erreur à la création du fichier temporaire pour les images");
            }
        }
        mDemoSlider.removeAllSliders();

        for(Pair<String, File> pair : fileList){
            Log.d(TAG, "adding a slider for " + pair.first);
            TextSliderView textSliderView = new TextSliderView(getActivity());
            // initialize a SliderLayout
            textSliderView
                    .description(pair.first)
                    .image(pair.second)
                    .setScaleType(BaseSliderView.ScaleType.FitCenterCrop);
            mDemoSlider.addSlider(textSliderView);
        }
        mDemoSlider.setPresetTransformer(SliderLayout.Transformer.Stack);
        mDemoSlider.setPresetIndicator(SliderLayout.PresetIndicators.Center_Bottom);
        mDemoSlider.stopAutoCycle();
        mDemoSlider.setCurrentPosition(mDemoSlider.getChildCount());
        setLoading(false);
    }

    @Override
    public Context getContext() {
        return getActivity().getApplicationContext();
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
            for (File file : files) {
                if(StringUtils.startsWithIgnoreCase(file.getName(), "sliderimg")){
                    Log.i(TAG, "deleting " + file.getName());
                    file.delete();
                }
            }
        }
    }

}