package istic.gla.groupeb.flerjeco.springRest;

import android.util.Log;

import org.apache.http.HttpStatus;
import org.apache.http.util.ExceptionUtils;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.HttpStatusCodeException;
import org.springframework.web.client.RestTemplate;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {
    private static final String TAG = SpringService.class.getSimpleName();
    private static final String URL = "http://ns3002211.ip-37-59-58.eu:8080/nivimoju/rest/";

    public void codeSinistreClient() {

        final String url = URL + "utl/to/code/sinistre/";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);
        IncidentCode[] coordinates = incidentCode.getBody();

    }

    public String login(String id, String password) {
        Log.i(TAG, "login start");
        final String url = URL + "authentication/connected/"+id+"/"+password;
        String httpResult = "";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        try {
            ResponseEntity<String> entity = restTemplate.getForEntity(url, String.class);
            httpResult = entity.getStatusCode().toString();
        } catch (HttpStatusCodeException e) {
            httpResult = "400";
        }
        Log.i(TAG, "login end");
        return httpResult;
    }

}
