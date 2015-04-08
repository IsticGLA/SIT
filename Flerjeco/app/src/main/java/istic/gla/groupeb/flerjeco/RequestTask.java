package istic.gla.groupeb.flerjeco;

import android.os.AsyncTask;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;

import org.apache.http.Header;
import org.apache.http.HeaderElement;
import org.apache.http.HttpRequest;
import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.ParseException;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.client.methods.HttpUriRequest;
import org.apache.http.entity.ByteArrayEntity;
import org.apache.http.impl.client.DefaultHttpClient;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;

/**
 * Created by jules on 09/03/15.
 */
class RequestTask extends AsyncTask<String, String, String> {

    private final HttpUriRequest request;
    private final Messenger replyTo;
    private final int message;

    public RequestTask(HttpUriRequest request, int message, Messenger replyTo) {
        this.message = message;
        this.request = request;
        this.replyTo = replyTo;
    }

    @Override
    protected String doInBackground(String... uri) {
        String response;
        try {
            response = request();
        } catch (IOException e) {
            e.printStackTrace();
            response = e.getMessage();
        }
        return response;
    }

    private String request() throws IOException {
        request.setHeader(new Header() {
            @Override
            public String getName() {
                return "content-type";
            }

            @Override
            public String getValue() {
                return "application/json";
            }

            @Override
            public HeaderElement[] getElements() throws ParseException {
                return new HeaderElement[0];
            }
        });
        HttpClient httpclient = new DefaultHttpClient();
        HttpResponse response = httpclient.execute(request);
        StatusLine statusLine = response.getStatusLine();
        if(statusLine.getStatusCode() == HttpStatus.SC_OK){
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            response.getEntity().writeTo(out);
            String responseString = out.toString();
            out.close();
            return responseString;
        } else{
            //Closes the connection.
            response.getEntity().getContent().close();
            throw new IOException(statusLine.getStatusCode()+"\n"+statusLine.getReasonPhrase());
        }
    }

    @Override
    protected void onPostExecute(String result) {
        super.onPostExecute(result);

        Message _msg = Message.obtain(null, message, result);
        try {
            replyTo.send(_msg);
        } catch (RemoteException e) {
            e.printStackTrace();
        }
    }
}