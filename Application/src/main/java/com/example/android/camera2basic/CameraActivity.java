/*
 * Copyright 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.camera2basic;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;
import android.widget.TextView;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.nio.ByteBuffer;

public class CameraActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);
        if (null == savedInstanceState) {
            getFragmentManager().beginTransaction()
                    .replace(R.id.container, Camera2BasicFragment.newInstance())
                    .commit();
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        int w=1920, h=1080;
        String rgbFile =  "/sdcard/DCIM/Camera/imx586_1.jpeg";
        String tofFile =  "/sdcard/DCIM/Camera/imx454_1.jpeg";

        File imgFile = new  File(rgbFile);

        if(imgFile.exists()){

            Bitmap myBitmap = BitmapFactory.decodeFile(imgFile.getAbsolutePath());

            ByteArrayOutputStream stream = new ByteArrayOutputStream();
            if(myBitmap != null)
              myBitmap.compress(Bitmap.CompressFormat.JPEG, 70, stream);

            byte[] array = stream.toByteArray();

            ByteBuffer img454 = ByteBuffer.allocateDirect(w*h*4);
            img454.wrap(array);
            Log.d("Argus",stringFromJNI(img454, rgbFile, tofFile, w, h));
        }

        //ByteBuffer img454 = ByteBuffer.allocateDirect(w*h*4);


    }

    static {
        System.loadLibrary("hello-jnicallback");
    }
    public native  String stringFromJNI(ByteBuffer img, String hImg, String hassFile, int w, int h);
}
