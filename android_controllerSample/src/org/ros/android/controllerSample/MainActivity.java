/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.controllerSample;

import android.content.ActivityNotFoundException;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.speech.RecognizerIntent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.address.InetAddressFactory;
import org.ros.android.MessageCallable;
import org.ros.android.MessagePub;
import org.ros.android.MessageSub;
import org.ros.android.RosActivity;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import sensor_msgs.CompressedImage;


public class MainActivity extends RosActivity {

  private JoystickOnlyView joystickOnlyView;
  private ImageView imageView;
  private Context mContext;
  final String forward = "FORWARD" ;
  final String backward = "BACKWARD" ;
  final String right = "RIGHT" ;
  final String left = "LEFT" ;
  final String stop = "STOP" ;
  final String forwardArray[] = {  "앞으로", "앞", "고" ,"go","front","forward" } ;
  final String backwardArray[] = { "뒤로" , "뒤", "빽", "빼" , "back","rear"} ;
  final String leftArray[] = { "왼쪽" , "왼","left"  } ;
  final String rightArray[] = { "오른쪽" , "오른" ,"right" } ;
  final String stopArray[] = { "멈춰" , "정지", "스탑" ,"stop" } ;
  final String korean = "Korean" ;
  final String english = "English" ;
  private String setLanguage = korean ;
  private Spinner s ;
  private Boolean voiceConrol = false ;
  private EditText imageTopic;
  private Button imageTopicSet;
  private TextView txtSpeechInput;
  private ImageButton btnSpeak;
  private final int REQ_CODE_SPEECH_INPUT = 100;
  private Bitmap bmp;
  private Switch sw;
  private MessageSub<nav_msgs.Odometry> subOdometry;
  private MessageSub<sensor_msgs.CompressedImage> subCompressedImage;
  private MessagePub<geometry_msgs.Twist> pubTwist;
  private MessagePub<std_msgs.String> pubVoice;

  public MainActivity() {
    // The RosActivity constructor configures the notification title and ticker
    // messages.
    super("android controller", "android controller");
  }

  @SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);
    mContext = getApplicationContext();

    imageView = (ImageView)findViewById(R.id.robot_image_view);
    joystickOnlyView = (JoystickOnlyView) findViewById(R.id.virtual_joystick);
    txtSpeechInput = (TextView) findViewById(R.id.txtSpeechInput);
    btnSpeak = (ImageButton) findViewById(R.id.btnSpeak);
    imageTopic = (EditText) findViewById(R.id.imageTopic);
    imageTopicSet = (Button) findViewById(R.id.imageTopicSet);
    s = (Spinner)findViewById(R.id.spinner1);
    sw = (Switch)findViewById(R.id.voice_set_switch);

    imageTopicSet.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        if(subCompressedImage!=null) {
          subCompressedImage.changeTopicName(imageTopic.getText().toString());
        }
      }
    });

    sw.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
      @Override
      public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

        voiceConrol = isChecked;
        if(voiceConrol){
          joystickOnlyView.setVisibility(View.INVISIBLE);
          btnSpeak.setVisibility(View.VISIBLE);
          txtSpeechInput.setVisibility(View.VISIBLE);
          s.setVisibility(View.VISIBLE);
        }else{
          joystickOnlyView.setVisibility(View.VISIBLE);
          btnSpeak.setVisibility(View.INVISIBLE);
          txtSpeechInput.setVisibility(View.INVISIBLE);
          s.setVisibility(View.INVISIBLE);
        }
      }
    });

    btnSpeak.setOnClickListener(new View.OnClickListener() {

      @Override
      public void onClick(View v) {
        promptSpeechInput();
      }
    });

    setLanguage = korean;
    s.setSelection(0);
    s.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
      @Override
      public void onItemSelected(AdapterView<?> parent, View view,
                                 int position, long id) {

        if( parent.getItemAtPosition(position).toString().equals(korean)){
          setLanguage = korean;
        }else if( parent.getItemAtPosition(position).toString().equals(english)){
          setLanguage = english;
        }
      }
      @Override
      public void onNothingSelected(AdapterView<?> parent) {}
    });

    joystickOnlyView.onStart();


  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {

    NodeConfiguration nodeConfiguration =
            NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                    getMasterUri());

    ///////////////subscribe Odometry//
    subOdometry =  new MessageSub<nav_msgs.Odometry>(mContext) ;
    subOdometry.setTopicName("odom");
    subOdometry.setMessageType(nav_msgs.Odometry._TYPE);

    subOdometry.setMessageCallable(new MessageCallable< String, nav_msgs.Odometry>() {
      @Override
      public String call(nav_msgs.Odometry message) {

        if( joystickOnlyView != null ) {
          joystickOnlyView.setOdometry(message);
        }
        return null ;
      }
    });
    nodeMainExecutor.execute(subOdometry,
            nodeConfiguration.setNodeName("android/subscribe_odom"));

    ///////////////subscribe CompressedImage//
    subCompressedImage =  new MessageSub<sensor_msgs.CompressedImage>(mContext) ;
    subCompressedImage.setTopicName("/camera1/image_raw/compressed");
    subCompressedImage.setMessageType(sensor_msgs.CompressedImage._TYPE);

    subCompressedImage.setMessageCallable(new MessageCallable< String, sensor_msgs.CompressedImage>() {
      @Override
      public String call(CompressedImage message) {

        ChannelBuffer buffer = message.getData();
        byte[] data = buffer.array();
        InputStream inputStream  = new ByteArrayInputStream(data);
        bmp = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(), buffer.readableBytes());

        new Thread()
        {
          public void run()
          {
            Message messageImage = handler.obtainMessage();
            handler.sendMessage(messageImage);
          }
        }.start();

        return null;
      }
    });
    nodeMainExecutor.execute(subCompressedImage,
            nodeConfiguration.setNodeName("android/subscribe_compressedimage"));

    ////////////////publish Twist //
    pubTwist =  new MessagePub<geometry_msgs.Twist>(mContext) ;
    pubTwist.setTopicName("/cmd_vel");
    pubTwist.setMessageType(geometry_msgs.Twist._TYPE);
    nodeMainExecutor.execute(pubTwist,
            nodeConfiguration.setNodeName("android/publish_twist"));
    //nodeMainExecutor.shutdown();

    //pubTwist.onShutdown(nodeConfiguration.getNodeName());
    //pubTwist.setTopicName("/cmd_vel2");
    /*nodeMainExecutor.execute(pubTwist,
            nodeConfiguration.setNodeName("android/publish_twist"));*/

    ////////////////publish String //
    pubVoice =  new MessagePub<std_msgs.String>(mContext) ;
    pubVoice.setTopicName("~recognizer/output");
    pubVoice.setMessageType(std_msgs.String._TYPE);
    nodeMainExecutor.execute(pubVoice,
            nodeConfiguration.setNodeName("android/publish_voice_string"));

  }

  final Handler handler = new Handler()
  {
    public void handleMessage(Message msg)
    {
      imageView.setImageBitmap(bmp);
    }
  };


  public void setTwist( double twistLinear[] , double twistArgular[] ){
    if( !voiceConrol ) {
      pubTwist.message.getLinear().setX(twistLinear[0]/12);
      pubTwist.message.getLinear().setY(twistLinear[1]/12);
      pubTwist.message.getLinear().setZ(twistLinear[2]/12);
      pubTwist.message.getAngular().setX(twistArgular[0]/3);
      pubTwist.message.getAngular().setY(twistArgular[1]/3);
      pubTwist.message.getAngular().setZ(twistArgular[2]/3);
      pubTwist.publish();
    }
  }

  public void setTwistbyVoice( double twistLinear[] , double twistArgular[] ){
    if( voiceConrol ) {
      pubTwist.message.getLinear().setX(twistLinear[0]);
      pubTwist.message.getLinear().setY(twistLinear[1]);
      pubTwist.message.getLinear().setZ(twistLinear[2]);
      pubTwist.message.getAngular().setX(twistArgular[0]);
      pubTwist.message.getAngular().setY(twistArgular[1]);
      pubTwist.message.getAngular().setZ(twistArgular[2]);
      pubTwist.publish();
    }
  }

  /**
   * Showing google speech input dialog
   * */
  private void promptSpeechInput() {

    double twistLinear[] = { 0, 0, 0 } ;
    double twistArgular[] = { 0, 0, 0 } ;
    Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
    intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
            RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
    //intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault());
    if( setLanguage.equals(korean)) {
      intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "ko-KR");
    }else if( setLanguage.equals(english)) {
      intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "en-US");
    }

    intent.putExtra(RecognizerIntent.EXTRA_PROMPT,
            getString(R.string.speech_prompt));
    try {
      startActivityForResult(intent, REQ_CODE_SPEECH_INPUT);
    } catch (ActivityNotFoundException a) {
      Toast.makeText(getApplicationContext(),
              getString(R.string.speech_not_supported),
              Toast.LENGTH_SHORT).show();
    }
    setTwistbyVoice(twistLinear,twistArgular);

  }

  /**
   * Receiving speech input
   * */
  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {

    switch (requestCode) {
      case REQ_CODE_SPEECH_INPUT: {
        if (resultCode == RESULT_OK && null != data) {

          ArrayList<String> result = data
                  .getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
          String sentance = result.get(0) ;
          String controlDirection = judgeDirection ( sentance );
          txtSpeechInput.setText( controlDirection + "_" + sentance);
          pubVoice.message.setData(sentance);
          pubVoice.publish();
          if( voiceConrol ){
            voiceContorlDiretion( controlDirection );
          }
        }
        break;
      }
      default:
        super.onActivityResult(requestCode, resultCode, data);
        break;
    }
  }

  public void voiceContorlDiretion( String controlDirection ){

    double linearVelocityX  = 0, linearVelocityY = 0 , angularVelocityZ = 0;
    double twistLinear[] = { 0, 0, 0 } ;
    double twistArgular[] = { 0, 0, 0 } ;

    if(controlDirection.equals(forward)){
      linearVelocityX = 0.05 ;
      angularVelocityZ = 0 ;
    }else if(controlDirection.equals(backward)){
      linearVelocityX = -0.05 ;
      angularVelocityZ = 0 ;
    }else if(controlDirection.equals(left)){
      linearVelocityX = 0 ;
      angularVelocityZ = 0.3 ;
    }else if(controlDirection.equals(right)){
      linearVelocityX = 0 ;
      angularVelocityZ = -0.3 ;
    }else if(controlDirection.equals(stop)){
      linearVelocityX = 0 ;
      angularVelocityZ = 0 ;
    }

    twistLinear[0] = linearVelocityX;
    twistLinear[1] = linearVelocityY;
    twistArgular[2] = angularVelocityZ;

     setTwistbyVoice(twistLinear, twistArgular);

  }

  public String judgeDirection( String sentance ){

    String currentControlState = stop;
    int i = 0 ;
    for( i = 0 ; i<forwardArray.length ; i++ ){
      if( sentance.contains(forwardArray[i]) ){
        currentControlState = forward ;
        break;
      }
    }
    for( i = 0 ; i<backwardArray.length ; i++ ){
      if( sentance.contains(backwardArray[i]) ){
        currentControlState = backward ;
        break;
      }
    }
    for( i = 0 ; i<leftArray.length ; i++ ){
      if( sentance.contains(leftArray[i]) ){
        currentControlState = left ;
        break;
      }
    }
    for( i = 0 ; i<rightArray.length ; i++ ){
      if( sentance.contains(rightArray[i]) ){
        currentControlState = right ;
        break;
      }
    }
    for( i = 0 ; i<stopArray.length ; i++ ){
      if( sentance.contains(stopArray[i]) ){
        currentControlState = stop ;
        break;
      }
    }
    return currentControlState ;
  }

}
