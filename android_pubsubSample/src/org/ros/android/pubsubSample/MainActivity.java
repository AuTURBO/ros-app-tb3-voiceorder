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

package org.ros.android.pubsubSample;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import org.ros.android.MessageCallable;
import org.ros.android.MessagePub;
import org.ros.android.RosActivity;
import org.ros.android.MessageSub;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MainActivity extends RosActivity {

  private MessagePub<std_msgs.String> pubString;
  private MessageSub<std_msgs.String> subString;
  private String subStringMessage;
  private Button publishButton;
  private EditText textPublish;
  private TextView textSubscrib;
  private Context mContext;

  public MainActivity() {
    // The RosActivity constructor configures the notification title and ticker
    // messages.
    super("Pubsub Sample", "Pubsub Sample");
  }

  @SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);
    mContext = getApplicationContext();
    publishButton = (Button) findViewById(R.id.publishButton);
    textPublish = (EditText) findViewById(R.id.stringPublish);
    textSubscrib = (TextView) findViewById(R.id.stringSubscrib);

    publishButton.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View view) {
        String publishtext = textPublish.getText().toString();
        pubString.message.setData(publishtext);
        pubString.publish();
      }
    });

  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {

    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
    nodeConfiguration.setMasterUri(getMasterUri());

    ////////////////publish string_test //
    pubString = new MessagePub<std_msgs.String>(mContext);
    pubString.setTopicName("string_test");
    pubString.setMessageType(std_msgs.String._TYPE);
    nodeMainExecutor.execute(pubString,
            nodeConfiguration.setNodeName("android/string_test_pub"));

    ///////////////subscribe string_test//
    subString =  new MessageSub<std_msgs.String>(mContext) ;
    subString.setTopicName("string_test");
    subString.setMessageType(std_msgs.String._TYPE);
    subString.setMessageCallable(new MessageCallable< String, std_msgs.String>() {
      @Override
      public String call(std_msgs.String message) {
        subStringMessage = message.getData();
        new Thread()
        {
          public void run()
          {
            Message messageImage = handler.obtainMessage();
            handler.sendMessage(messageImage);
          }
        }.start();
        return null ;
      }
    });
    nodeMainExecutor.execute(subString,
            nodeConfiguration.setNodeName("android/string_test_sub"));

  }

  final Handler handler = new Handler()
  {
    public void handleMessage(Message msg)
    {
      textSubscrib.setText("Subscribe : " + subStringMessage);
    }
  };

}