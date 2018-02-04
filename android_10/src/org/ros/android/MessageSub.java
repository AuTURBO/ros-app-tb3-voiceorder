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

package org.ros.android;

import android.annotation.SuppressLint;
import android.content.Context;
import android.util.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */

public class MessageSub<T> implements NodeMain {

    private String topicName;
    private String messageType;
    private MessageCallable<String, T> callable;
    private ConnectedNode mConnectedNode;
    private Subscriber<T> subscriber;

    public MessageSub(Context context) { }

    public void setTopicName(String topicName) {
        this.topicName = topicName;
    }

    public void changeTopicName(String changetopicName) {
        if(!changetopicName.equals(this.topicName)) {
            this.topicName = changetopicName;
            subscriber.shutdown();
            subscriber = mConnectedNode.newSubscriber(topicName, messageType);
            subscriber.addMessageListener(new MessageListener<T>() {
                @Override
                public void onNewMessage(final T message) {

                    if (callable != null) {
                        callable.call(message);
                    }
                }
            });
        }
    }

    public void setMessageType(String messageType) {
        this.messageType = messageType;
    }

    public void setMessageCallable(MessageCallable<String, T> callable) {
        this.callable = callable;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("android/ros_subscrib");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        subscriber = connectedNode.newSubscriber(topicName, messageType);
        subscriber.addMessageListener(new MessageListener<T>() {
            @Override
            public void onNewMessage(final T message) {

                if (callable != null) {
                    callable.call(message);
                }
            }
        });
    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }


}