class BobRos {
    constructor(urlBob) {
        this.url = urlBob;
        this.ros = null;
        this.hasListeners = false;
        this.subscribedTopics = [];
    };

    connect(retry=false) {
        if (!this.ros) {
            this.ros = new ROSLIB.Ros();
        }
        if (this.ros.isConnected) {
            return;
        }

        if (!this.hasListeners) { // Ensure listeners are only set once
            this.ros.on('connection', () => {
                console.log('Connected to websocket server.');
            });

            this.ros.on('error', (error) => {
                console.log('Error connecting to websocket server:', error);
                if (this.ros) {
                    this.ros.isConnected = false;
                }
            });

            this.ros.on('close', () => {
                console.log('Connection to websocket server closed.');
                if (this.ros) {
                    this.ros.isConnected = false;
                    if (retry) {
                        console.log('Connection lost, attempting to reconnect...');
                        setTimeout(() => this.connect(retry), 1000);
                    }
                }
            });

            this.hasListeners = true;
        }

        this.ros.connect(this.url);
    }

    disconnect() {
        if (this.ros && this.ros.isConnected) {
            this.unsubscribeTopics();
            this.ros.close();
            this.ros = null;
            console.log('Disconnected.');
        } else {
            console.log('No active ROS connection to disconnect.');
        }
    }

    subscribeTopic(topic, messageType, callback) {
        var listener = new ROSLIB.Topic({
            ros: this.ros,
            name: topic,
            messageType: messageType,
        });
  
        listener.subscribe(callback);
        this.subscribedTopics.push(listener);
        console.log('Subscribing to ' + topic);
    }

    unsubscribeTopics() {
        this.subscribedTopics.forEach(listener => {
            listener.unsubscribe();
            console.log('Unsubscribing from ' + listener.name);
        });
        this.subscribedTopics = [];
    }
}