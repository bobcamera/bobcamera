import { Injectable, Inject, Optional, InjectionToken } from '@angular/core';
import { catchError, Observable, Subject } from "rxjs";

import * as ROSLIB from 'roslib';

export const BOB_ROS_URL = new InjectionToken<string>('BOB_ROS_URL');

@Injectable({
  providedIn: 'root',
})
export class RosLibServiceBase {

    protected _url: string;
    protected _ros: ROSLIB.Ros;
    protected _hasListeners: boolean;
    protected _topics: ROSLIB.Topic[];
    private _connected = new Subject<boolean>();

    constructor(@Optional() @Inject(BOB_ROS_URL) urlBob?: string) {
      this._url = urlBob ?? "";
      this._ros = null;
      this._hasListeners = false;
      this._topics = [];

      this._connected.next(false);
    }

    get connected(): Observable<boolean>{
        return this._connected.asObservable();
    }

    connect(retry: boolean = false) {
        if (!this._ros) {
            this._ros = new ROSLIB.Ros();
        }
        if (this._ros.isConnected) {
            return;
        }

        if (!this._hasListeners) { // Ensure listeners are only set once
            this._ros.on('connection', (event: any) => {
                console.log('Connected to websocket server.');
                this._connected.next(true);
            });

            this._ros.on('error', (error) => {
                console.log('Error connecting to websocket server:', error);
                if (this._ros) {
                    this._ros.isConnected = false;
                }
                this._connected.next(false);
            });

            this._ros.on('close', (event: any) => {
                console.log('Connection to websocket server closed.');
                if (this._ros) {
                    this._ros.isConnected = false;
                    if (retry) {
                        console.log('Connection lost, attempting to reconnect...');
                        setTimeout(() => this.connect(retry), 1000);
                    }
                }
                this._connected.next(false);
            });

            this._hasListeners = true;
        }

        this._ros.connect(this._url);
    }

    disconnect() {
        if (this._ros && this._ros.isConnected) {
            this.unsubscribeTopics();
            this._ros.close();
            this._ros = null;
            console.log('Disconnected.');
        } else {
            console.log('No active ROS connection to disconnect.');
        }
    }

    protected subscribeTopic<T>(topic: string, messageType: string, callback: (n: T) => any) {
        let listener: ROSLIB.Topic = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: messageType,
        });
  
        listener.subscribe(callback);
        this._topics.push(listener);
        console.log('Subscribing to ' + topic);
    }

    protected unsubscribeTopics() {
        this._topics.forEach((listener: ROSLIB.Topic) => {
            listener.unsubscribe();
            console.log('Unsubscribing from ' + listener.name);
        });
        this._topics = [];
    }
}