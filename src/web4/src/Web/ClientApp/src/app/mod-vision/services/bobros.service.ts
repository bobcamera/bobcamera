import { Injectable, Inject } from "@angular/core";
import { catchError, Observable, Subject } from "rxjs";
import { map, switchMap } from 'rxjs/operators';

import * as ROSLIB from 'roslib';

import { AppStateDto } from '../models';

export enum SubscriptionType {
    Annotated,
    ForegroundMask,
    DetectionMask,
    PrivacyMask,
}

@Injectable()
export class BobRosService {    

    // https://stackoverflow.com/questions/64647388/roslib-with-angular-10
    protected _port: number = 9090;
    protected _url: string;
    protected _ros: ROSLIB.Ros;
    protected _hasListeners: boolean;
    protected _topics: ROSLIB.Topic[];
    private _connected = new Subject<boolean>();

    constructor(@Inject('BASE_URL') baseUrl: string) {
      //this._url = baseUrl ?? "http://localhost";
      this._url = "http://localhost";
      this._ros = null;
      this._hasListeners = false;
      this._topics = [];

      this._connected.next(false);
    }

    get isConnected(): boolean{
        if (this._ros) {
            return this._ros.isConnected;
        }
        return false;
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

        console.log(`Ros Bridge URL: ${this._url}:${this._port}`);

        this._ros.connect(`${this._url}:${this._port}`);
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

    private _types = {
        [SubscriptionType.Annotated]: "/bob/frames/annotated/resized/compressed",
        [SubscriptionType.ForegroundMask]: "/bob/frames/foreground_mask/resized/compressed",
        [SubscriptionType.DetectionMask]: "/bob/frames/allsky/original/resized/compressed",
        [SubscriptionType.PrivacyMask]: "/bob/frames/allsky/original/resized/compressed"
    };

    private _videoStream = new Subject<any>();
    private _appState = new Subject<any>();

    private _videoStreamListener: ROSLIB.Topic;
    private _appStateListener: ROSLIB.Topic;

    public videoStream(type: SubscriptionType): Observable<string>{

        this.internal_videoStream(type);

        return this._videoStream.pipe(
            map((message: any) => this.transformImage(message))
          );        
    }

    public appState(subscribe: boolean): Observable<AppStateDto> {

        if (subscribe) {
            if (!this._appStateListener) {
                this.internal_appState();
            }
        } else {
            this.unsubscribeTopic(this._appStateListener);
            this._appStateListener = null;
        }

        return this._appState.pipe(
            map((message: any) => this.transformStatus(message))
          );
    }

    private internal_videoStream(type: SubscriptionType) {

        if (this._videoStreamListener) {
            this.unsubscribeTopic(this._videoStreamListener);
        }

        const url = this._types[type];

        this._videoStreamListener = this.subscribeTopic<string>(
            url, 
            'sensor_msgs/msg/CompressedImage', 
            (msg: any) => { this._videoStream.next(msg);}
        );
    }

    private internal_appState() {
        this._appStateListener = this.subscribeTopic<string>(
            '/bob/monitoring/status', 
            'bob_interfaces/msg/MonitoringStatus', 
            (msg: any) => { this._appState.next(msg);}
        );
    }

    private transformImage(message: any): string {
        return `data:image/jpeg;base64,${message.data}`;
    }

    private transformStatus(message: any): AppStateDto {
        return {
            trackable: message.trackable,
            alive: message.alive,
            started: message.started,
            ended: message.ended,
            sensitivity: message.sensitivity,
            max_blobs_reached: message.max_blobs_reached,
            recording: message.recording,
            percentage_cloud_cover: message.percentage_cloud_cover,
            unimodal_cloud_cover: message.unimodal_cloud_cover,
            day_night_enum: message.day_night_enum,
            avg_brightness: message.avg_brightness
        };
    }

    private subscribeTopic<T>(topic: string, messageType: string, callback: (n: T) => any): ROSLIB.Topic {
        let listener: ROSLIB.Topic = new ROSLIB.Topic({
            ros: this._ros,
            name: topic,
            messageType: messageType,
        });
  
        listener.subscribe(callback);
        this._topics.push(listener);
        console.log('Subscribing to ' + topic);
        return listener;
    }

    private unsubscribeTopic(listener: ROSLIB.Topic) {
        const index = this._topics.indexOf(listener, 0);
        if (index > -1) {
            this._topics.splice(index, 1);
        }
        listener.unsubscribe();
        console.log('Unsubscribing from ' + listener.name);        
    }

    private unsubscribeTopics() {
        this._topics.forEach((listener: ROSLIB.Topic) => {
            listener.unsubscribe();
            console.log('Unsubscribing from ' + listener.name);
        });
        this._topics = [];
    }     
}