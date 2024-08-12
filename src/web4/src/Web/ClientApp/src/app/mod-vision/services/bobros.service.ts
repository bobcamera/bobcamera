import { Injectable, Inject } from "@angular/core";
import { Store } from '@ngrx/store';
import { catchError, Observable, Subject, distinctUntilChanged } from "rxjs";
import { map, switchMap } from 'rxjs/operators';

import * as ROSLIB from 'roslib';

import * as MainActions from '../../mod-main/state/main.actions';
import { NotificationType } from '../../mod-main/models';

import { VisionActions } from '../state';
import { VisionState } from '../state';

import { AppInfoDto, AppStateDto } from '../models';

export enum ImageStreamType {
    Annotated,
    ForegroundMask,
    DetectionMask,
    PrivacyMask,
}

export class BobRosConnection {

    // https://stackoverflow.com/questions/64647388/roslib-with-angular-10
    protected _port: number;
    protected _url: string;
    protected _ros: ROSLIB.Ros;
    private _connected = new Subject<boolean>();    

    constructor(url: string = "http://localhost", port: number = 9090) {
        this._url = url;
        this._port = port;
        this._connected.next(false);
    }

    get Ros(): ROSLIB.Ros {
        return this._ros;
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

    open(retry: boolean = false) {
        if (!this._ros) {
            this._ros = new ROSLIB.Ros();
        }
        if (this._ros.isConnected) {
            return;
        }

        this._ros.on('connection', (event: any) => {
            console.log('BobRosConnection: Connected to websocket server.');
            this._connected.next(true);
        });

        this._ros.on('error', (error) => {
            console.log('BobRosConnection: Error connecting to websocket server:', error);
            if (this._ros) {
                this._ros.isConnected = false;
            }
            this._connected.next(false);
        });

        this._ros.on('close', (event: any) => {
            //console.log('BobRosConnection: Connection to websocket server closed.');
            if (this._ros) {
                this._ros.isConnected = false;
                if (retry) {
                    console.log('BobRosConnection: Connection lost, attempting to reconnect...');
                    setTimeout(() => this.open(retry), 1000);
                }
            }
            this._connected.next(false);
        });    

        //console.log(`BobRosConnection: URL: ${this._url}:${this._port}`);

        this._ros.connect(`${this._url}:${this._port}`);
    }

    close() {
        if (this._ros && this._ros.isConnected) {
            this._ros.close();
            this._ros = null;
            console.log('BobRosConnection: Disconnected.');
        } else {
            console.log('BobRosConnection: No active ROS connection to disconnect.');
        }
    }

    execute(callback: () => void) {
        this.connected
        .pipe(distinctUntilChanged())
        .subscribe((connected: boolean) => {
            console.log(`BobRosConnection: Connection status to ROS Bridge: ${connected}`);
            if (connected) {       
                callback();
            }
        })
    }
}

@Injectable()
export class BobRosService {    

    // https://stackoverflow.com/questions/64647388/roslib-with-angular-10
    protected _connection: BobRosConnection;
    protected _hasListeners: boolean;
    protected _topics: ROSLIB.Topic[];
    protected _store: Store<VisionState>;

    constructor(@Inject('BASE_URL') baseUrl: string, store: Store<VisionState>) {
      this._connection = new BobRosConnection();
      this._hasListeners = false;
      this._topics = [];
      this._store = store;
    }

    get isConnected(): boolean{
      if (this._connection) {
        return this._connection.isConnected;
      }
      return false;
    }

    get connected(): Observable<boolean>{
      return this._connection.connected;
    }

    connect(retry: boolean = false) {
        this._connection.open(retry);
    }

    disconnect() {        
        if (this._connection && this._connection.isConnected) {
            this.unsubscribeTopics();
            this._connection.close();
        }
    }

    private _imageStreamTypes = {
        [ImageStreamType.Annotated]: "/bob/frames/annotated/resized/compressed",
        [ImageStreamType.ForegroundMask]: "/bob/frames/foreground_mask/resized/compressed",
        [ImageStreamType.DetectionMask]: "/bob/frames/allsky/original/resized/compressed",
        [ImageStreamType.PrivacyMask]: "/bob/frames/allsky/original/resized/compressed"
    };

    private _videoStream = new Subject<any>();
    private _appState = new Subject<any>();

    private _videoStreamListener: ROSLIB.Topic;
    private _appStateListener: ROSLIB.Topic;

    public subVideoStream(type: ImageStreamType): Observable<string>{

        this.internal_videoStream(type);

        return this._videoStream.pipe(
            map((message: any) => this.transformImage(message))
          );        
    }

    public subAppState(subscribe: boolean): Observable<AppStateDto> {

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

    public svcAppInfo(): void {

        let connection = new BobRosConnection();
        connection.open(false); 
        connection.execute(() => {
            
            let service = new ROSLIB.Service({
                ros : connection.Ros,
                name : '/bob/webapi/application/info',
                serviceType : 'bob_interfaces/srv/ApplicationInfo'
            });              

            let request = new ROSLIB.ServiceRequest({empty: ''});

            console.log(`BobRosService.svcAppInfo: Calling service.callService`);
    
            service.callService(request, function(result) {
    
                console.log(`Bob-ROS2 Version: ${result.version}, Frame Size: w=${result.frame_width} x h=${result.frame_height}, FPS: ${result.video_fps}`);
    
                let appInfo = <AppInfoDto>{ version: result.version,
                    frame_width: result.frame_width,
                    frame_height: result.frame_height,
                    video_fps: result.video_fps };
    
                this._store.dispatch(VisionActions.setBobInfo({ info: appInfo }));
    
                connection.close();
    
            }.bind(this));
        });     
    }

    public svcPrivacyMaskOverride(mask_enabled: boolean = true) {

        let connection = new BobRosConnection();
        connection.open(false); 
        connection.execute(() => {     

            let service = new ROSLIB.Service({
                ros : connection.Ros,
                name : '/bob/mask/privacy/override',
                serviceType : 'bob_interfaces/srv/MaskOverrideRequest'
            });

            let request = new ROSLIB.ServiceRequest({
                mask_enabled: mask_enabled
            });

            service.callService(request, function(result) {

                connection.close();
                if (result.success) {
                    if (mask_enabled) {
                        console.log("Privacy Mask enabled successfully");
                        this._store.dispatch(MainActions.Notification({
                            notification: { type: NotificationType.Information, message: "Privacy Mask enabled successfully" }}));
                    } else {
                        console.log("Privacy Mask disabled successfully");
                        this._store.dispatch(MainActions.Notification({
                            notification: { type: NotificationType.Information, message: "Privacy Mask disabled successfully" }}));
                    }
                } else {
                    console.error("Failed to send mask override:", result.message);
                    this._store.dispatch(MainActions.Notification({
                        notification: { type: NotificationType.Error, message: "Failed to enabled/disabled Privacy Mask:" + result.message }}));                
                }
            }.bind(this));
        });          
    }

    public svcDetectionMaskOverride(mask_enabled: boolean = true) {

        let connection = new BobRosConnection();
        connection.open(false); 
        connection.execute(() => {     

            let service = new ROSLIB.Service({
                ros : connection.Ros,
                name : '/bob/mask/detection/override',
                serviceType : 'bob_interfaces/srv/MaskOverrideRequest'
            });

            let request = new ROSLIB.ServiceRequest({
                mask_enabled: mask_enabled
            });

            service.callService(request, function(result) {

                connection.close();
                if (result.success) {
                    if (mask_enabled) {
                        console.log("Detection Mask enabled successfully");
                        this._store.dispatch(MainActions.Notification({
                            notification: { type: NotificationType.Information, message: "Detection Mask enabled successfully" }}));
                    } else {
                        console.log("Detection Mask disabled successfully");
                        this._store.dispatch(MainActions.Notification({
                            notification: { type: NotificationType.Information, message: "Detection Mask disabled successfully" }}));
                    }
                } else {
                    console.error("Failed to send detection mask override:", result.message);
                    this._store.dispatch(MainActions.Notification({
                        notification: { type: NotificationType.Error, message: "Failed to enabled/disabled Detection Mask:" + result.message }}));                
                }
            }.bind(this));
        });          
    }

    public svcDeleteMask(maskFilename: string) {

        let connection = new BobRosConnection();
        connection.open(false); 
        connection.execute(() => {          

            let deleteMaskService = new ROSLIB.Service({
                ros : connection.Ros,
                name : '/bob/webapi/mask/delete/svg',
                serviceType : 'bob_interfaces/srv/MaskSvgDelete'
            });

            let request = new ROSLIB.ServiceRequest({
                file_name: maskFilename,
            });

            deleteMaskService.callService(request, function(result) {

                connection.close();
                if (result.success) {
                    console.log(`Mask ${maskFilename} deleted successfully!`);
                    this._store.dispatch(MainActions.Notification({
                        notification: { type: NotificationType.Information, message: `Mask ${maskFilename} deleted successfully. It can take up to 5 seconds for your changes to take affect.` }}));

                        this._store.dispatch(VisionActions.clearMaskSvg());
                } else {
                    console.error(`Failed to delete mask ${maskFilename}:`, result.message);
                    this._store.dispatch(MainActions.Notification({                        
                        notification: { type: NotificationType.Error, message: `Failed to delete mask ${maskFilename}:` + result.message }}));
                }                
            }.bind(this));
        });           
    }

    public svcSaveMask(svgContent: string, maskFilename: string) {

        let connection = new BobRosConnection();
        connection.open(false); 
        connection.execute(() => {     

            let updateSvgMaskService = new ROSLIB.Service({
                ros : connection.Ros,
                name : '/bob/webapi/mask/update/svg',
                serviceType : 'bob_interfaces/srv/MaskSvgUpdate'
            });

            let request = new ROSLIB.ServiceRequest({
                file_name: maskFilename,
                mask: svgContent
            });

            updateSvgMaskService.callService(request, function(result) {

                connection.close();
                if (result.success) {
                    console.log(`Mask ${maskFilename} saved successfully`);
                    this._store.dispatch(MainActions.Notification({
                        notification: { type: NotificationType.Information, message: `Mask ${maskFilename} successfully saved to server. It can take up to 5 seconds for your new mask to be applied.` }}));

                    this._store.dispatch(VisionActions.setMaskSvg({ mask: svgContent }));
                } else {
                    console.error(`Failed to save mask ${maskFilename}:`, result.message);
                    this._store.dispatch(MainActions.Notification({
                        notification: { type: NotificationType.Error, message: `Failed to save mask ${maskFilename}:` + result.message }}));                
                }                
            }.bind(this));
        });
    }

    public svcGetMask(maskFilename: string) {

        let connection = new BobRosConnection();
        connection.open(false); 
        connection.execute(() => {     

            let updateSvgMaskService = new ROSLIB.Service({
                ros : connection.Ros,
                name : '/bob/webapi/mask/svg',
                serviceType : 'bob_interfaces/srv/MaskGetSvg'
            });

            let request = new ROSLIB.ServiceRequest({
                file_name: maskFilename
            });

            updateSvgMaskService.callService(request, function(result) {

                connection.close();
                if (result.success) {
                    console.log(`Mask ${maskFilename} retrieved successfully`);
                    this._store.dispatch(VisionActions.setMaskSvg({
                        mask: result.mask }));
                } else {
                    console.error(`Failed to retrieve mask ${maskFilename}:`, result.message);
                    this._store.dispatch(MainActions.Notification({
                        notification: { type: NotificationType.Error, message: `Failed to retrieve mask ${maskFilename}:` + result.message }}));                
                }                
            }.bind(this));
        });
    }

    private internal_videoStream(type: ImageStreamType) {

        if (this._videoStreamListener) {
            this.unsubscribeTopic(this._videoStreamListener);
        }

        const topic = this._imageStreamTypes[type];

        this._videoStreamListener = this.subscribeTopic<string>(
            topic, 
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
            ros: this._connection.Ros,
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