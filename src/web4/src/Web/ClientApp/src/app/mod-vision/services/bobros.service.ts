import { Injectable, Inject } from "@angular/core";
import { catchError, Observable, Subject } from "rxjs";
import { map, switchMap } from 'rxjs/operators';

import { RosLibServiceBase } from '../../mod-main/services';

import { AppStateDto } from '../models';

@Injectable({
  providedIn: 'root',
})
export class BobRosService extends RosLibServiceBase {    

    // https://stackoverflow.com/questions/64647388/roslib-with-angular-10
    constructor(@Inject('BASE_URL') baseUrl: string) {
        super(baseUrl);        
    }

    private _annotated = new Subject<any>();
    private _appState = new Subject<any>();

    public Subscribe() {
        this.internal_annotated();
        this.internal_appState();
    }

    public get annotated(): Observable<string>{
        return this._annotated.pipe(
            map((message: any) => this.transformImage(message))
          );        
    }

    public get appState(): Observable<AppStateDto> {
        return this._appState.pipe(
            map((message: any) => this.transformStatus(message))
          );
    }

    private internal_annotated() {
        super.subscribeTopic<string>(
            '/bob/frames/annotated/resized/compressed', 
            'sensor_msgs/msg/CompressedImage', 
            (msg: any) => { this._annotated.next(msg);}
        );
    }

    private internal_appState() {
        super.subscribeTopic<string>(
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
}