import { Injectable } from "@angular/core";
import { catchError, Observable, Subject } from "rxjs";

import * as ROSLIB from 'roslib';

import { RosLibServiceBase } from '../../mod-main/services';

@Injectable({
  providedIn: 'root',
})
export class BobRosService extends RosLibServiceBase {

    // https://stackoverflow.com/questions/64647388/roslib-with-angular-10
    constructor(public urlBob: string) {
        super(urlBob);
    }

    private _chat = new Subject<String>();

    get chat(): Observable<String>{
        return this._chat.asObservable();
    }

    private subscribeToChat() {
        //this._topic = new ROSLIB.Topic({ros: this._ros, name: '/chat', messageType: 'std_msgs/String'});
        //this._topic.subscribe((msg: String) => { this._chat.next(msg);});
        this.subscribeTopic<String>('/chat', 'std_msgs/String', (msg: String) => { this._chat.next(msg);});
    }
}